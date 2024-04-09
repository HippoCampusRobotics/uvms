// Copyright (C) 2023  Niklas Trekel

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control_msgs/msg/actuator_controls.hpp>
#include <hippo_control_msgs/msg/thruster_forces.hpp>
#include <hippo_msgs/msg/esc_rpms.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "alpha_msgs/msg/joint_data.hpp"
#include "hippo_control_msgs/msg/velocity_control_target.hpp"

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace hippo_msgs::msg;
using namespace hippo_control_msgs::msg;
using namespace ignition;
using namespace nav_msgs::msg;
namespace gz_msgs = ignition::msgs;
using std::placeholders::_1;

class Bridge {
 public:
  Bridge() {
    node_topics = (rclcpp::node_interfaces::NodeTopics *)ros_node_
                      ->get_node_topics_interface()
                      .get();

    std::string parameterName = "simulate_kinematics";
    const bool default_value = false;
    ros_node_->declare_parameter<bool>(parameterName, default_value);
    if (!ros_node_->get_parameter(parameterName, simulate_kinematics_)) {
      RCLCPP_WARN(ros_node_->get_logger(), "%s",
                  ("Parameter " + parameterName +
                   " not properly loaded, use default "
                   "value " +
                   std::to_string(default_value))
                      .c_str());
    }

    // CreateClockBridge();
    CreateGroundTruthBridge();
    CreateImuBridge();
    CreateJointStateBridge();
    CreateJointCmdBridge();

    if (simulate_kinematics_) {
      CreateVelocityCommandBridge();
    } else {
      CreateAccelerationsBridge();
      CreateThrusterBridge();
      CreateWorldLinearAccelerationBridge();
      CreateJointAccelerationBridge();
      CreateBaseForceTorqueBridge();
    }
  }

  void CreateClockBridge() {
    bool use_sim_time;
    bool default_value = true;
    std::string parameterName = "use_sim_time";
    if (!ros_node_->has_parameter(parameterName)) {
      ros_node_->declare_parameter<bool>(parameterName, default_value);
    }
    if (!ros_node_->get_parameter<bool>(parameterName, use_sim_time)) {
      RCLCPP_WARN(ros_node_->get_logger(), "%s",
                  ("Parameter " + parameterName +
                   " not properly loaded, use default "
                   "value " +
                   std::to_string(default_value))
                      .c_str());
    }
    if (!use_sim_time) {
      return;
    }
    clock_pub_ = ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::SystemDefaultsQoS());
    std::function<void(const ignition::msgs::Clock &)> f =
        std::bind(&Bridge::OnClock, this, _1);
    gz_node_->Subscribe("/clock", f);
  }

  void CreateAccelerationsBridge() {
    std::string name;
    name = node_topics->resolve_topic_name("ground_truth/accelerations");
    accelerations_pub_ =
        ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            name, rclcpp::SystemDefaultsQoS());
    gz_node_->Subscribe(name, &Bridge::OnAccelerations, this);
  }

  void CreateWorldLinearAccelerationBridge() {
    std::string name;
    name = node_topics->resolve_topic_name(
        "ground_truth/world_linear_acceleration");
    world_linear_acceleration_pub_ =
        ros_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            name, rclcpp::SystemDefaultsQoS());
    gz_node_->Subscribe(name, &Bridge::OnWorldLinearAcceleration, this);
  }

  void CreateGroundTruthBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("ground_truth/pose");
    pose_pub_ = ros_node_->create_publisher<PoseStamped>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnPose, this);
    RCLCPP_INFO(ros_node_->get_logger(), "Create gz subscription: [%s]",
                topic_name.c_str());

    topic_name = node_topics->resolve_topic_name("ground_truth/odometry");
    odometry_pub_ = ros_node_->create_publisher<Odometry>(topic_name, qos);
    gz_node_->Subscribe(topic_name, &Bridge::OnOdometry, this);
  }

  void CreateImuBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("imu");
    imu_pub_ = ros_node_->create_publisher<Imu>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnImu, this);
  }

  void CreateThrusterBridge() {
    rclcpp::SystemDefaultsQoS qos;
    rpm_pub_ = ros_node_->create_publisher<EscRpms>(
        node_topics->resolve_topic_name("esc_rpm"), qos);
    thruster_forces_pub_ =
        ros_node_->create_publisher<ThrusterForces>("thrusts", qos);
    for (size_t i = 0; i < ActuatorControls().control.size(); i++) {
      std::string topic_name;
      qos.keep_last(50);
      std::string topic_prefix =
          node_topics->resolve_topic_name("thruster_") + std::to_string(i);
      topic_name = topic_prefix + "/throttle_cmd";

      // gazebo publisher
      throttle_cmd_pubs_[i] = gz_node_->Advertise<gz_msgs::Double>(topic_name);

      topic_name = topic_prefix + "/rpm";
      std::function<void(const gz_msgs::Double &)> f =
          std::bind(&Bridge::OnThrusterRpm, this, _1, i);
      gz_node_->Subscribe(topic_name, f);
      topic_name = topic_prefix + "/thrust";
      f = std::bind(&Bridge::OnThrust, this, _1, i);
      gz_node_->Subscribe(topic_name, f);
    }
    thrust_sub_ = ros_node_->create_subscription<ActuatorControls>(
        "thruster_command", qos,
        std::bind(&Bridge::OnThrusterCommand, this, _1));
  }

  void CreateVelocityCommandBridge() {
    vel_cmd_sub_ = ros_node_->create_subscription<
        hippo_control_msgs::msg::VelocityControlTarget>(
        "velocity_setpoint", rclcpp::SystemDefaultsQoS(),
        std::bind(&Bridge::OnVelocityCommand, this, _1));
    vel_cmd_pub_ = gz_node_->Advertise<ignition::msgs::Twist>(
        std::string(ros_node_->get_namespace()) + "/vel_cmds");
  }

  void CreateJointAccelerationBridge() {
    joint_accel_pub_ = ros_node_->create_publisher<alpha_msgs::msg::JointData>(
        "joint_accelerations_raw", rclcpp::SystemDefaultsQoS());
    std::function<void(const ignition::msgs::Double_V &)> f =
        std::bind(&Bridge::OnJointAcceleration, this, _1);
    gz_node_->Subscribe(
        std::string(ros_node_->get_namespace()) + "/joint_accelerations", f);
  }

  void CreateJointStateBridge() {
    std::string parameterName = "update_frequency";
    const double default_value = 50;
    ros_node_->declare_parameter<double>(parameterName, default_value);
    if (!ros_node_->get_parameter(parameterName,
                                  joint_state_update_frequency_)) {
      RCLCPP_WARN(ros_node_->get_logger(), "%s",
                  ("Parameter " + parameterName +
                   " not properly loaded, use default "
                   "value " +
                   std::to_string(default_value))
                      .c_str());
    }
    std::function<void(const ignition::msgs::Model &)> f =
        std::bind(&Bridge::OnJointState, this, _1);
    gz_node_->Subscribe(
        std::string(ros_node_->get_namespace()) + "/joint_states", f);
    RCLCPP_INFO(
        ros_node_->get_logger(), "%s",
        (std::string(ros_node_->get_namespace()) + "/joint_states").c_str());
    joint_state_pub_ =
        ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states_raw", rclcpp::SystemDefaultsQoS());
    last_joint_state_update_ = ros_node_->now();
  }

  void CreateJointCmdBridge() {
    joint_cmd_pubs_.resize(alpha_msgs::msg::JointData().data.size());
    std::string topic_name;
    for (size_t i = 0; i < alpha_msgs::msg::JointData().data.size(); i++) {
      topic_name = std::string(ros_node_->get_namespace()) + "/joint/" +
                   "joint_" + std::to_string(i + 1) + "/cmd";
      joint_cmd_pubs_[i] =
          gz_node_->Advertise<ignition::msgs::Double>(topic_name);
    }
    std::string topic_sub_name;

    if (simulate_kinematics_) {
      topic_sub_name = "joint_vel_cmds";
    } else {
      topic_sub_name = "joint_torque_cmds";
    }

    joint_cmd_sub_ = ros_node_->create_subscription<alpha_msgs::msg::JointData>(
        topic_sub_name, rclcpp::SystemDefaultsQoS(),
        std::bind(&Bridge::OnJointCommand, this, _1));
  }

  void CreateBaseForceTorqueBridge() {
    base_ft_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Wrench>(
        "force_torque_gz", rclcpp::SystemDefaultsQoS());
    std::function<void(const ignition::msgs::Wrench &)> f =
        std::bind(&Bridge::OnBaseForceTorque, this, _1);
    gz_node_->Subscribe(
        std::string(ros_node_->get_namespace()) + "/force_torque_gz", f);
  }

  void OnClock(const ignition::msgs::Clock &msg) {
    rosgraph_msgs::msg::Clock ros_msg;
    ros_gz_bridge::convert_gz_to_ros(msg, ros_msg);
    clock_pub_->publish(ros_msg);
  }

  void OnAccelerations(const gz_msgs::Twist &_msg) {
    geometry_msgs::msg::TwistStamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg.header(), ros_msg.header);
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg.twist);
    accelerations_pub_->publish(ros_msg);
  }

  void OnWorldLinearAcceleration(const gz_msgs::Vector3d &_msg) {
    geometry_msgs::msg::Vector3Stamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg.vector);
    ros_gz_bridge::convert_gz_to_ros(_msg.header(), ros_msg.header);
    world_linear_acceleration_pub_->publish(ros_msg);
  }

  void OnPressure(const gz_msgs::FluidPressure &_msg) {
    sensor_msgs::msg::FluidPressure ros_msg;
    ros_msg.header.frame_id = "map";
    ros_msg.header.stamp = ros_node_->now();
    ros_msg.fluid_pressure = _msg.pressure();
    pressure_pub_->publish(ros_msg);
  }

  void OnImu(const gz_msgs::IMU &_msg) {
    sensor_msgs::msg::Imu ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    imu_pub_->publish(ros_msg);
  }

  void OnThrusterRpm(const gz_msgs::Double &_msg, size_t _i) {
    if (_i > thrusters_rpm_msg_.rpms.size() - 1) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thrusters_rpm_msg_.rpms[_i] = _msg.data();
    if (_i == 0) {
      thrusters_rpm_msg_.header.stamp = ros_node_->get_clock()->now();
      rpm_pub_->publish(thrusters_rpm_msg_);
    }
  }

  void OnThrust(const gz_msgs::Double &_msg, size_t _i) {
    if (_i > thruster_forces_msg_.force.size() - 1) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thruster_forces_msg_.force[_i] = _msg.data();
    if (_i == 0) {
      thruster_forces_msg_.header.stamp = ros_node_->now();
      thruster_forces_pub_->publish(thruster_forces_msg_);
    }
  }

  void OnVelocityCommand(
      const hippo_control_msgs::msg::VelocityControlTarget::SharedPtr ros_msg) {
    ignition::msgs::Twist msg;
    msg.mutable_linear()->set_x(ros_msg->velocity.linear.x);
    msg.mutable_linear()->set_y(ros_msg->velocity.linear.y);
    msg.mutable_linear()->set_z(ros_msg->velocity.linear.z);
    msg.mutable_angular()->set_x(ros_msg->velocity.angular.x);
    msg.mutable_angular()->set_y(ros_msg->velocity.angular.y);
    msg.mutable_angular()->set_z(ros_msg->velocity.angular.z);
    vel_cmd_pub_.Publish(msg);
    // model_vel_cmd_pub_.Publish(msg);
  }

  void OnPose(const gz_msgs::Pose &_msg) {
    geometry_msgs::msg::PoseStamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    pose_pub_->publish(ros_msg);
  }

  void OnOdometry(const gz_msgs::Odometry &_msg) {
    Odometry ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    odometry_pub_->publish(ros_msg);
  }

  void OnThrusterCommand(const ActuatorControls::SharedPtr _msg) {
    if (!(_msg->control.size() == throttle_cmd_pubs_.size())) {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "ActuatControls and publisher map do not have same size!");
      return;
    }
    for (unsigned int i = 0; i < throttle_cmd_pubs_.size(); ++i) {
      gz_msgs::Double gz_msg;
      gz_msg.set_data(_msg->control[i]);
      throttle_cmd_pubs_[i].Publish(gz_msg);
    }
  }

  void OnJointCommand(const alpha_msgs::msg::JointData::SharedPtr msg) {
    if (msg->data.size() != joint_cmd_pubs_.size()) {
      RCLCPP_ERROR(
          ros_node_->get_logger(),
          "Joint Command Message and num. of publishers aren't equal!");
      return;
    }
    ignition::msgs::Double ign_msg;
    for (size_t i = 0; i < joint_cmd_pubs_.size(); i++) {
      ign_msg.set_data(msg->data[i]);
      joint_cmd_pubs_[i].Publish(ign_msg);
    }
  }

  void OnJointAcceleration(const ignition::msgs::Double_V &msg) {
    alpha_msgs::msg::JointData ros_msg;
    if (msg.data_size() > int(ros_msg.data.size())) {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "Ignition message size for joint accelerations greater than "
                   "ros msg size");
    }
    ros_msg.header.stamp = ros_node_->now();
    for (int i = 0; i < msg.data_size(); i++) {
      ros_msg.data[i] = msg.data()[i];
    }
    joint_accel_pub_->publish(ros_msg);
  }

  void OnJointState(const ignition::msgs::Model &msg) {
    if ((ros_node_->now() - last_joint_state_update_).seconds() >=
        1 / joint_state_update_frequency_) {
      sensor_msgs::msg::JointState ros_msg;
      ros_msg.position.resize(msg.joint_size());
      ros_msg.velocity.resize(msg.joint_size());
      ros_msg.effort.resize(msg.joint_size());
      ros_gz_bridge::convert_gz_to_ros(msg.header(), ros_msg.header);
      for (auto i = 0; i < msg.joint_size(); ++i) {
        ros_msg.name.push_back(msg.joint(i).name());
        if (msg.joint(i).axis1().position() >= 0.0) {
          ros_msg.position[i] = fmod(msg.joint(i).axis1().position(), 2 * M_PI);
        } else {
          ros_msg.position[i] =
              msg.joint(i).axis1().position() +
              (std::floor(std::abs(msg.joint(i).axis1().position()) /
                          (2 * M_PI)) +
               1) *
                  2 * M_PI;
        }
        ros_msg.velocity[i] = msg.joint(i).axis1().velocity();
        ros_msg.effort[i] = msg.joint(i).axis1().force();
      }

      last_joint_state_update_ = rclcpp::Time(ros_msg.header.stamp);
      joint_state_pub_->publish(ros_msg);
    }
  }

  void OnBaseForceTorque(const ignition::msgs::Wrench &msg) {
    geometry_msgs::msg::Wrench ros_msg;
    ros_gz_bridge::convert_gz_to_ros(msg, ros_msg);
    base_ft_pub_->publish(ros_msg);
  }

  void Run() { rclcpp::spin(ros_node_); }

 private:
  rclcpp::Node::SharedPtr ros_node_ = std::make_shared<rclcpp::Node>("bridge");
  std::shared_ptr<transport::Node> gz_node_ =
      std::make_shared<transport::Node>();
  rclcpp::node_interfaces::NodeTopics *node_topics;

  std::mutex mutex_;

  bool simulate_kinematics_;

  EscRpms thrusters_rpm_msg_;
  ThrusterForces thruster_forces_msg_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<ThrusterForces>::SharedPtr thruster_forces_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      world_linear_acceleration_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      accelerations_pub_;

  std::map<int, transport::Node::Publisher> throttle_cmd_pubs_;
  rclcpp::Publisher<EscRpms>::SharedPtr rpm_pub_;
  rclcpp::Subscription<hippo_control_msgs::msg::ActuatorControls>::SharedPtr
      thrust_sub_;
  rclcpp::Subscription<
      hippo_control_msgs::msg::VelocityControlTarget>::SharedPtr vel_cmd_sub_;
  // ignition::transport::Node::Publisher linear_vel_cmd_pub_;
  // ignition::transport::Node::Publisher angular_vel_cmd_pub_;
  ignition::transport::Node::Publisher vel_cmd_pub_;
  // ignition::transport::Node::Publisher model_vel_cmd_pub_;

  std::string manipulator_topic_prefix_;
  std::string auv_topic_prefix_;

  double joint_state_update_frequency_;
  rclcpp::Time last_joint_state_update_;
  rclcpp::Publisher<alpha_msgs::msg::JointData>::SharedPtr joint_accel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr base_ft_pub_;
  std::vector<ignition::transport::Node::Publisher> joint_cmd_pubs_;
  rclcpp::Subscription<alpha_msgs::msg::JointData>::SharedPtr joint_cmd_sub_;
};

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto bridge = Bridge();
  bridge.Run();
}
