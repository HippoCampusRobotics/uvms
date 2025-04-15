// Copyright (C) 2023  Niklas Trekel
// Copyright (C) 2024  Vincent Lenz

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

#include "uvms_switching_kin_ctrl_node.hpp"

using std::placeholders::_1;
using namespace std::chrono;

namespace uvms_kin_ctrl {
UVMSSwitchingKinematicControlNode::UVMSSwitchingKinematicControlNode()
    : Node("uvms_switching_kin_ctrl_node") {
  RCLCPP_INFO(this->get_logger(), "Declaring parameters.");
  declareParams();
  initPublishers();
  initTimers();
  initSubscriptions();
  initController();
}

void UVMSSwitchingKinematicControlNode::initTimers() {
  setpoint_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&UVMSSwitchingKinematicControlNode::onSetpointTimeout, this));
}

void UVMSSwitchingKinematicControlNode::initController() {
  startup_controller_ = new UVMSSwitchingKinematicConfigurationControl();
  startup_controller_->initialize(this);
  controller_interface_ = new UVMSKinematicControlInterface();
  controller_interface_->initialize(this);
  startup_controller_->setAUVCmdPublisherPtr(auv_vel_cmd_pub_);
  startup_controller_->setManipulatorCmdPublisherPtr(manipulator_cmd_pub_);
  startup_controller_->setControllerStatusPtr(&controller_status_);
  startup_controller_->initializeParameterCallbacks();
}

void UVMSSwitchingKinematicControlNode::initPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "joint_vel_cmds";
  manipulator_cmd_pub_ =
      create_publisher<alpha_msgs::msg::JointData>(topic, qos);
  topic = "velocity_setpoint";
  auv_vel_cmd_pub_ =
      create_publisher<hippo_msgs::msg::VelocityControlTarget>(topic, qos);

  topic = "pose_endeffector";
  eef_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos);
}
void UVMSSwitchingKinematicControlNode::declareParams() {
  ros_param_utils::getParam(this, publish_on_joint_state_,
                            "publish_on_joint_state", false);
}

void UVMSSwitchingKinematicControlNode::initSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "joint_states";
  state_sub_manipulator_ = create_subscription<sensor_msgs::msg::JointState>(
      topic, qos, std::bind(&UVMSSwitchingKinematicControlNode::onJointState, this, _1));

  topic = "odometry";
  odom_sub_auv_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, qos, std::bind(&UVMSSwitchingKinematicControlNode::onOdometry, this, _1));


  topic = "traj_setpoint";
  eef_traj_sub_ = create_subscription<hippo_msgs::msg::ControlTarget>(
      topic, qos,
      std::bind(&UVMSSwitchingKinematicControlNode::onSetpointTarget, this, _1));

}

alpha_msgs::msg::JointData UVMSSwitchingKinematicControlNode::zeroManipulatorMsg(
    const rclcpp::Time &_stamp) {
  alpha_msgs::msg::JointData msg;
  msg.header.stamp = _stamp;
  std::fill(msg.data.begin(), msg.data.end(), 0.0);
  return msg;
}

void UVMSSwitchingKinematicControlNode::onSetpointTimeout() {
//   if (controller_status_ == ControllerStatus::joint_space_control) {
//     return;
//   }  
  
  if (setpoint_timed_out_) {
    return;
  }

  setpoint_timed_out_ = true;
  RCLCPP_WARN(
      get_logger(),
      "Endeffector trajectory setpoint timed out. Sending zero commands.");
  controller_interface_->onSetpointTimeout();
  if (controller_status_ != ControllerStatus::eef_control) {
    return;
  }
  auto joint_vel_msg = zeroManipulatorMsg(now());
  manipulator_cmd_pub_->publish(joint_vel_msg);
}

void UVMSSwitchingKinematicControlNode::onSetpointTarget(
    const hippo_msgs::msg::ControlTarget::SharedPtr _msg) {
  if(controller_status_ <= ControllerStatus::joint_space_control) {
    // In case of configuration space controller or undefined,
    // eef will not get data
    return;
  }
  
  if (_msg->header.frame_id != "map") {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s",
        ("Endeffector trajectory frame is " +
         std::string(_msg->header.frame_id) + " but only [" +
         std::string(hippo_common::tf2_utils::frame_id::kInertialName) +
         "] is handled. Ignoring..." + _msg->header.frame_id)
            .c_str());
    return;
  }
  if (!got_first_setpoint_) {
    got_first_setpoint_ = true;
  }

  setpoint_timeout_timer_->reset();
  if (setpoint_timed_out_) {
    RCLCPP_INFO(get_logger(),
                "Received endeffector trajectory setpoint. Setpoint not timed "
                "out anymore.");
    setpoint_timed_out_ = false;
  }
  controller_interface_->setSetpointTarget(_msg);
}


void UVMSSwitchingKinematicControlNode::onOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  last_odometry_ = *_msg;
  if (!got_first_auv_state_) {
    got_first_auv_state_ = true;
  }
  if (!publish_on_joint_state_ && got_first_manipulator_state_) {
    publishControlCmds();
  }
}
void UVMSSwitchingKinematicControlNode::onJointState(
    const sensor_msgs::msg::JointState::SharedPtr _msg) {
  last_joint_state_ = *_msg;
  if (!got_first_manipulator_state_) {
    got_first_manipulator_state_ = true;
  }
  if (publish_on_joint_state_ && got_first_auv_state_) {
    publishControlCmds();
  }
}

void UVMSSwitchingKinematicControlNode::publishControlCmds() {
  hippo_msgs::msg::VelocityControlTarget auv_vel_msg;
  alpha_msgs::msg::JointData joint_vel_msg;
  geometry_msgs::msg::PoseStamped eef_pose;

  double dt;
  if (got_first_time_stamp_) {
    dt = (this->now() - last_stamp_).seconds();
  }
  last_stamp_ = this->now();

  switch (controller_status_) {
    case ControllerStatus::undeclared:
      joint_vel_msg = zeroManipulatorMsg(now());
      eef_pose.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      eef_pose.header.stamp = this->now();
      manipulator_cmd_pub_->publish(joint_vel_msg);
      controller_interface_->getEndeffectorPose(
          last_odometry_, last_joint_state_, eef_pose.pose);
      break;
    case ControllerStatus::joint_space_control:
      if (!startup_controller_) {
        RCLCPP_ERROR(this->get_logger(),
                     "Requested startup controller, although it is already "
                     "deleted, this should never happen!");
      }
      startup_controller_->publishControlCommands(last_odometry_,
                                                  last_joint_state_);
      controller_interface_->getEndeffectorPose(
          last_odometry_, last_joint_state_, eef_pose.pose);
      break;
    case ControllerStatus::eef_control:
      if (setpoint_timed_out_ | !got_first_time_stamp_) {
        joint_vel_msg = zeroManipulatorMsg(now());
        manipulator_cmd_pub_->publish(joint_vel_msg);
        controller_interface_->getEndeffectorPose(
            last_odometry_, last_joint_state_, eef_pose.pose);
      } else {
        auv_vel_msg.header.stamp = now();
        auv_vel_msg.header.frame_id =
            hippo_common::tf2_utils::frame_id::BaseLink(this);
        auv_vel_msg.use_acceleration = true;
        joint_vel_msg.header.stamp = auv_vel_msg.header.stamp;
        controller_interface_->getControllerOutputWithDerivative(
            dt, last_odometry_, last_joint_state_, auv_vel_msg.velocity,
            auv_vel_msg.acceleration, joint_vel_msg);
        auv_vel_cmd_pub_->publish(auv_vel_msg);
        manipulator_cmd_pub_->publish(joint_vel_msg);
        controller_interface_->getEndeffectorPose(eef_pose.pose);
      }

      break;
    default:
      RCLCPP_ERROR(
          this->get_logger(),
          "Unknown controller status, sending zero manipulator velocities!");
      joint_vel_msg = zeroManipulatorMsg(now());
      manipulator_cmd_pub_->publish(joint_vel_msg);
      controller_interface_->getEndeffectorPose(
          last_odometry_, last_joint_state_, eef_pose.pose);
      break;
  }
  if (!got_first_time_stamp_) {
    got_first_time_stamp_ = true;
  }
  eef_pose.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  eef_pose.header.stamp = this->now();
  eef_pose_pub_->publish(eef_pose);
}
}  // namespace uvms_kin_ctrl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_kin_ctrl::UVMSSwitchingKinematicControlNode>());
  rclcpp::shutdown();
  return 0;
}
