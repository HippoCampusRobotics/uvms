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

#ifndef BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP
#define BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "hippo_common/param_utils.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/control_target.hpp"

namespace uvms_ctrl {
using std::placeholders::_1;

class EstimationWatchdogNode : public rclcpp::Node {
 public:
  EstimationWatchdogNode() : Node("estimation_drift_watchdog_node") {
    RCLCPP_INFO(this->get_logger(), "Declaring parameters.");
    declareParams();
    initPublishers();
    initSubscriptions();
  }

 private:
  void declareParams() {
    std::string name;
    rcl_interfaces::msg::ParameterDescriptor descr;
    std::string descr_text;

    name = "max_position_error";
    descr_text = "Maximum absolute position error before shutdown.";
    descr = hippo_common::param_utils::Description(descr_text, true);
    max_position_error_ = 0.2;
    max_position_error_ = declare_parameter(name, max_position_error_, descr);

    name = "max_attitude_error";
    descr_text = "Maximum absolute attitude error before shutdown.";
    descr = hippo_common::param_utils::Description(descr_text, true);
    max_attitude_error_ = 0.2;
    max_attitude_error_ = declare_parameter(name, max_attitude_error_, descr);
  }

  void initPublishers() {
    std::string topic;

    topic = "estimation_drift_shutdown";
    shutdown_pub_ =
        create_publisher<std_msgs::msg::Bool>(topic, rclcpp::SensorDataQoS());

    topic = "~/position_error";
    position_error_debug_pub_ = create_publisher<std_msgs::msg::Float64>(
        topic, rclcpp::SensorDataQoS());
    topic = "~/attitude_error";
    attitude_error_debug_pub_ = create_publisher<std_msgs::msg::Float64>(
        topic, rclcpp::SensorDataQoS());
  }

  void initSubscriptions() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "traj_setpoint";
    target_sub_ = create_subscription<hippo_msgs::msg::ControlTarget>(
        topic, qos,
        std::bind(&EstimationWatchdogNode::onSetpointTarget, this, _1));

    topic = "pose_endeffector";
    eef_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, qos,
        std::bind(&EstimationWatchdogNode::onEndeffectorPose, this, _1));
  }

  void onSetpointTarget(const hippo_msgs::msg::ControlTarget::SharedPtr _msg) {
    if (_msg->header.frame_id !=
        hippo_common::tf2_utils::frame_id::kInertialName) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "%s",
          ("Target frame is " + _msg->header.frame_id + "but only " +
           hippo_common::tf2_utils::frame_id::kInertialName +
           "is handled. Ignoring...")
              .c_str());
      return;
    }
    if (!got_first_setpoint_) {
      got_first_setpoint_ = true;
      return;
    }
    hippo_common::convert::RosToEigen(_msg->position, position_des_);
    hippo_common::convert::RosToEigen(_msg->attitude, attitude_des_);
    mask_ = Eigen::Matrix<double, 6, 1>::Ones();
    if ((_msg->mask & _msg->IGNORE_POSITION_X) == _msg->IGNORE_POSITION_X) {
      mask_(0) = 0.0;
    }
    if ((_msg->mask & _msg->IGNORE_POSITION_Y) == _msg->IGNORE_POSITION_Y) {
      mask_(1) = 0.0;
    }
    if ((_msg->mask & _msg->IGNORE_POSITION_Z) == _msg->IGNORE_POSITION_Z) {
      mask_(2) = 0.0;
    }
    if ((_msg->mask & _msg->IGNORE_ATTITUDE_X) == _msg->IGNORE_ATTITUDE_X) {
      mask_(3) = 0.0;
    }
    if ((_msg->mask & _msg->IGNORE_ATTITUDE_Y) == _msg->IGNORE_ATTITUDE_Y) {
      mask_(4) = 0.0;
    }
    if ((_msg->mask & _msg->IGNORE_ATTITUDE_Z) == _msg->IGNORE_ATTITUDE_Z) {
      mask_(5) = 0.0;
    }
  }

  void skew(const Eigen::Ref<const Eigen::Vector3d> &x,
            Eigen::Ref<Eigen::Matrix3d> x_tilde) {
    x_tilde << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  }

  Eigen::Vector3d quaternionError(const Eigen::Quaterniond &des,
                                  const Eigen::Quaterniond &state) {
    Eigen::Matrix3d v_des_tilde;
    skew(des.vec(), v_des_tilde);
    double scalar_error =
        des.w() * state.w() + des.vec().transpose() * state.vec();
    Eigen::Vector3d out = state.w() * des.vec() - des.w() * state.vec() -
                          v_des_tilde * state.vec();
    out *= int(scalar_error >= 0) * 1 +
           int(scalar_error < 0) *
               -1;  // flip sign if scalar error negative, quaternion convention
                    // requires error quaternion to be have positive scalar
    return out;
  }

  void onEndeffectorPose(
      const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
    if (!got_first_setpoint_) {
      return;
    }
    if (!got_first_eef_pose_) {
      got_first_eef_pose_ = true;
      return;
    }
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    hippo_common::convert::RosToEigen(_msg->pose.position, position);
    hippo_common::convert::RosToEigen(_msg->pose.orientation, attitude);

    const double position_error =
        (mask_.segment<3>(0).array() * (position_des_ - position).array())
            .matrix()
            .norm();
    const double attitude_error = (mask_.segment<3>(3).array() *
                                   (attitude_des_.toRotationMatrix().inverse() *
                                    quaternionError(attitude_des_, attitude))
                                       .array())
                                      .matrix()
                                      .norm();
    if (feasible_ && (position_error > max_position_error_ ||
                      attitude_error > max_attitude_error_)) {
      feasible_ = false;
    }
    std_msgs::msg::Bool shutdown_msg;
    shutdown_msg.data = !feasible_;
    shutdown_pub_->publish(shutdown_msg);
    std_msgs::msg::Float64 debug_msg;
    debug_msg.data = position_error;
    position_error_debug_pub_->publish(debug_msg);
    debug_msg.data = attitude_error;
    attitude_error_debug_pub_->publish(debug_msg);
  }

  //////////////////////////////////////////////////////////////////////////////
  // ros params
  //////////////////////////////////////////////////////////////////////////////

  Eigen::Vector3d position_des_;
  Eigen::Quaterniond attitude_des_;
  Eigen::Matrix<double, 6, 1> mask_ = Eigen::Matrix<double, 6, 1>::Ones();
  double max_position_error_;
  double max_attitude_error_;
  bool feasible_ = true;

  bool got_first_setpoint_{false};
  bool got_first_eef_pose_{false};

  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      position_error_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      attitude_error_debug_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ControlTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      eef_pose_sub_;
};
}  // namespace uvms_ctrl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_ctrl::EstimationWatchdogNode>());
  rclcpp::shutdown();
}

#endif  // BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP
