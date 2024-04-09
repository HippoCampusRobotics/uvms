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

#include "uvms_kin_ctrl_node_configuration_space.hpp"

namespace uvms_kin_ctrl {

UVMSKinematicConfigurationControl::UVMSKinematicConfigurationControl() {}

void UVMSKinematicConfigurationControl::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  initController();
  initTimers();
  initSubscriptions();
}

void UVMSKinematicConfigurationControl::resetConnections() {
  setpoint_timeout_timer_.reset();
  manipulator_cmd_pub_.reset();
  auv_vel_cmd_pub_.reset();
  setpoint_sub_.reset();
}

void UVMSKinematicConfigurationControl::initializeParameterCallbacks() {
  auv_position_controller_interface_->initializeParamCallbacks();
  auv_attitude_controller_interface_->initializeParamCallbacks();
  manipulator_controller_interface_->initializeParameterCallbacks();
}

void UVMSKinematicConfigurationControl::publishControlCommands(
    const nav_msgs::msg::Odometry &auv_msg,
    const sensor_msgs::msg::JointState &manipulator_msg) {
  hippo_control_msgs::msg::VelocityControlTarget out_auv_msg;
  alpha_msgs::msg::JointData out_manipulator_msg;
  if (!got_first_setpoint_ || setpoint_timed_out_) {
    out_manipulator_msg = zeroManipulatorMsg(node_ptr_->now());
    manipulator_cmd_pub_->publish(out_manipulator_msg);
    return;
  }

  out_auv_msg.header.stamp = node_ptr_->now();
  out_auv_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  auv_position_controller_interface_->update(auv_msg.pose.pose.position,
                                             auv_msg.pose.pose.orientation,
                                             out_auv_msg.velocity.linear);
  out_auv_msg.acceleration.linear.x = 0.0;
  out_auv_msg.acceleration.linear.y = 0.0;
  out_auv_msg.acceleration.linear.z = 0.0;
  auv_attitude_controller_interface_->update(auv_msg.pose.pose.orientation,
                                             out_auv_msg.velocity.angular);
  out_auv_msg.acceleration.angular.x = 0.0;
  out_auv_msg.acceleration.angular.y = 0.0;
  out_auv_msg.acceleration.angular.z = 0.0;

  out_manipulator_msg.header.stamp = out_auv_msg.header.stamp;
  sensor_msgs::msg::JointState::SharedPtr manipulator_msg_ptr =
      std::make_shared<sensor_msgs::msg::JointState>(manipulator_msg);
  manipulator_controller_interface_->update(manipulator_msg_ptr,
                                            out_manipulator_msg);

  auv_vel_cmd_pub_->publish(out_auv_msg);
  manipulator_cmd_pub_->publish(out_manipulator_msg);
}
void UVMSKinematicConfigurationControl::initTimers() {
  setpoint_timeout_timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), std::chrono::milliseconds(500),
      std::bind(&UVMSKinematicConfigurationControl::onSetpointTimeout, this));
}

void UVMSKinematicConfigurationControl::initController() {
  auv_position_controller_interface_ = new bluerov_ctrl::PosPModuleInterface();
  auv_attitude_controller_interface_ =
      new bluerov_ctrl::AttSkewSymmetricPModuleInterface();
  manipulator_controller_interface_ =
      new alpha_ctrl::JointKinematicControlInterface();
  auv_position_controller_interface_->initialize(node_ptr_);
  auv_attitude_controller_interface_->initialize(node_ptr_);
  manipulator_controller_interface_->initialize(node_ptr_);
  auv_position_controller_interface_->declareParams();
  auv_attitude_controller_interface_->declareParams();
}

void UVMSKinematicConfigurationControl::initSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  topic = "traj_setpoint_uvms";
  setpoint_sub_ =
      node_ptr_->create_subscription<uvms_msgs::msg::UVMSControlTarget>(
          topic, qos,
          std::bind(&UVMSKinematicConfigurationControl::onSetpointTarget, this,
                    _1));
}

alpha_msgs::msg::JointData
UVMSKinematicConfigurationControl::zeroManipulatorMsg(
    const rclcpp::Time &_stamp) {
  alpha_msgs::msg::JointData msg;
  msg.header.stamp = _stamp;
  std::fill(msg.data.begin(), msg.data.end(), 0.0);
  return msg;
}

void UVMSKinematicConfigurationControl::onSetpointTimeout() {
  if (setpoint_timed_out_) {
    return;
  }
  RCLCPP_WARN(node_ptr_->get_logger(),
              "Configuration space setpoint timed out. Sending zero commands.");
  setpoint_timed_out_ = true;
  if (*controller_status_ptr_ != ControllerStatus::joint_space_control) {
    return;
  }
  auto joint_vel_msg = zeroManipulatorMsg(node_ptr_->now());
  manipulator_cmd_pub_->publish(joint_vel_msg);
}

void UVMSKinematicConfigurationControl::onSetpointTarget(
    const uvms_msgs::msg::UVMSControlTarget::SharedPtr _msg) {
  if (_msg->header.frame_id !=
      hippo_common::tf2_utils::frame_id::kInertialName) {
    RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000, "%s",
        ("Configuration trajectory frame is [%s] but only [" +
         std::string(hippo_common::tf2_utils::frame_id::kInertialName) +
         "] is handled. Ignoring..." + _msg->header.frame_id)
            .c_str());
    return;
  }
  if (!got_first_setpoint_) {
    got_first_setpoint_ = true;
  }

  if (*controller_status_ptr_ != ControllerStatus::joint_space_control) {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Activating configuration space controller.");
    *controller_status_ptr_ = ControllerStatus::joint_space_control;
  }

  setpoint_timeout_timer_->reset();
  if (setpoint_timed_out_) {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Received configuration space setpoint. Setpoint not timed out "
                "anymore.");
    setpoint_timed_out_ = false;
  }

  hippo_control_msgs::msg::ControlTarget auv_setpoint = _msg->auv;
  hippo_control_msgs::msg::ControlTarget::SharedPtr auv_setpoint_ptr =
      std::make_shared<hippo_control_msgs::msg::ControlTarget>(auv_setpoint);
  std::lock_guard<std::mutex> lock(mutex_);
  auv_position_controller_interface_->setControlTarget(auv_setpoint_ptr);
  auv_attitude_controller_interface_->setControlTarget(auv_setpoint_ptr);
  manipulator_controller_interface_->setPositionTarget(
      _msg->manipulator.position);
  manipulator_controller_interface_->setVelocityTarget(
      _msg->manipulator.velocity);
}

}  // namespace uvms_kin_ctrl
