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

#include "velocity_control_node.hpp"

namespace bluerov_ctrl {
AUVVelocityControlNode::AUVVelocityControlNode()
    : Node("auv_velocity_control_node") {
  RCLCPP_INFO(this->get_logger(), "Declaring parameters.");
  declareParams();
  initController();
  initPublishers();
  initTimers();
  initSubscriptions();
}

void AUVVelocityControlNode::initTimers() {
  setpoint_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&AUVVelocityControlNode::onSetpointTimeout, this));
  state_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&AUVVelocityControlNode::onStateTimeout, this));
}

void AUVVelocityControlNode::initController() {
  ros_param_utils::getParam(this, controller_type_, "controller_type",
                            int(ControllerType::vel_pid_comp));
  switch (controller_type_) {
    case (ControllerType::vel_pid_comp):

      controller_interface_ = new PIDCompInterface();
      break;

    case (ControllerType::vel_model_based_comp):

      controller_interface_ = new ModelBasedCompInterface();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "%s",
                   ("Requested controller of type " +
                    std::to_string(controller_type_) + "not found!")
                       .c_str());
      break;
  }
  controller_interface_->initialize(this);
}

void AUVVelocityControlNode::declareParams() {}

void AUVVelocityControlNode::initPublishers() {
  std::string topic;

  topic = "thrust_setpoint";
  thrust_pub_ = create_publisher<hippo_msgs::msg::ActuatorSetpoint>(
      topic, rclcpp::SensorDataQoS());

  topic = "torque_setpoint";
  torque_pub_ = create_publisher<hippo_msgs::msg::ActuatorSetpoint>(
      topic, rclcpp::SensorDataQoS());
}

void AUVVelocityControlNode::initSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "velocity_setpoint";
  target_sub_ = create_subscription<hippo_msgs::msg::VelocityControlTarget>(
      topic, qos,
      std::bind(&AUVVelocityControlNode::onSetpointTarget, this, _1));

  topic = "odometry";
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, qos, std::bind(&AUVVelocityControlNode::onOdometry, this, _1));

  topic = "estimation_drift_shutdown";
  estimation_drift_shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&AUVVelocityControlNode::onEstimationDriftShutdown, this,
                std::placeholders::_1));
}

hippo_msgs::msg::ActuatorSetpoint AUVVelocityControlNode::zeroMsg(
    rclcpp::Time _stamp) {
  hippo_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = _stamp;
  msg.x = 0.0;
  msg.y = 0.0;
  msg.z = 0.0;
  return msg;
}

void AUVVelocityControlNode::onSetpointTimeout() {
  if (setpoint_timed_out_) {
    return;
  }
  RCLCPP_WARN(get_logger(), "Setpoint timed out. Sending zero commands.");
  setpoint_timed_out_ = true;
  controller_interface_->onTimeout();
  auto thrust_msg = zeroMsg(now());
  auto torque_msg = zeroMsg(now());
  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

void AUVVelocityControlNode::onStateTimeout() {
  if (states_timed_out_) {
    return;
  }
  RCLCPP_WARN(get_logger(), "Odometry timed out. Sending zero commands.");
  states_timed_out_ = true;
  controller_interface_->onTimeout();
  auto thrust_msg = zeroMsg(now());
  auto torque_msg = zeroMsg(now());
  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

void AUVVelocityControlNode::onSetpointTarget(
    const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) {
  if (_msg->header.frame_id !=
      hippo_common::tf2_utils::frame_id::BaseLink(this)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s",
        ("Control Target frame is " + _msg->header.frame_id + "but only " +
         hippo_common::tf2_utils::frame_id::BaseLink(this) +
         "is handled. Ignoring...")
            .c_str());
    return;
  }
  if (!got_first_setpoint_) {
    got_first_setpoint_ = true;
    return;
  }

  setpoint_timeout_timer_->reset();
  if (setpoint_timed_out_) {
    RCLCPP_INFO(get_logger(),
                "Received setpoint. Setpoint not timed out anymore.");
    setpoint_timed_out_ = false;
  }

  controller_interface_->setSetpointTarget(_msg);
  // setpoint_pub_->publish(setpoint_target_);
}

void AUVVelocityControlNode::onOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  if (!got_first_setpoint_) {
    return;
  }
  if (!requested_first_control_output_) {
    requested_first_control_output_ = true;
    last_time_ = this->now();
    return;
  }

  state_timeout_timer_->reset();
  if (states_timed_out_) {
    RCLCPP_INFO(get_logger(),
                "Received odometry update. Setpoint not timed out anymore.");
    states_timed_out_ = false;
  }

  hippo_msgs::msg::ActuatorSetpoint thrust_msg;
  hippo_msgs::msg::ActuatorSetpoint torque_msg;

  if (!estimation_feasible_) {
    thrust_msg = zeroMsg(now());
    torque_msg = zeroMsg(now());
  } else if (setpoint_timed_out_ || !controller_interface_->isOk()) {
    thrust_msg = zeroMsg(now());
    torque_msg = zeroMsg(now());
  } else {
    thrust_msg.header.stamp = now();
    torque_msg.header.stamp = thrust_msg.header.stamp;
    const double dt = double((this->now() - last_time_).nanoseconds()) * 1e-9;
    controller_interface_->getControllerOutput(dt, _msg, thrust_msg,
                                               torque_msg);
  }
  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
  last_time_ = this->now();
  controller_interface_->publishDebugMsgs();
}

void AUVVelocityControlNode::onEstimationDriftShutdown(
    const std_msgs::msg::Bool::SharedPtr _msg) {
  if (_msg->data) {
    estimation_feasible_ = false;
    auto thrust_msg = zeroMsg(now());
    auto torque_msg = zeroMsg(now());
    thrust_pub_->publish(thrust_msg);
    torque_pub_->publish(torque_msg);
    RCLCPP_ERROR(get_logger(), "Estimation drifted, sending zero setpoints!");
  }
}

};  // namespace bluerov_ctrl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bluerov_ctrl::AUVVelocityControlNode>());
  rclcpp::shutdown();
}
