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

#include "bluerov_ctrl/manipulator_compensation_interface.hpp"

namespace bluerov_ctrl {

void ManipulatorCompInterface::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  active_ = false;
  first_wrench_msg_ = false;
  manipulator_compensation_ = new ManipulatorCompensation();
  wrench_sub_ =
      node_ptr->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "force_torque", rclcpp::SystemDefaultsQoS(),
          std::bind(&ManipulatorCompInterface::onWrench, this,
                    std::placeholders::_1));

  // initialize debug publishers
  debug_compensation_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "~/compensation_component", rclcpp::SystemDefaultsQoS());
  timeout_timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), std::chrono::milliseconds(500),
      std::bind(&ManipulatorCompInterface::onTimeout, this));
}

void ManipulatorCompInterface::onWrench(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (!first_wrench_msg_) {
    first_wrench_msg_ = true;
  }
  timeout_timer_->reset();
  if (timed_out_) {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Received manipulator wrench. Not timed out anymore.");
    timed_out_ = false;
  }
  manipulator_compensation_->setCompensationForce(
      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
  manipulator_compensation_->setCompensationTorque(
      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
}

void ManipulatorCompInterface::onTimeout() {
  if (timed_out_) {
    return;
  }
  RCLCPP_WARN(node_ptr_->get_logger(), "Manipulator wrench timed out.");
  timed_out_ = true;
}

void ManipulatorCompInterface::addCompensation(
    const geometry_msgs::msg::Quaternion &msg,
    hippo_msgs::msg::ActuatorSetpoint &out_thrust,
    hippo_msgs::msg::ActuatorSetpoint &out_torque) {
  if (!first_wrench_msg_) {
    return;
  }
  Eigen::Quaterniond orientation;
  Eigen::Vector3d thrust_eigen, torque_eigen;
  hippo_common::convert::RosToEigen(msg, orientation);
  manipulator_compensation_->update(orientation, thrust_eigen, torque_eigen);
  out_thrust.x += thrust_eigen.x();
  out_thrust.y += thrust_eigen.y();
  out_thrust.z += thrust_eigen.z();
  out_torque.x += torque_eigen.x();
  out_torque.y += torque_eigen.y();
  out_torque.z += torque_eigen.z();
  debug_msg_.force.x = thrust_eigen.x();
  debug_msg_.force.y = thrust_eigen.y();
  debug_msg_.force.z = thrust_eigen.z();
  debug_msg_.torque.x = torque_eigen.x();
  debug_msg_.torque.y = torque_eigen.y();
  debug_msg_.torque.z = torque_eigen.z();
}

void ManipulatorCompInterface::publishDebugMsgs() {
  geometry_msgs::msg::WrenchStamped debug_msg;
  debug_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  debug_msg.header.stamp = node_ptr_->now();
  if (isActive()) {
    debug_msg.wrench = debug_msg_;
  } else {
    debug_msg.wrench.force.x = 0.0;
    debug_msg.wrench.force.y = 0.0;
    debug_msg.wrench.force.z = 0.0;
    debug_msg.wrench.torque.x = 0.0;
    debug_msg.wrench.torque.y = 0.0;
    debug_msg.wrench.torque.z = 0.0;
  }
  debug_compensation_pub_->publish(debug_msg);
}

void ManipulatorCompInterface::declareParams() {
  std::string name = "compensate_manipulator";
  std::string descr_text =
      "Boolean to decide if manipulator compensation is used for control "
      "output.";
  rcl_interfaces::msg::ParameterDescriptor descr =
      hippo_common::param_utils::Description(descr_text);
  active_ = false;
  active_ = node_ptr_->declare_parameter(name, active_, descr);

  name = "manipulator_comp.buoyancy_mass";
  descr_text = "Buoyancy added to the AUV to compensate the manipulator weight";
  descr = hippo_common::param_utils::Description(descr_text);
  double added_buoyancy_mass = 0.0;
  added_buoyancy_mass =
      node_ptr_->declare_parameter(name, added_buoyancy_mass, descr);

  std::vector<std::string> suffixes = {"x", "y", "z"};
  Eigen::Vector3d buoyancy_origin;
  for (int i = 0; i < int(suffixes.size()); i++) {
    name = "manipulator_comp.origin." + suffixes[i];
    descr_text =
        "Center of buoyancy added to the AUV to compensate the manipulator "
        "weight";
    descr = hippo_common::param_utils::Description(descr_text);
    buoyancy_origin(i) = 0.0;
    buoyancy_origin(i) =
        node_ptr_->declare_parameter(name, buoyancy_origin(i), descr);
  }

  manipulator_compensation_->setBuoyancyParams(added_buoyancy_mass,
                                               buoyancy_origin);
}

void ManipulatorCompInterface::initializeParamCallbacks() {
  activation_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &ManipulatorCompInterface::onSetActive, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ManipulatorCompInterface::onSetActive(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "compensate_manipulator", active_)) {
      std::cout << "Set parameter " << parameter.get_name() << " to " << active_
                << std::endl;
    }
  }
  return result;
}

}  // namespace bluerov_ctrl