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

#include "bluerov_ctrl/position_p_module_interface.hpp"

namespace bluerov_ctrl {
void PosPModuleInterface::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  controller_ = new PosPControlModule();
}

void PosPModuleInterface::update(const geometry_msgs::msg::Point &pos,
                                 const geometry_msgs::msg::Quaternion &att,
                                 geometry_msgs::msg::Vector3 &out_vel) {
  Eigen::Vector3d pos_eigen, out_vel_eigen;
  hippo_common::convert::RosToEigen(pos, pos_eigen);
  Eigen::Quaterniond att_eigen;
  hippo_common::convert::RosToEigen(att, att_eigen);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_->update(pos_eigen, att_eigen, out_vel_eigen);
  }
  out_vel.x = out_vel_eigen.x();
  out_vel.y = out_vel_eigen.y();
  out_vel.z = out_vel_eigen.z();
}

void PosPModuleInterface::setControlTarget(
    const hippo_msgs::msg::ControlTarget::SharedPtr msg) {
  Eigen::Vector3d mask = Eigen::Vector3d::Ones();
  if ((msg->mask & msg->IGNORE_POSITION_X) == msg->IGNORE_POSITION_X) {
    mask(0) = 0.0;
  }
  if ((msg->mask & msg->IGNORE_POSITION_Y) == msg->IGNORE_POSITION_Y) {
    mask(1) = 0.0;
  }
  if ((msg->mask & msg->IGNORE_POSITION_Z) == msg->IGNORE_POSITION_Z) {
    mask(2) = 0.0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  controller_->setPositionTarget(msg->position.x, msg->position.y,
                                 msg->position.z);
  controller_->setVelocityTarget(msg->velocity.x, msg->velocity.y,
                                 msg->velocity.z);
  controller_->setPositionMask(mask);
}

void PosPModuleInterface::declareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "pos_p_module.gain.p.x";
  descr_text = "Proportional gain for x-position.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_p_x_ = 1.0;
    gain_p_x_ = node_ptr_->declare_parameter(name, gain_p_x_, descr);
    controller_->setPgainX(gain_p_x_);
  }

  name = "pos_p_module.gain.p.y";
  descr_text = "Proportional gain for y-position.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_p_y_ = 1.0;
    gain_p_y_ = node_ptr_->declare_parameter(name, gain_p_y_, descr);
    controller_->setPgainY(gain_p_y_);
  }

  name = "pos_p_module.gain.p.z";
  descr_text = "Proportional gain for z-position.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_p_z_ = 1.0;
    gain_p_z_ = node_ptr_->declare_parameter(name, gain_p_z_, descr);
    controller_->setPgainZ(gain_p_z_);
  }
}

void PosPModuleInterface::initializeParamCallbacks() {
  p_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &PosPModuleInterface::onSetPgains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult PosPModuleInterface::onSetPgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "pos_p_module.gain.p.x", gain_p_x_)) {
      controller_->setPgainX(gain_p_x_);
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "pos_p_module.gain.p.y", gain_p_y_)) {
      controller_->setPgainY(gain_p_y_);
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "pos_p_module.gain.p.z", gain_p_z_)) {
      controller_->setPgainZ(gain_p_z_);
      continue;
    }
  }
  return result;
}
}  // namespace bluerov_ctrl