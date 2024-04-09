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

#include "bluerov_ctrl/attitude_skew_symmetric_p_module_interface.hpp"

namespace bluerov_ctrl {
void AttSkewSymmetricPModuleInterface::initialize(rclcpp::Node *node_ptr) {
  controller_ = new AttSkewSymmetricPControlModule();
  node_ptr_ = node_ptr;
}

void AttSkewSymmetricPModuleInterface::update(
    const geometry_msgs::msg::Quaternion &att,
    geometry_msgs::msg::Vector3 &out_rates) {
  Eigen::Quaterniond att_eigen;
  hippo_common::convert::RosToEigen(att, att_eigen);
  Eigen::Vector3d rates_des_eigen;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_->update(att_eigen, rates_des_eigen);
  }
  out_rates.x = rates_des_eigen.x();
  out_rates.y = rates_des_eigen.y();
  out_rates.z = rates_des_eigen.z();
}

void AttSkewSymmetricPModuleInterface::setControlTarget(
    const hippo_control_msgs::msg::ControlTarget::SharedPtr msg) {
  Eigen::Quaterniond attitude;
  hippo_common::convert::RosToEigen(msg->attitude, attitude);
  Eigen::Vector3d mask = Eigen::Vector3d::Ones();
  if ((msg->mask & msg->IGNORE_ATTITUDE_X) == msg->IGNORE_ATTITUDE_X) {
    mask(0) = 0.0;
  }
  if ((msg->mask & msg->IGNORE_ATTITUDE_Y) == msg->IGNORE_ATTITUDE_Y) {
    mask(1) = 0.0;
  }
  if ((msg->mask & msg->IGNORE_ATTITUDE_Z) == msg->IGNORE_ATTITUDE_Z) {
    mask(2) = 0.0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  controller_->setOrientationTarget(attitude);
  controller_->setAngularVelocityTarget(msg->angular_velocity.x,
                                        msg->angular_velocity.y,
                                        msg->angular_velocity.z);
  controller_->setAttitudeMask(mask);
}

void AttSkewSymmetricPModuleInterface::declareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "att_p_module.gain.roll.p";
  descr_text = "Proportional gain for roll.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_roll_p_ = 1.0;
    gain_roll_p_ = node_ptr_->declare_parameter(name, gain_roll_p_, descr);
    controller_->setRollGainP(gain_roll_p_);
  }

  name = "att_p_module.gain.pitch.p";
  descr_text = "Proportional gain for pitch.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_pitch_p_ = 1.0;
    gain_pitch_p_ = node_ptr_->declare_parameter(name, gain_pitch_p_, descr);
    controller_->setPitchGainP(gain_pitch_p_);
  }

  name = "att_p_module.gain.yaw.p";
  descr_text = "Proportional gain for yaw.";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_yaw_p_ = 1.0;
    gain_yaw_p_ = node_ptr_->declare_parameter(name, gain_yaw_p_, descr);
    controller_->setYawGainP(gain_yaw_p_);
  }
}

void AttSkewSymmetricPModuleInterface::initializeParamCallbacks() {
  p_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&AttSkewSymmetricPModuleInterface::onSetPgains, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
AttSkewSymmetricPModuleInterface::onSetPgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "att_p_module.gain.roll.p", gain_roll_p_)) {
      controller_->setRollGainP(gain_roll_p_);
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "att_p_module.gain.pitch.p", gain_pitch_p_)) {
      controller_->setPitchGainP(gain_pitch_p_);
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "att_p_module.gain.yaw.p", gain_yaw_p_)) {
      controller_->setYawGainP(gain_yaw_p_);
      continue;
    }
  }
  return result;
}

}  // namespace bluerov_ctrl
