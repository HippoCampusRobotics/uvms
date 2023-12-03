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

#ifndef BLUEROV_CTRL_VELOCITY_CONTROL_NODE_INTERFACES_HPP
#define BLUEROV_CTRL_VELOCITY_CONTROL_NODE_INTERFACES_HPP

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "hippo_common/convert.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_msgs/msg/velocity_control_target.hpp"
#include "manipulator_compensation_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "twist_model_based_interface.hpp"
#include "twist_pid_interface.hpp"

namespace bluerov_ctrl {
enum ControllerType {
  vel_pid_comp = 1,
  vel_model_based_comp = 4,
};

//////////////////////////////////////////////////////////////////////////////
// controller interfaces
//////////////////////////////////////////////////////////////////////////////
class ControllerNodeInterface {
 public:
  ControllerNodeInterface() = default;
  virtual void initialize(rclcpp::Node *node_ptr) = 0;

  bool isOk() {
    checkStatus();
    return is_ok_;
  }

  //! set setpoint for the controller
  //! \param _msg
  virtual void setSetpointTarget(
      const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) = 0;
  virtual void getControllerOutput(
      const double &dt, const nav_msgs::msg::Odometry::SharedPtr msg,
      hippo_msgs::msg::ActuatorSetpoint &out_thrust,
      hippo_msgs::msg::ActuatorSetpoint &out_torque) = 0;
  virtual void onTimeout() = 0;
  virtual void publishDebugMsgs() = 0;

 protected:
  virtual void declareParams() = 0;
  virtual void checkStatus() = 0;
  bool is_ok_ = true;
  rclcpp::Node *node_ptr_;
};

class PIDCompInterface : public ControllerNodeInterface {
 public:
  PIDCompInterface() = default;

  void initialize(rclcpp::Node *node_ptr) override;

  void setSetpointTarget(
      const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) override;

  void getControllerOutput(
      const double &dt, const nav_msgs::msg::Odometry::SharedPtr msg,
      hippo_msgs::msg::ActuatorSetpoint &out_thrust,
      hippo_msgs::msg::ActuatorSetpoint &out_torque) override;

  void onTimeout() override;

  void publishDebugMsgs() override;

 private:
  void declareParams() override;
  void checkStatus() override;

  TwistPIDInterface *velocity_control_interface_;
  ManipulatorCompInterface *manipulator_comp_interface_;
};

class ModelBasedCompInterface : public ControllerNodeInterface {
 public:
  ModelBasedCompInterface() = default;
  void initialize(rclcpp::Node *node_ptr) override;
  void setSetpointTarget(
      const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) override;
  void getControllerOutput(
      const double &dt, const nav_msgs::msg::Odometry::SharedPtr msg,
      hippo_msgs::msg::ActuatorSetpoint &out_thrust,
      hippo_msgs::msg::ActuatorSetpoint &out_torque) override;
  void onTimeout() override;
  void publishDebugMsgs() override;

 private:
  void declareParams() override;
  void checkStatus() override;

  TwistModelBasedInterface *velocity_control_interface_;
  ManipulatorCompInterface *manipulator_comp_interface_;
};

}  // namespace bluerov_ctrl

#endif  // BLUEROV_CTRL_VELOCITY_CONTROL_NODE_INTERFACES_HPP
