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

#ifndef UVMS_TRAJECTORY_GEN_UVMS_SWITCHING_KIN_CTRL_NODE_CONFIGURATION_SPACE_HPP
#define UVMS_TRAJECTORY_GEN_UVMS_SWITCHING_KIN_CTRL_NODE_CONFIGURATION_SPACE_HPP
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "alpha_ctrl/joint_kinematic_control_interface.hpp"
#include "alpha_msgs/msg/joint_data.hpp"
#include "bluerov_ctrl/attitude_skew_symmetric_p_module_interface.hpp"
#include "bluerov_ctrl/position_p_module_interface.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/velocity_control_target.hpp"
#include "uvms_msgs/msg/uvms_control_target.hpp"

namespace uvms_kin_ctrl {
using rcl_interfaces::msg::SetParametersResult;
using std::placeholders::_1;

enum ControllerStatus {
  undeclared = -1,
  joint_space_control = 1,
  eef_control = 2
};

class UVMSSwitchingKinematicConfigurationControl {
 public:
  UVMSSwitchingKinematicConfigurationControl();

  void setControllerStatusPtr(int *controller_status_ptr) {
    controller_status_ptr_ = controller_status_ptr;
  }

  void initialize(rclcpp::Node *node_ptr);

//   void resetConnections();

  void initializeParameterCallbacks();

  void setManipulatorCmdPublisherPtr(
      const rclcpp::Publisher<alpha_msgs::msg::JointData>::SharedPtr &pub) {
    manipulator_cmd_pub_ = pub;
  }
  void setAUVCmdPublisherPtr(
      const rclcpp::Publisher<hippo_msgs::msg::VelocityControlTarget>::SharedPtr
          &pub) {
    auv_vel_cmd_pub_ = pub;
  }

  void publishControlCommands(
      const nav_msgs::msg::Odometry &auv_msg,
      const sensor_msgs::msg::JointState &manipulator_msg);

 private:
  void initController();

  void initTimers();

  void initSubscriptions();

  alpha_msgs::msg::JointData zeroManipulatorMsg(const rclcpp::Time &_stamp);

  void onSetpointTimeout();

  void onSetpointTarget(
      const uvms_msgs::msg::UVMSControlTarget::SharedPtr _msg);

  std::mutex mutex_;

  //////////////////////////////////////////////////////////////////////////////
  // ros params
  //////////////////////////////////////////////////////////////////////////////

  rclcpp::Node *node_ptr_;
  nav_msgs::msg::Odometry last_odometry_;
  sensor_msgs::msg::JointState last_joint_state_;
//   bool setpoint_timed_out_{false};

  bool got_first_setpoint_{false};

  rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;

  /// Publishers
  rclcpp::Publisher<alpha_msgs::msg::JointData>::SharedPtr manipulator_cmd_pub_;
  rclcpp::Publisher<hippo_msgs::msg::VelocityControlTarget>::SharedPtr
      auv_vel_cmd_pub_;

  /// Subscriptions
  rclcpp::Subscription<uvms_msgs::msg::UVMSControlTarget>::SharedPtr
      setpoint_sub_;

  /// controller interfaces
  alpha_ctrl::JointKinematicControlInterface *manipulator_controller_interface_;
  bluerov_ctrl::PosPModuleInterface *auv_position_controller_interface_;
  bluerov_ctrl::AttSkewSymmetricPModuleInterface
      *auv_attitude_controller_interface_;

  int *controller_status_ptr_;
};
}  // namespace uvms_kin_ctrl

#endif  // UVMS_TRAJECTORY_GEN_UVMS_SWITCHING_KIN_CTRL_NODE_CONFIGURATION_SPACE_HPP
