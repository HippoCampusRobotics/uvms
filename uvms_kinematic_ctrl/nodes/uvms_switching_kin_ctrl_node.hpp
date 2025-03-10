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

#ifndef UVMS_KINEMATIC_CTRL_UVMS_SWITCHING_KIN_CTRL_NODE_H
#define UVMS_KINEMATIC_CTRL_UVMS_SWITCHING_KIN_CTRL_NODE_H
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "alpha_msgs/msg/joint_data.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "hippo_msgs/msg/velocity_control_target.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_switching_kin_ctrl_node_configuration_space.hpp"
#include "uvms_kinematic_ctrl/kin_ctrl_node_interface.hpp"
#include "uvms_msgs/msg/control_target_prediction.hpp"

namespace uvms_kin_ctrl {
using std::placeholders::_1;

class UVMSSwitchingKinematicControlNode : public rclcpp::Node {
 public:
  UVMSSwitchingKinematicControlNode();

 private:
  void initTimers();
  void initController();
  void declareParams();
  void initPublishers();
  void initSubscriptions();
  alpha_msgs::msg::JointData zeroManipulatorMsg(const rclcpp::Time& _stamp);
  void onSetpointTimeout();
  void onSetpointTarget(const hippo_msgs::msg::ControlTarget::SharedPtr _msg);
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr _msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void publishControlCmds();

  std::mutex mutex_;

  //////////////////////////////////////////////////////////////////////////////
  // ros params
  //////////////////////////////////////////////////////////////////////////////

  UVMSKinematicControlInterface* controller_interface_;
  nav_msgs::msg::Odometry last_odometry_;
  sensor_msgs::msg::JointState last_joint_state_;
  bool setpoint_timed_out_{false};
  bool state_timed_out_{false};

  bool got_first_setpoint_{false};
  bool got_first_auv_state_{false};
  bool got_first_manipulator_state_{false};
  bool publish_on_joint_state_;

  rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;

  bool got_first_time_stamp_{false};
  rclcpp::Time last_stamp_;

  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<alpha_msgs::msg::JointData>::SharedPtr manipulator_cmd_pub_;
  rclcpp::Publisher<hippo_msgs::msg::VelocityControlTarget>::SharedPtr
      auv_vel_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pose_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      state_sub_manipulator_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_auv_;
  rclcpp::Subscription<hippo_msgs::msg::ControlTarget>::SharedPtr eef_traj_sub_;

  UVMSSwitchingKinematicConfigurationControl* startup_controller_;
  int controller_status_ = ControllerStatus::undeclared;
};

}  // namespace uvms_kin_ctrl

#endif  // UVMS_KINEMATIC_CTRL_UVMS_SWITCHING_KIN_CTRL_NODE_H
