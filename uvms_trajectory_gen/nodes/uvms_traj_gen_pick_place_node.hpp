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

#pragma once
#include <cstdio>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <eigen3/Eigen/Dense>

#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "uvms_common/pose_to_pose_trajectory.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_traj_gen_node_initial_startup.hpp"
#include "uvms_trajectory_gen/traj.hpp"
#include "hippo_msgs/msg/pose_stamped_numbered.hpp"

using std::placeholders::_1;
using namespace std::chrono;
namespace uvms_traj_gen {

//! This class generates endeffector trajectories in the inertial coordinate
//! system, meaning that position, velocity and acceleration as well as angular
//! velocity and acceleration vectors are represented in the inertial coordinate
//! system. Attitude describes rotation between inertial coordinate system and
//! body coordinate system
class UVMSTrajGenPickPlace : public rclcpp::Node {
 public:
  UVMSTrajGenPickPlace();

 private:
  void initializeParameters(bool output);

  //!< returns setpoint in world coordinate system, meaning that position,
  //!< velocity, angular velocity are specified in inertial coordinate system,
  // attitude represents world-body
  void sendSetpoint();
  void updateEef(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void updateGoal(const hippo_msgs::msg::PoseStampedNumbered::SharedPtr msg);
  void updatePlannerMode(const std_msgs::msg::Int64::SharedPtr msg);
  void updatePlannerStatus(const std_msgs::msg::Int64::SharedPtr msg);

  rclcpp::Publisher<hippo_msgs::msg::ControlTarget>::SharedPtr eef_traj_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_eef_sub_;
  rclcpp::Subscription<hippo_msgs::msg::PoseStampedNumbered>::SharedPtr 
      goal_pose_eef_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr 
      planner_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr 
      planner_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;

  Eigen::Vector3d pos_;
  Eigen::Quaterniond att_;
  Eigen::Vector3d goal_pos_;
  Eigen::Quaterniond goal_att_;
  Eigen::Vector3d zero_vec_;

  Eigen::Vector3d start_pos_;
  Eigen::Quaterniond start_att_;
  uvms_common::p2p_trajectory::Pose2PoseTrajectory traj_gen_;
  double v_max_init_;
  double w_max_init_;
  double start_accuracy_;
  double freq_ = 50;  // Hz
  double run_accuracy_;
  double v_max_short_init_;
  double a_max_init_;
  double dw_max_init_;


  int n_runs_;
  int run_counter_ = 0;

  int old_goal_number_;

  hippo_msgs::msg::ControlTarget out_msg_;

  bool first_state_ = false;
  bool received_goal_ = true;
  bool first_goal_ = false;

  int traj_status_ = TrajStatus::undeclared;
  int planner_mode_ = TrajMode::undeclared_mode;
  int planner_status_ = -1; //cannot include uvms_kinematic_ctrl with the plannerMode due to cycle dependencies

  UVMSTrajGenStartUp initial_startup_;
};

}  // namespace uvms_traj_gen
