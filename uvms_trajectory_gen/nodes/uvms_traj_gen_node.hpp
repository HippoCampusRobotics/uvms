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

#ifndef UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_HPP
#define UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_HPP
#include <cstdio>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "uvms_common/pose_to_pose_trajectory.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_msgs/msg/control_target_prediction.hpp"
#include "uvms_traj_gen_node_initial_startup.hpp"
#include "uvms_trajectory_gen/traj.hpp"

using std::placeholders::_1;
using namespace std::chrono;
using param_utils::joint_names;
using param_utils::link_names;
using param_utils::StateVector;
namespace uvms_traj_gen {

//! This class generates endeffector trajectories in the inertial coordinate
//! system, meaning that position, velocity and acceleration as well as angular
//! velocity and acceleration vectors are represented in the inertial coordinate
//! system. Attitude describes rotation between inertial coordinate system and
//! body coordinate system
class UVMSTrajGen : public rclcpp::Node {
 public:
  UVMSTrajGen();

 private:
  void initializeParameters(bool output);

  //!< returns setpoint in world coordinate system, meaning that position,
  //!< velocity, angular velocity are specified in inertial coordinate system,
  // attitude represents world-body
  void sendSetpoint();
  void updateEef(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Publisher<hippo_msgs::msg::ControlTarget>::SharedPtr eef_traj_pub_;
  rclcpp::Publisher<uvms_msgs::msg::ControlTargetPrediction>::SharedPtr
      eef_traj_pub_prediction_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
      angular_vel_debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_eef_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr startup_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Traj *traj_gen_;
  rclcpp::Time start_time_;

  Eigen::Vector3d pos_;
  Eigen::Quaterniond att_;
  // parameters relevant for trajectory to initial point
  uvms_common::p2p_trajectory::Pose2PoseTrajectory start_traj_;
  double v_max_init_;
  double w_max_init_;
  double start_accuracy_;
  double freq_ = 50;  // Hz

  int n_runs_;
  int run_counter_ = 0;

  hippo_msgs::msg::ControlTarget out_msg_;
  uvms_msgs::msg::ControlTargetPrediction out_msg_prediction_;
  bool publish_prediction_ = false;

  bool first_state_ = false;

  int traj_status_ = TrajStatus::undeclared;

  UVMSTrajGenStartUp initial_startup_;
};

}  // namespace uvms_traj_gen
#endif  // UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_HPP
