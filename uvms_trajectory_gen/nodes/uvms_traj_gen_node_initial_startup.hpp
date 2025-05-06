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

#ifndef UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_INITIAL_STARTUP_HPP
#define UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_INITIAL_STARTUP_HPP
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "uvms_common/param_utils.hpp"
#include "uvms_common/pose_to_pose_trajectory.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_msgs/msg/uvms_control_target.hpp"
#include "uvms_trajectory_gen/traj.hpp"

using std::placeholders::_1;
using namespace std::chrono;
using param_utils::joint_names;
using param_utils::link_names;
using param_utils::StateVector;

namespace uvms_traj_gen {

struct AUVTrajSetpoint {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Quaterniond att;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d ang_acc;
};

struct ManipulatorTrajSetpoint {
  StateVector q;
  StateVector dq;
  StateVector ddq;
};

class UVMSTrajGenStartUp {
 public:
  UVMSTrajGenStartUp() {};
  bool startupFinished() { return finished_; }

  void setTrajStatusPtr(int* status_ptr) { status_ptr_ = status_ptr; }

  void setTrajStatusPublisherPtr(
      const rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr& status_pub) {
    status_pub_ptr_ = status_pub;
  }
  void resetConnections();
  void sendSetpoint();
  void initialize(rclcpp::Node* node_ptr);

  void resetAUVState();

 private:
  void initializeParameters(bool output);
  void updateManipulatorStates(
      const sensor_msgs::msg::JointState::SharedPtr msg);
  void updateAUVStates(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node* node_ptr_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_auv_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      state_manipulator_sub_;
  rclcpp::Publisher<uvms_msgs::msg::UVMSControlTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_ptr_;

  rclcpp::Time start_time_;
  Eigen::Vector3d pos_auv_;
  Eigen::Vector3d pos_auv_des_;
  Eigen::Quaterniond att_auv_;
  Eigen::Quaterniond att_auv_des_;
  StateVector q_des_;
  StateVector q_;

  AUVTrajSetpoint auv_setpoint_;
  ManipulatorTrajSetpoint manipulator_setpoint_;

  uvms_common::p2p_trajectory::Pose2PoseTrajectory initial_auv_traj_;
  uvms_common::p2p_trajectory::State2StateTrajectory<
      param_utils::n_active_joints>
      initial_manipulator_traj_;

  // maximum initial velocities
  double v_max_init_;
  double w_max_init_;
  double dq_max_init_;
  double start_accuracy_;

  bool first_auv_state_ = false;
  bool first_manipulator_state_ = false;
  bool finished_ = false;  //!< decides if startup sequence is finished
  int* status_ptr_;

  bool already_initialized_ = false;
};

}  // namespace uvms_traj_gen
#endif  // UVMS_TRAJECTORY_GEN_UVMS_TRAJ_GEN_NODE_INITIAL_STARTUP_HPP
