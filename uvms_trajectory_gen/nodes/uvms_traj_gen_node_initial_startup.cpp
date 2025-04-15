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

#include "uvms_traj_gen_node_initial_startup.hpp"

namespace uvms_traj_gen {
void UVMSTrajGenStartUp::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  first_auv_state_ = false;
  first_manipulator_state_ = false;
  finished_ = false;
  bool print_load_motion_output = true;
  if (!already_initialized_) 
    initializeParameters(print_load_motion_output);
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  state_auv_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", qos,
      std::bind(&UVMSTrajGenStartUp::updateAUVStates, this,
                std::placeholders::_1));
  state_manipulator_sub_ =
      node_ptr_->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", qos,
          std::bind(&UVMSTrajGenStartUp::updateManipulatorStates, this,
                    std::placeholders::_1));
  setpoint_pub_ =
      node_ptr_->create_publisher<uvms_msgs::msg::UVMSControlTarget>(
          "traj_setpoint_uvms", qos);
}

void UVMSTrajGenStartUp::resetConnections() {
  state_auv_sub_.reset();
  state_manipulator_sub_.reset();
  setpoint_pub_.reset();
}

void UVMSTrajGenStartUp::resetAUVState() {
  first_auv_state_ = false;
}

void UVMSTrajGenStartUp::initializeParameters(bool output) {
  ros_param_utils::getParam(node_ptr_, v_max_init_, "startup.v_max_init", 0.1,
                            output);
  ros_param_utils::getParam(node_ptr_, w_max_init_, "startup.w_max_init", 0.1,
                            output);
  ros_param_utils::getParam(node_ptr_, dq_max_init_, "startup.dq_max_init", 0.1,
                            output);
  ros_param_utils::getParam(node_ptr_, start_accuracy_,
                            "startup.start_accuracy", 0.005, output);
  std::vector<double> start_pos;
  if (!ros_param_utils::getParamArray(node_ptr_, start_pos, "startup.start_pos",
                                      {1.0, 1.5, -0.5})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_pos not set!");
    return;
  }
  if (start_pos.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_pos parameter has size " +
                  std::to_string(start_pos.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  pos_auv_des_ = Eigen::Vector3d(start_pos.data());

  std::vector<double> start_att;
  if (!ros_param_utils::getParamArray(node_ptr_, start_att, "startup.start_att",
                                      {0.0, 0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_att not set!");
    return;
  }
  if (start_att.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_att parameter has size " +
                  std::to_string(start_att.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  att_auv_des_ =
      Eigen::AngleAxisd(start_att[2] * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(start_att[1] * M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(start_att[0] * M_PI, Eigen::Vector3d::UnitX());

  std::vector<double> start_q;
  if (!ros_param_utils::getParamArray(
          node_ptr_, start_q, "startup.start_joints", {1.0, 0.5, 0.5, 1.0} )) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "Param startup.start_joints not set!");

    return;
  }
  if (start_q.size() != param_utils::n_active_joints) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("startup.start_joints parameter has size " +
                  std::to_string(start_q.size()) + " but must be equal to " +
                  std::to_string(int(param_utils::n_active_joints)))
                     .c_str());
    return;
  }
  for (int i = 0; i < int(param_utils::n_active_joints); i++) {
    q_des_(i) = M_PI * start_q[i];
  }

  already_initialized_ = true;
  RCLCPP_INFO(node_ptr_->get_logger(),
                 "Initialized startup!");
}

void UVMSTrajGenStartUp::sendSetpoint() {
  if (!first_auv_state_ || !first_manipulator_state_) {
    return;
  }
  Eigen::Matrix3d att_auv_des_tilde;
  skew(att_auv_des_.vec(), att_auv_des_tilde);

  Eigen::Vector3d att_error = att_auv_.w() * att_auv_des_.vec() -
                              att_auv_des_.w() * att_auv_.vec() -
                              att_auv_des_tilde * att_auv_.vec();

  if (*status_ptr_ == TrajStatus::approaching_initial_auv_pose &&
      (pos_auv_des_ - pos_auv_).norm() < start_accuracy_ &&
      att_error.norm() < start_accuracy_) {
    *status_ptr_ = TrajStatus::reached_initial_auv_pose;
    std_msgs::msg::Int64 msg;
    msg.data = *status_ptr_;
    status_pub_ptr_->publish(msg);
  }
  if ((q_des_ - q_).norm() < start_accuracy_) {
    *status_ptr_ = TrajStatus::reached_initial_pose;
    std_msgs::msg::Int64 msg;
    msg.data = *status_ptr_;
    status_pub_ptr_->publish(msg);
    finished_ = true;
  }
  double t = (node_ptr_->now() - start_time_).seconds();
  if (*status_ptr_ < TrajStatus::reached_initial_auv_pose) {
    initial_auv_traj_.getPositionSetpoint(t, auv_setpoint_.pos,
                                          auv_setpoint_.vel, auv_setpoint_.acc);
    initial_auv_traj_.getOrientationSetpoint(
        t, auv_setpoint_.att, auv_setpoint_.ang_vel, auv_setpoint_.ang_acc);
  } else {
    initial_auv_traj_.getFinalPositionSetpoint(
        auv_setpoint_.pos, auv_setpoint_.vel, auv_setpoint_.acc);
    initial_auv_traj_.getFinalOrientationSetpoint(
        auv_setpoint_.att, auv_setpoint_.ang_vel, auv_setpoint_.ang_acc);
  }

  if (*status_ptr_ == TrajStatus::approaching_initial_manipulator_pose ||
      *status_ptr_ == TrajStatus::reached_initial_pose ||
      *status_ptr_ == TrajStatus::waiting_for_goal) {
    initial_manipulator_traj_.getSetpoint(t, manipulator_setpoint_.q,
                                          manipulator_setpoint_.dq,
                                          manipulator_setpoint_.ddq);
  } else { //
    manipulator_setpoint_.q = q_;
    manipulator_setpoint_.dq.setZero();
    manipulator_setpoint_.ddq.setZero();
  }

  uvms_msgs::msg::UVMSControlTarget out_msg;
  out_msg.header.stamp = node_ptr_->now();
  out_msg.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  hippo_common::convert::EigenToRos(auv_setpoint_.pos, out_msg.auv.position);
  hippo_common::convert::EigenToRos(auv_setpoint_.vel, out_msg.auv.velocity);
  hippo_common::convert::EigenToRos(auv_setpoint_.acc,
                                    out_msg.auv.acceleration);
  hippo_common::convert::EigenToRos(auv_setpoint_.att, out_msg.auv.attitude);
  hippo_common::convert::EigenToRos(auv_setpoint_.ang_vel,
                                    out_msg.auv.angular_velocity);
  hippo_common::convert::EigenToRos(auv_setpoint_.ang_acc,
                                    out_msg.auv.angular_acceleration);
  out_msg.auv.mask = 0;

  for (int i = 0; i < int(param_utils::n_active_joints); i++) {
    out_msg.manipulator.position[i] = manipulator_setpoint_.q(i);
    out_msg.manipulator.velocity[i] = manipulator_setpoint_.dq(i);
    out_msg.manipulator.acceleration[i] = manipulator_setpoint_.ddq(i);
  }

  setpoint_pub_->publish(out_msg);
}

void UVMSTrajGenStartUp::updateManipulatorStates(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < param_utils::n_active_joints; i++) {
    auto entry = std::find(msg->name.begin(), msg->name.end(),
                           static_cast<std::string>(joint_names[i]));
    if (entry != std::end(msg->name)) {
      size_t idx = std::distance(msg->name.begin(), entry);
      q_(int(i)) = msg->position[idx];
    } else {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "Requested joint not found in published states");
      return;
    }
  }
  if (!first_manipulator_state_) {
    first_manipulator_state_ = true;
    start_time_ = node_ptr_->now();
  }

  if (*status_ptr_ == TrajStatus::reached_initial_auv_pose) {
    start_time_ = node_ptr_->now();
    *status_ptr_ = TrajStatus::approaching_initial_manipulator_pose;
    initial_manipulator_traj_.initializeFromVelocityLimit(q_, q_des_,
                                                          dq_max_init_);
    std_msgs::msg::Int64 msg;
    msg.data = *status_ptr_;
    status_pub_ptr_->publish(msg);
  }
}

void UVMSTrajGenStartUp::updateAUVStates(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  hippo_common::convert::RosToEigen(msg->pose.pose.position, pos_auv_);
  hippo_common::convert::RosToEigen(msg->pose.pose.orientation, att_auv_);
  if (!first_auv_state_) {
    first_auv_state_ = true;
    start_time_ = node_ptr_->now();
    *status_ptr_ = TrajStatus::approaching_initial_auv_pose;
    initial_auv_traj_.initializeFromVelocityLimits(pos_auv_, att_auv_,
                                                   pos_auv_des_, att_auv_des_,
                                                   v_max_init_, w_max_init_);
    std_msgs::msg::Int64 msg;
    msg.data = *status_ptr_;
    status_pub_ptr_->publish(msg);
  }
  // std::cout << "Called after reset " << std::endl;
}

}  // namespace uvms_traj_gen