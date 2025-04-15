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

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64.hpp>
#include "hippo_msgs/msg/int64_stamped.hpp"
#include "hippo_msgs/msg/float64_stamped.hpp"
#include "hippo_msgs/msg/vector_error.hpp"
#include "hippo_msgs/msg/control_target.hpp"

#include "hippo_common/convert.hpp"



namespace uvms_kin_ctrl {
using std::placeholders::_1;

class PaPEvaluationNode : public rclcpp::Node {
 public:
  PaPEvaluationNode();
 private:
    // Functions
    void initPublishers();
    void declareParams();
    void initTimers();
    void initSubscriptions();

    void onPoseEndeffector(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
    void onSetpointEndeffector(const hippo_msgs::msg::ControlTarget::SharedPtr _msg);
    void onPosePlatform(const nav_msgs::msg::Odometry::SharedPtr _msg);
    void onPoseCylinder(const nav_msgs::msg::Odometry::SharedPtr _msg);
    void onTrajStatus(const std_msgs::msg::Int64::SharedPtr _msg);
    void onPlannerMode(const std_msgs::msg::Int64::SharedPtr _msg);


    void getFrameZAxis(Eigen::Vector3d& zaxis, const Eigen::Quaterniond& frame_att);
    void getFrameYAxis(Eigen::Vector3d& yaxis, const Eigen::Quaterniond& frame_att);
    Eigen::Matrix4d getHomogeneousTransformation();
    Eigen::Vector3d intersectionPlatform(const Eigen::Vector3d& point_on_line, 
                                             const Eigen::Vector3d& direction_line,
                                             const Eigen::Vector3d& point_on_plane, 
                                             const Eigen::Vector3d& normal_plane);
    Eigen::Vector3d transformPointToNewFrame(const Eigen::Vector3d& point_in_inertial,
                                                            const Eigen::Matrix4d& transformation_matrix);
    void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde);
    void publishEvaluation();



    // Other-------------------------------------------
    std::mutex mutex_;
    std::mutex traj_queue_mutex;
    std::mutex SM_queue_mutex;


    std::queue<int> traj_status_queue;
    std::queue<int> SM_queue;

    hippo_msgs::msg::Int64Stamped last_SM_change_;

    geometry_msgs::msg::PointStamped msg_eef_pos_;
    geometry_msgs::msg::PointStamped msg_eef_pos_setpoint_;
    geometry_msgs::msg::PointStamped msg_eef_pos_vec_err_;
    geometry_msgs::msg::PointStamped msg_eef_att_vec_err_;
    hippo_msgs::msg::Int64Stamped msg_SM_change_;
    hippo_msgs::msg::Float64Stamped msg_eef_pos_err_;
    hippo_msgs::msg::Float64Stamped msg_eef_att_err_;
    hippo_msgs::msg::VectorError msg_placed_pos_;
    hippo_msgs::msg::VectorError msg_gripped_pos_;
    hippo_msgs::msg::VectorError msg_gripped_angle_;
    hippo_msgs::msg::Int64Stamped msg_traj_status_;

    //R_eef^(eef,perpendicular), where eef is the frame in pose_endeffector topic
    Eigen::Matrix3d rotation_x_60_{{1.0, 0.0, 0.0},{0.0, 0.5, -std::sqrt(3)/2.0},{0.0, std::sqrt(3)/2.0, 0.5}};


    // Boolean Varibales ------------------------------
    bool SM_stage_changed_{false};
    bool received_traj_status_{false};

    // Constant Variables / ros params ----------------
    Eigen::Vector3d cylinder_pos_;
    Eigen::Quaterniond cylinder_att_;
    Eigen::Vector3d cylinder_z_axis_;

    Eigen::Vector3d platform_pos_;
    Eigen::Quaterniond platform_att_;
    Eigen::Vector3d platform_z_axis_;

    Eigen::Vector3d eef_pos_;
    Eigen::Quaterniond eef_att_;
    Eigen::Vector3d eef_z_axis_;
    Eigen::Vector3d eef_y_axis_;

    Eigen::Vector3d eef_setpoint_pos_;
    Eigen::Quaterniond eef_setpoint_att_;


    // Timer ------------------------------------------
    rclcpp::TimerBase::SharedPtr publish_evaluation_timer_;


    // Publisher --------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eef_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eef_pos_setpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eef_pos_vec_error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eef_att_vec_error_pub_;

    rclcpp::Publisher<hippo_msgs::msg::Int64Stamped>::SharedPtr StateMachine_change_pub_;

    rclcpp::Publisher<hippo_msgs::msg::Float64Stamped>::SharedPtr eef_pos_error_pub_;
    rclcpp::Publisher<hippo_msgs::msg::Float64Stamped>::SharedPtr eef_att_error_pub_;

    rclcpp::Publisher<hippo_msgs::msg::VectorError>::SharedPtr placed_pos_pub_;
    rclcpp::Publisher<hippo_msgs::msg::VectorError>::SharedPtr gripped_pos_pub_;
    rclcpp::Publisher<hippo_msgs::msg::VectorError>::SharedPtr gripped_angle_pub_;

    rclcpp::Publisher<hippo_msgs::msg::Int64Stamped>::SharedPtr traj_status_stamped_pub_;

  
    // Subscriber -------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pose_sub_;
    rclcpp::Subscription<hippo_msgs::msg::ControlTarget>::SharedPtr eef_traj_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr platform_odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cylinder_odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr traj_staus_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr planner_mode_sub_;


};

}  // namespace uvms_kin_ctrl