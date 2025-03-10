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
#include <mutex> //part of standard C++ library and directly provided through compiler
#include <chrono> //part of standard C++ library and directly provided through compiler
#include <std_msgs/msg/int64.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath> //part of standard C++ library and directly provided through compiler. For std::sqrt(), std::pow(), std::copysign()
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "alpha_ctrl/gripper_ctrl_interface.hpp"
#include "uvms_trajectory_gen/traj.hpp"
#include "hippo_msgs/msg/pose_stamped_numbered.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "hippo_common/convert.hpp"

namespace uvms_kin_ctrl {
using std::placeholders::_1;

enum PlannerMode {
  undefined = -1,
  waiting_for_all_processes = 1,
  looking_for_object = 2,
  approaching_offset_distance = 3,
  prepare_gripping = 4,
  approaching_object = 5,
  gripping_object = 6,
  lift_from_surface = 7,
  approach_placement_offset_pose = 8,
  approaching_placement_pose = 9,
  releasing_object = 10,
  retreat_from_object = 11,
  finishing_pick_and_place = 12,
  done = 13,
  transition_after_reaching_goal = 14,
  read_new_cylinder = 15,
};

class UVMSPlannerNode : public rclcpp::Node {
 public:
  UVMSPlannerNode();
 private:
    // Functions
    void initPublishers();
    void declareParams();
    void initTimers();
    void initSubscriptions();
    void initServices();

    void serveTestRun(const std_srvs::srv::SetBool_Request::SharedPtr _request,
                   std_srvs::srv::SetBool_Response::SharedPtr _response);

    void onStatusTrajectoryTimeout();
    void onStatusGripperTimeout();

    void onPoseObject(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
    void onPoseEndeffector(const geometry_msgs::msg::PoseStamped::SharedPtr _msg); 
    void onStatusTrajectory(const std_msgs::msg::Int64::SharedPtr _msg);
    void onStatusGripper(const std_msgs::msg::Int64::SharedPtr _msg);
    void onOdometryCylinder(const nav_msgs::msg::Odometry::SharedPtr _msg);
    void onOdometryPlatform(const nav_msgs::msg::Odometry::SharedPtr _msg);
    void onOdometryCylinderHolder(const nav_msgs::msg::Odometry::SharedPtr _msg);


    void runPlanner();

    void vectorPlaneProjection(const Eigen::Vector3d &_plane_normal, const Eigen::Vector3d &_vector, Eigen::Vector3d &_proj);
    void getRotationFromVector(const Eigen::Vector3d &_x_eff, const Eigen::Vector3d &_z_eff, Eigen::Matrix3d &_rotation);
    void vectorVectorProjection(const Eigen::Vector3d &_base_vector, const Eigen::Vector3d &_vector, Eigen::Vector3d &_proj);
    Eigen::Quaterniond alignZAxes(const Eigen::Quaterniond& qA, const Eigen::Quaterniond& qB);

    // Other-------------------------------------------
    std::mutex mutex_;

    hippo_msgs::msg::PoseStampedNumbered pose_out_msg_;

    std_msgs::msg::Int64 msg_gripper_;
    std_msgs::msg::Int64 msg_traj_;
    std_msgs::msg::Int64 msg_planner_;

    // State Variables --------------------------------
    int gripper_status_{alpha_ctrl::GripperStatus::undefined};
    int gripper_mode_{alpha_ctrl::GripperMode::undefined_mode};

    int traj_status_{uvms_traj_gen::TrajStatus::undeclared};
    int traj_mode_{uvms_traj_gen::TrajMode::undeclared_mode};

    int planner_mode_{PlannerMode::undefined};

    int next_planner_mode_{PlannerMode::undefined};
    int next_traj_mode_{uvms_traj_gen::TrajMode::undeclared_mode};

    // Boolean Varibales ------------------------------
    bool trajectory_status_timed_out_{false};
    bool gripper_status_timed_out_{false};
    bool got_first_object_pose_{false};
    bool got_first_platform_pose_{false};
    bool got_first_cylinder_holder_pose_{false};
    bool got_first_eef_pose_{false};
    bool got_first_traj_status_{false};
    bool got_first_gripper_status_{false};
    bool test_run_activated_{false};
    bool automated_{false};
    bool automated_check_{false};


    // Constant Variables / ros params ----------------
    double offset_dist_;

    double cylinder_height_;

    double discharge_height_;

    double lift_surface_dist_;

    double eps_orthogonal_{0.001};

    int number_rounds_;
    int counter_rounds_;

    int counter_msg_pose_des_{0};

    std::stringstream stream_counter_;
    std::stringstream stream_rounds_;

    Eigen::Vector3d place_pos_;
    Eigen::Quaterniond place_att_;
    Eigen::Vector3d place_plane_normal_;//{0.0, 0.0, 1.0}; // in world frame

    Eigen::Vector3d cylinder_holder_pos_;
    Eigen::Quaterniond cylinder_holder_att_;
    Eigen::Vector3d cylinder_holder_plane_normal_; // in world frame

    Eigen::Vector3d dropping_pos_;
    Eigen::Quaterniond dropping_att_;
    Eigen::Vector3d dropping_plane_normal_;

    Eigen::Matrix3d rotation_x_60_{{1.0, 0.0, 0.0},{0.0, 0.5, -std::sqrt(3)/2.0},{0.0, std::sqrt(3)/2.0, 0.5}};

    // Eigen::Vector3d lift_surface_dist_{0.0, 0.0, 0.1}; //unit = m

    Eigen::Vector3d direction_2_place_;

    Eigen::Vector3d eef_offset_obj_new_I_;

    // Message Storage --------------------------------
    Eigen::Vector3d object_pos_;
    Eigen::Quaterniond object_att_;
    Eigen::Vector3d object_plane_normal_; // in world frame

    Eigen::Vector3d eef_pos_;
    Eigen::Quaterniond eef_att_;

    Eigen::Vector3d eef_pos_des_;
    Eigen::Quaterniond eef_att_des_;

    // Timer ------------------------------------------
    rclcpp::TimerBase::SharedPtr trajectory_status_timeout_timer_;
    rclcpp::TimerBase::SharedPtr gripper_status_timeout_timer_;
    rclcpp::TimerBase::SharedPtr run_planner_timer_;

    // Publisher --------------------------------------
    rclcpp::Publisher<hippo_msgs::msg::PoseStampedNumbered>::SharedPtr eef_pose_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr gripper_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr planner_mode_pub_;
  
    // Subscriber -------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr traj_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr gripper_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr platform_odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cylinder_odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cylinder_holder_odometry_sub_;


    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_run_service_;
};

}  // namespace uvms_kin_ctrl