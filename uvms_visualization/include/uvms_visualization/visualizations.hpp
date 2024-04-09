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

#ifndef UVMS_KINEMATIC_CTRL_VISUALIZATIONS_HPP
#define UVMS_KINEMATIC_CTRL_VISUALIZATIONS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hippo_common/tf2_utils.hpp"
#include "hippo_control_msgs/msg/actuator_setpoint.hpp"
#include "hippo_control_msgs/msg/control_target.hpp"
#include "hippo_control_msgs/msg/velocity_control_target.hpp"
#include "uvms_msgs/msg/uvms_control_target.hpp"

namespace uvms_visualization {
constexpr int kNMarker = 500;  //!< number of markers used for line strips

enum visualization_modules {
  endeffector = 1,
  target = 2,
  velocity_target = 3,
  auv_velocity_target = 4,
  auv_thrust = 5,
  auv_pose = 6,
  uvms_target = 7,
  endeffector_frame = 8,
  target_frame = 9,
  endeffector_axis = 10,
};

class Visualization {
 public:
  Visualization() = default;
  virtual void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) = 0;
  virtual void initializeMarkers() = 0;

 protected:
  rclcpp::Node* node_ptr_;
  visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr_;
  std::map<std::string, int> idx_map_;
};

class EndeffectorVisualization : public Visualization {
 public:
  EndeffectorVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onEndeffectorPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_sub_;
};

class EndeffectorFrameVisualization : public Visualization {
 public:
  EndeffectorFrameVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onEndeffectorPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_sub_;
};

class EndeffectorAxisVisualization : public Visualization {
 public:
  EndeffectorAxisVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onEndeffectorPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_sub_;
};

class TargetVisualization : public Visualization {
 public:
  TargetVisualization(){};
  virtual void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 protected:
  void onTarget(const hippo_control_msgs::msg::ControlTarget::SharedPtr msg);
  rclcpp::Subscription<hippo_control_msgs::msg::ControlTarget>::SharedPtr
      target_sub_;
};

class TargetFrameVisualization : public Visualization {
 public:
  TargetFrameVisualization(){};
  virtual void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 protected:
  void onTarget(const hippo_control_msgs::msg::ControlTarget::SharedPtr msg);
  rclcpp::Subscription<hippo_control_msgs::msg::ControlTarget>::SharedPtr
      target_sub_;
};

class UVMSTargetVisualization : public TargetVisualization {
 public:
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;

 private:
  void onUVMSTarget(const uvms_msgs::msg::UVMSControlTarget::SharedPtr msg);
  rclcpp::Subscription<uvms_msgs::msg::UVMSControlTarget>::SharedPtr
      uvms_target_sub_;
};

class VelocityTargetVisualization : public Visualization {
 public:
  VelocityTargetVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onTarget(const hippo_control_msgs::msg::ControlTarget::SharedPtr msg);
  rclcpp::Subscription<hippo_control_msgs::msg::ControlTarget>::SharedPtr
      target_sub_;
};

class AUVPoseVisualization : public Visualization {
 public:
  AUVPoseVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

class AUVVelocityTargetVisualization : public Visualization {
 public:
  AUVVelocityTargetVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onVelocityTarget(
      const hippo_control_msgs::msg::VelocityControlTarget::SharedPtr msg);
  rclcpp::Subscription<hippo_control_msgs::msg::VelocityControlTarget>::
      SharedPtr velocity_target_sub_;
};

class AUVThrustVisualization : public Visualization {
 public:
  AUVThrustVisualization(){};
  void initialize(
      rclcpp::Node* node_ptr,
      visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) override;
  void initializeMarkers() override;

 private:
  void onThrust(const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr msg);
  rclcpp::Subscription<hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_sub_;
};

}  // namespace uvms_visualization
#endif  // UVMS_KINEMATIC_CTRL_VISUALIZATIONS_HPP
