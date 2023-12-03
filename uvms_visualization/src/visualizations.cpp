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

#include "uvms_visualization/visualizations.hpp"
namespace uvms_visualization {

void EndeffectorVisualization::initialize(
    rclcpp::Node* node_ptr,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  eef_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_endeffector", qos,
      std::bind(&EndeffectorVisualization::onEndeffectorPose, this,
                std::placeholders::_1));
}

void EndeffectorVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_est;
  marker_est.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_est.action = visualization_msgs::msg::Marker::ADD;
  marker_est.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est.id = idx;
  marker_est.color.a = 0.7;
  marker_est.color.r = 1.0;
  marker_est.color.g = 0.0;
  marker_est.color.b = 0.0;
  marker_est.scale.x = 0.01;
  marker_est.scale.y = 0.01;
  marker_est.scale.z = 0.01;
  marker_est.pose.orientation.w = 1.0;
  marker_ptr_->markers.emplace_back(marker_est);
  idx_map_["pos_est_eef"] = idx;
}
void EndeffectorVisualization::onEndeffectorPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  marker_ptr_->markers[idx_map_.at("pos_est_eef")].header.stamp =
      msg->header.stamp;
  if (int(marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.size()) <
      kNMarker) {
    marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.push_back(
        msg->pose.position);
  } else {
    marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.push_back(
        msg->pose.position);
    for (int i = 0; i < kNMarker; i++) {
      marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.at(i) =
          marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.at(i + 1);
    }
    marker_ptr_->markers[idx_map_.at("pos_est_eef")].points.pop_back();
  }
}

void EndeffectorFrameVisualization::initialize(
    rclcpp::Node* node_ptr,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  eef_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_endeffector", qos,
      std::bind(&EndeffectorFrameVisualization::onEndeffectorPose, this,
                std::placeholders::_1));
}

void EndeffectorFrameVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_est_pose_x;
  marker_est_pose_x.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_x.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_x.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_x.id = idx;
  marker_est_pose_x.color.a = 0.7;
  marker_est_pose_x.color.r = 1.0;
  marker_est_pose_x.color.g = 0.0;
  marker_est_pose_x.color.b = 0.0;
  marker_est_pose_x.scale.x = 0.01;
  marker_est_pose_x.scale.y = 0.05;
  marker_est_pose_x.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_x.points.push_back(p);
  p.x = 1.0;
  marker_est_pose_x.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_x);
  idx_map_["pose_est_eef_x"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_est_pose_y;
  marker_est_pose_y.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_y.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_y.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_y.id = idx;
  marker_est_pose_y.color.a = 0.7;
  marker_est_pose_y.color.r = 0.0;
  marker_est_pose_y.color.g = 1.0;
  marker_est_pose_y.color.b = 0.0;
  marker_est_pose_y.scale.x = 0.01;
  marker_est_pose_y.scale.y = 0.05;
  marker_est_pose_y.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_y.points.push_back(p);
  p.y = 1.0;
  marker_est_pose_y.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_y);
  idx_map_["pose_est_eef_y"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_est_pose_z;
  marker_est_pose_z.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_z.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_z.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_z.id = idx;
  marker_est_pose_z.color.a = 0.7;
  marker_est_pose_z.color.r = 0.0;
  marker_est_pose_z.color.g = 0.0;
  marker_est_pose_z.color.b = 1.0;
  marker_est_pose_z.scale.x = 0.01;
  marker_est_pose_z.scale.y = 0.05;
  marker_est_pose_z.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_z.points.push_back(p);
  p.z = 1.0;
  marker_est_pose_z.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_z);
  idx_map_["pose_est_eef_z"] = idx;
}
void EndeffectorFrameVisualization::onEndeffectorPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].pose.orientation =
        msg->pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].points.push_back(p);
    p.x = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_x")].pose.orientation =
        msg->pose.orientation;
  }
  marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.orientation =
        msg->pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.push_back(p);
    p.y = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.orientation =
        msg->pose.orientation;
  }
  marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].pose.orientation =
        msg->pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].points.push_back(p);
    p.z = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_z")].pose.orientation =
        msg->pose.orientation;
  }
}

void EndeffectorAxisVisualization::initialize(
    rclcpp::Node* node_ptr,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  eef_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_endeffector", qos,
      std::bind(&EndeffectorAxisVisualization::onEndeffectorPose, this,
                std::placeholders::_1));
}

void EndeffectorAxisVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_est_pose_y;
  marker_est_pose_y.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_y.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_y.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_y.id = idx;
  marker_est_pose_y.color.a = 0.7;
  marker_est_pose_y.color.r = 0.0;
  marker_est_pose_y.color.g = 1.0;
  marker_est_pose_y.color.b = 0.0;
  marker_est_pose_y.scale.x = 0.01;
  marker_est_pose_y.scale.y = 0.02;
  marker_est_pose_y.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_y.points.push_back(p);
  p.y = 0.5;
  marker_est_pose_y.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_y);
  idx_map_["pose_est_eef_y"] = idx;
}
void EndeffectorAxisVisualization::onEndeffectorPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.orientation =
        msg->pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.push_back(p);
    p.y = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.position =
        msg->pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_eef_y")].pose.orientation =
        msg->pose.orientation;
  }
}

void TargetVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  target_sub_ = node_ptr_->create_subscription<hippo_msgs::msg::ControlTarget>(
      "traj_setpoint", qos,
      std::bind(&TargetVisualization::onTarget, this, std::placeholders::_1));
}

void TargetVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;

  visualization_msgs::msg::Marker marker_des;
  marker_des.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_des.action = visualization_msgs::msg::Marker::ADD;
  marker_des.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  marker_des.id = idx;
  marker_des.color.a = 0.7;
  marker_des.color.r = 0.0;
  marker_des.color.g = 0.0;
  marker_des.color.b = 1.0;
  marker_des.scale.x = 0.01;
  marker_des.scale.y = 0.01;
  marker_des.scale.z = 0.01;
  marker_des.pose.orientation.w = 1.0;
  marker_ptr_->markers.emplace_back(marker_des);
  idx_map_["pos_des"] = idx;
}

void TargetVisualization::onTarget(
    const hippo_msgs::msg::ControlTarget::SharedPtr msg) {
  geometry_msgs::msg::Point p;
  if (msg->mask != msg->IGNORE_POSITION && msg->mask != msg->IGNORE_POSE) {
    marker_ptr_->markers[idx_map_.at("pos_des")].header.stamp =
        msg->header.stamp;
    p.x = msg->position.x;
    p.y = msg->position.y;
    p.z = msg->position.z;
    if (int(marker_ptr_->markers[idx_map_.at("pos_des")].points.size()) <
        kNMarker) {
      marker_ptr_->markers[idx_map_.at("pos_des")].points.push_back(p);
    } else {
      marker_ptr_->markers[idx_map_.at("pos_des")].points.push_back(p);
      for (int i = 0; i < kNMarker; i++) {
        marker_ptr_->markers[idx_map_.at("pos_des")].points.at(i) =
            marker_ptr_->markers[idx_map_.at("pos_des")].points.at(i + 1);
      }
      marker_ptr_->markers[idx_map_.at("pos_des")].points.pop_back();
    }
  }
}

void TargetFrameVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  target_sub_ = node_ptr_->create_subscription<hippo_msgs::msg::ControlTarget>(
      "traj_setpoint", qos,
      std::bind(&TargetFrameVisualization::onTarget, this,
                std::placeholders::_1));
}

void TargetFrameVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_des_pose_x;
  marker_des_pose_x.type = visualization_msgs::msg::Marker::ARROW;
  marker_des_pose_x.action = visualization_msgs::msg::Marker::ADD;
  marker_des_pose_x.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_des_pose_x.id = idx;
  marker_des_pose_x.color.a = 0.35;
  marker_des_pose_x.color.r = 1.0;
  marker_des_pose_x.color.g = 0.0;
  marker_des_pose_x.color.b = 0.0;
  marker_des_pose_x.scale.x = 0.02;
  marker_des_pose_x.scale.y = 0.05;
  marker_des_pose_x.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_des_pose_x.points.push_back(p);
  p.x = 1.0;
  marker_des_pose_x.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_des_pose_x);
  idx_map_["pose_des_x"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_des_pose_y;
  marker_des_pose_y.type = visualization_msgs::msg::Marker::ARROW;
  marker_des_pose_y.action = visualization_msgs::msg::Marker::ADD;
  marker_des_pose_y.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_des_pose_y.id = idx;
  marker_des_pose_y.color.a = 0.35;
  marker_des_pose_y.color.r = 0.0;
  marker_des_pose_y.color.g = 1.0;
  marker_des_pose_y.color.b = 0.0;
  marker_des_pose_y.scale.x = 0.02;
  marker_des_pose_y.scale.y = 0.05;
  marker_des_pose_y.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_des_pose_y.points.push_back(p);
  p.y = 1.0;
  marker_des_pose_y.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_des_pose_y);
  idx_map_["pose_des_y"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_des_pose_z;
  marker_des_pose_z.type = visualization_msgs::msg::Marker::ARROW;
  marker_des_pose_z.action = visualization_msgs::msg::Marker::ADD;
  marker_des_pose_z.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_des_pose_z.id = idx;
  marker_des_pose_z.color.a = 0.35;
  marker_des_pose_z.color.r = 0.0;
  marker_des_pose_z.color.g = 0.0;
  marker_des_pose_z.color.b = 1.0;
  marker_des_pose_z.scale.x = 0.02;
  marker_des_pose_z.scale.y = 0.05;
  marker_des_pose_z.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_des_pose_z.points.push_back(p);
  p.z = 1.0;
  marker_des_pose_z.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_des_pose_z);
  idx_map_["pose_des_z"] = idx;
}

void TargetFrameVisualization::onTarget(
    const hippo_msgs::msg::ControlTarget::SharedPtr msg) {
  geometry_msgs::msg::Point p;

  if (msg->mask != msg->IGNORE_POSE) {
    marker_ptr_->markers[idx_map_.at("pose_des_x")].header.stamp =
        msg->header.stamp;
    if (!marker_ptr_->markers[idx_map_.at("pose_des_x")].points.empty()) {
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_x")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_x")].pose.orientation =
          msg->attitude;
    } else {
      p.x = 0.0;
      p.y = 0.0;
      p.z = 0.0;
      marker_ptr_->markers[idx_map_.at("pose_des_x")].points.push_back(p);
      p.x = 1.0;
      marker_ptr_->markers[idx_map_.at("pose_des_x")].points.push_back(p);
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_x")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_x")].pose.orientation =
          msg->attitude;
    }
    marker_ptr_->markers[idx_map_.at("pose_des_y")].header.stamp =
        msg->header.stamp;
    if (!marker_ptr_->markers[idx_map_.at("pose_des_y")].points.empty()) {
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_y")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_y")].pose.orientation =
          msg->attitude;
    } else {
      p.x = 0.0;
      p.y = 0.0;
      p.z = 0.0;
      marker_ptr_->markers[idx_map_.at("pose_des_y")].points.push_back(p);
      p.y = 1.0;
      marker_ptr_->markers[idx_map_.at("pose_des_y")].points.push_back(p);
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_y")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_y")].pose.orientation =
          msg->attitude;
    }
    marker_ptr_->markers[idx_map_.at("pose_des_z")].header.stamp =
        msg->header.stamp;
    if (!marker_ptr_->markers[idx_map_.at("pose_des_z")].points.empty()) {
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_z")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_z")].pose.orientation =
          msg->attitude;
    } else {
      p.x = 0.0;
      p.y = 0.0;
      p.z = 0.0;
      marker_ptr_->markers[idx_map_.at("pose_des_z")].points.push_back(p);
      p.z = 1.0;
      marker_ptr_->markers[idx_map_.at("pose_des_z")].points.push_back(p);
      // set pose
      marker_ptr_->markers[idx_map_.at("pose_des_z")].pose.position =
          msg->position;
      marker_ptr_->markers[idx_map_.at("pose_des_z")].pose.orientation =
          msg->attitude;
    }
  }
}

void VelocityTargetVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  target_sub_ = node_ptr_->create_subscription<hippo_msgs::msg::ControlTarget>(
      "traj_setpoint", qos,
      std::bind(&VelocityTargetVisualization::onTarget, this,
                std::placeholders::_1));
}

void VelocityTargetVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_des_dir;
  marker_des_dir.type = visualization_msgs::msg::Marker::ARROW;
  marker_des_dir.action = visualization_msgs::msg::Marker::ADD;
  marker_des_dir.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_des_dir.id = idx;
  marker_des_dir.color.a = 0.7;
  marker_des_dir.color.r = 0.0;
  marker_des_dir.color.g = 0.0;
  marker_des_dir.color.b = 0.0;
  marker_des_dir.scale.x = 0.01;
  marker_des_dir.scale.y = 0.02;
  marker_des_dir.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_des_dir.points.push_back(p);
  p.x = 0.5;
  marker_des_dir.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_des_dir);
  idx_map_["vel_dir_des"] = idx;
}

void VelocityTargetVisualization::onTarget(
    const hippo_msgs::msg::ControlTarget::SharedPtr msg) {
  double norm =
      sqrt(std::pow(msg->velocity.x, 2) + std::pow(msg->velocity.y, 2) +
           std::pow(msg->velocity.z, 2));
  if (norm >= 0.001) {
    const double length = 0.5;
    marker_ptr_->markers[idx_map_.at("vel_dir_des")].header.stamp =
        msg->header.stamp;
    if (!marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.empty()) {
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(0).x =
          msg->position.x;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(0).y =
          msg->position.y;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(0).z =
          msg->position.z;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(1).x =
          length * msg->velocity.x / norm + msg->position.x;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(1).y =
          length * msg->velocity.y / norm + msg->position.y;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.at(1).z =
          length * msg->velocity.z / norm + msg->position.z;
    } else {
      geometry_msgs::msg::Point p;
      p.x = msg->position.x;
      p.y = msg->position.y;
      p.z = msg->position.z;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.push_back(p);
      p.x = length * msg->velocity.x / norm + msg->position.x;
      p.y = length * msg->velocity.y / norm + msg->position.y;
      p.z = length * msg->velocity.z / norm + msg->position.z;
      marker_ptr_->markers[idx_map_.at("vel_dir_des")].points.push_back(p);
    }
  }
}

void AUVPoseVisualization::initialize(
    rclcpp::Node* node_ptr,
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", qos,
      std::bind(&AUVPoseVisualization::onOdometry, this,
                std::placeholders::_1));
}

void AUVPoseVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_est;
  marker_est.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_est.action = visualization_msgs::msg::Marker::ADD;
  marker_est.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est.id = idx;
  marker_est.color.a = 0.7;
  marker_est.color.r = 1.0;
  marker_est.color.g = 0.0;
  marker_est.color.b = 0.0;
  marker_est.scale.x = 0.01;
  marker_est.scale.y = 0.01;
  marker_est.scale.z = 0.01;
  marker_est.pose.orientation.w = 1.0;
  marker_ptr_->markers.emplace_back(marker_est);
  idx_map_["pos_est"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_est_pose_x;
  marker_est_pose_x.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_x.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_x.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_x.id = idx;
  marker_est_pose_x.color.a = 0.7;
  marker_est_pose_x.color.r = 1.0;
  marker_est_pose_x.color.g = 0.0;
  marker_est_pose_x.color.b = 0.0;
  marker_est_pose_x.scale.x = 0.01;
  marker_est_pose_x.scale.y = 0.05;
  marker_est_pose_x.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_x.points.push_back(p);
  p.x = 1.0;
  marker_est_pose_x.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_x);
  idx_map_["pose_est_x"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_est_pose_y;
  marker_est_pose_y.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_y.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_y.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_y.id = idx;
  marker_est_pose_y.color.a = 0.7;
  marker_est_pose_y.color.r = 0.0;
  marker_est_pose_y.color.g = 1.0;
  marker_est_pose_y.color.b = 0.0;
  marker_est_pose_y.scale.x = 0.01;
  marker_est_pose_y.scale.y = 0.05;
  marker_est_pose_y.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_y.points.push_back(p);
  p.y = 1.0;
  marker_est_pose_y.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_y);
  idx_map_["pose_est_y"] = idx;
  idx++;

  visualization_msgs::msg::Marker marker_est_pose_z;
  marker_est_pose_z.type = visualization_msgs::msg::Marker::ARROW;
  marker_est_pose_z.action = visualization_msgs::msg::Marker::ADD;
  marker_est_pose_z.header.frame_id =
      hippo_common::tf2_utils::frame_id::kInertialName;
  marker_est_pose_z.id = idx;
  marker_est_pose_z.color.a = 0.7;
  marker_est_pose_z.color.r = 0.0;
  marker_est_pose_z.color.g = 0.0;
  marker_est_pose_z.color.b = 1.0;
  marker_est_pose_z.scale.x = 0.01;
  marker_est_pose_z.scale.y = 0.05;
  marker_est_pose_z.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_est_pose_z.points.push_back(p);
  p.z = 1.0;
  marker_est_pose_z.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_est_pose_z);
  idx_map_["pose_est_z"] = idx;
}
void AUVPoseVisualization::onOdometry(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  marker_ptr_->markers[idx_map_.at("pos_est")].header.stamp = msg->header.stamp;
  if (int(marker_ptr_->markers[idx_map_.at("pos_est")].points.size()) <
      kNMarker) {
    marker_ptr_->markers[idx_map_.at("pos_est")].points.push_back(
        msg->pose.pose.position);
  } else {
    marker_ptr_->markers[idx_map_.at("pos_est")].points.push_back(
        msg->pose.pose.position);
    for (int i = 0; i < kNMarker; i++) {
      marker_ptr_->markers[idx_map_.at("pos_est")].points.at(i) =
          marker_ptr_->markers[idx_map_.at("pos_est")].points.at(i + 1);
    }
    marker_ptr_->markers[idx_map_.at("pos_est")].points.pop_back();
  }

  marker_ptr_->markers[idx_map_.at("pose_est_x")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_x")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_x")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_x")].pose.orientation =
        msg->pose.pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_x")].points.push_back(p);
    p.x = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_x")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_x")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_x")].pose.orientation =
        msg->pose.pose.orientation;
  }
  marker_ptr_->markers[idx_map_.at("pose_est_y")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_y")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_y")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_y")].pose.orientation =
        msg->pose.pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_y")].points.push_back(p);
    p.y = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_y")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_y")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_y")].pose.orientation =
        msg->pose.pose.orientation;
  }
  marker_ptr_->markers[idx_map_.at("pose_est_z")].header.stamp =
      msg->header.stamp;
  if (!marker_ptr_->markers[idx_map_.at("pose_est_z")].points.empty()) {
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_z")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_z")].pose.orientation =
        msg->pose.pose.orientation;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("pose_est_z")].points.push_back(p);
    p.z = 1.0;
    marker_ptr_->markers[idx_map_.at("pose_est_z")].points.push_back(p);
    // set pose
    marker_ptr_->markers[idx_map_.at("pose_est_z")].pose.position =
        msg->pose.pose.position;
    marker_ptr_->markers[idx_map_.at("pose_est_z")].pose.orientation =
        msg->pose.pose.orientation;
  }
}

void AUVVelocityTargetVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  velocity_target_sub_ =
      node_ptr_->create_subscription<hippo_msgs::msg::VelocityControlTarget>(
          "velocity_setpoint", qos,
          std::bind(&AUVVelocityTargetVisualization::onVelocityTarget, this,
                    std::placeholders::_1));
}

void AUVVelocityTargetVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_des_dir;
  marker_des_dir.type = visualization_msgs::msg::Marker::ARROW;
  marker_des_dir.action = visualization_msgs::msg::Marker::ADD;
  marker_des_dir.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  marker_des_dir.id = idx;
  marker_des_dir.color.a = 0.7;
  marker_des_dir.color.r = 0.5;
  marker_des_dir.color.g = 0.0;
  marker_des_dir.color.b = 1.0;
  marker_des_dir.scale.x = 0.02;
  marker_des_dir.scale.y = 0.05;
  marker_des_dir.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_des_dir.points.push_back(p);
  p.x = 1.0;
  marker_des_dir.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_des_dir);
  idx_map_["vel_target"] = idx;
}

void AUVVelocityTargetVisualization::onVelocityTarget(
    const hippo_msgs::msg::VelocityControlTarget::SharedPtr msg) {
  double norm = sqrt(std::pow(msg->velocity.linear.x, 2) +
                     std::pow(msg->velocity.linear.y, 2) +
                     std::pow(msg->velocity.linear.z, 2));
  if (norm >= 0.001) {
    marker_ptr_->markers[idx_map_.at("vel_target")].header.stamp =
        msg->header.stamp;
    if (!marker_ptr_->markers[idx_map_.at("vel_target")].points.empty()) {
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(0).x = 0.0;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(0).y = 0.0;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(0).z = 0.0;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(1).x =
          msg->velocity.linear.x / norm;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(1).y =
          msg->velocity.linear.y / norm;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.at(1).z =
          msg->velocity.linear.z / norm;
    } else {
      geometry_msgs::msg::Point p;
      p.x = 0.0;
      p.y = 0.0;
      p.z = 0.0;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.push_back(p);
      p.x = msg->velocity.linear.x / norm;
      p.y = msg->velocity.linear.y / norm;
      p.z = msg->velocity.linear.z / norm;
      marker_ptr_->markers[idx_map_.at("vel_target")].points.push_back(p);
    }
  }
}

void AUVThrustVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  thrust_sub_ =
      node_ptr_->create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
          "thrust_setpoint", qos,
          std::bind(&AUVThrustVisualization::onThrust, this,
                    std::placeholders::_1));
}

void AUVThrustVisualization::initializeMarkers() {
  int idx = int(marker_ptr_->markers.size());
  geometry_msgs::msg::Point p;
  visualization_msgs::msg::Marker marker_thrust;
  marker_thrust.type = visualization_msgs::msg::Marker::ARROW;
  marker_thrust.action = visualization_msgs::msg::Marker::ADD;
  marker_thrust.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  marker_thrust.id = idx;
  marker_thrust.color.a = 0.7;
  marker_thrust.color.r = 0.0;
  marker_thrust.color.g = 1.0;
  marker_thrust.color.b = 0.1;
  marker_thrust.scale.x = 0.02;
  marker_thrust.scale.y = 0.05;
  marker_thrust.pose.orientation.w = 1.0;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker_thrust.points.push_back(p);
  p.x = 1.0;
  marker_thrust.points.push_back(p);
  marker_ptr_->markers.emplace_back(marker_thrust);
  idx_map_["thrust_dir"] = idx;
}

void AUVThrustVisualization::onThrust(
    const hippo_msgs::msg::ActuatorSetpoint::SharedPtr msg) {
  marker_ptr_->markers[idx_map_.at("thrust_dir")].header.stamp =
      msg->header.stamp;

  if (!marker_ptr_->markers[idx_map_.at("thrust_dir")].points.empty()) {
    marker_ptr_->markers[idx_map_.at("thrust_dir")].points.at(1).x = msg->x;
    marker_ptr_->markers[idx_map_.at("thrust_dir")].points.at(1).y = msg->y;
    marker_ptr_->markers[idx_map_.at("thrust_dir")].points.at(1).z = msg->z;
  } else {
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_ptr_->markers[idx_map_.at("thrust_dir")].points.push_back(p);
    p.x = msg->x;
    p.y = msg->y;
    p.z = msg->z;
    marker_ptr_->markers[idx_map_.at("thrust_dir")].points.push_back(p);
  }
}

void UVMSTargetVisualization::initialize(
    rclcpp::Node* node_ptr,
    visualization_msgs::msg::MarkerArray::SharedPtr marker_ptr) {
  node_ptr_ = node_ptr;
  marker_ptr_ = marker_ptr;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  uvms_target_sub_ =
      node_ptr_->create_subscription<uvms_msgs::msg::UVMSControlTarget>(
          "traj_setpoint_uvms", qos,
          std::bind(&UVMSTargetVisualization::onUVMSTarget, this,
                    std::placeholders::_1));
}

void UVMSTargetVisualization::onUVMSTarget(
    const uvms_msgs::msg::UVMSControlTarget::SharedPtr msg) {
  hippo_msgs::msg::ControlTarget auv_msg;
  auv_msg = msg->auv;
  auv_msg.header = msg->header;
  hippo_msgs::msg::ControlTarget::SharedPtr auv_msg_ptr =
      std::make_shared<hippo_msgs::msg::ControlTarget>(auv_msg);
  onTarget(auv_msg_ptr);
}
}  // namespace uvms_visualization
