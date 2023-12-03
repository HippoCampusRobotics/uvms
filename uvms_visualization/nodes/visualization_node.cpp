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

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hippo_msgs/msg/control_target.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_visualization/visualizations.hpp"

namespace visualization {
using std::placeholders::_1;
using namespace std::chrono_literals;

class VisualizationNode : public rclcpp::Node {
 public:
  VisualizationNode() : Node("visualization_node") {
    marker_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
    initializeModules();
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    timer_ = rclcpp::create_timer(
        this, this->get_clock(), 50ms,
        std::bind(&VisualizationNode::publishMarkers, this));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "viz_marker", qos);
  }

  void initializeModules() {
    std::vector<int64_t> visualization_modules;
    ros_param_utils::getParamArray(this, visualization_modules,
                                   "visualization_modules", {1});
    for (auto& visualization_module : visualization_modules) {
      switch (visualization_module) {
        case uvms_visualization::visualization_modules::endeffector:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::EndeffectorVisualization>());
          break;
        case uvms_visualization::visualization_modules::target:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::TargetVisualization>());
          break;
        case uvms_visualization::visualization_modules::velocity_target:
          modules_.emplace_back(
              std::make_unique<
                  uvms_visualization::VelocityTargetVisualization>());
          break;
        case uvms_visualization::visualization_modules::auv_velocity_target:
          modules_.emplace_back(
              std::make_unique<
                  uvms_visualization::AUVVelocityTargetVisualization>());
          break;
        case uvms_visualization::visualization_modules::auv_thrust:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::AUVThrustVisualization>());
          break;
        case uvms_visualization::visualization_modules::auv_pose:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::AUVPoseVisualization>());
          break;
        case uvms_visualization::visualization_modules::uvms_target:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::UVMSTargetVisualization>());
          break;
        case uvms_visualization::visualization_modules::endeffector_frame:
          modules_.emplace_back(
              std::make_unique<
                  uvms_visualization::EndeffectorFrameVisualization>());
          break;
        case uvms_visualization::visualization_modules::target_frame:
          modules_.emplace_back(
              std::make_unique<uvms_visualization::TargetFrameVisualization>());
          break;
        case uvms_visualization::visualization_modules::endeffector_axis:
          modules_.emplace_back(
              std::make_unique<
                  uvms_visualization::EndeffectorAxisVisualization>());
          break;
        default:
          RCLCPP_ERROR(this->get_logger(),
                       "Unknown visualization module, aborting initialization");
          return;
      }
    }
    for (auto& vis_module : modules_) {
      vis_module->initialize(this, marker_);
      vis_module->initializeMarkers();
    }
  }

 private:
  void publishMarkers() { marker_pub_->publish(*marker_.get()); }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  visualization_msgs::msg::MarkerArray::SharedPtr marker_;
  std::vector<std::unique_ptr<uvms_visualization::Visualization>> modules_;
};
}  // namespace visualization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visualization::VisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
