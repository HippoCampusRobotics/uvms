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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "hippo_common/tf2_utils.hpp"
#include "bluerov_estimation/kf_interfaces.hpp"
#include "uvms_common/ros_param_utils.hpp"
using std::placeholders::_1;

namespace bluerov_estimation {
    class Estimation : public rclcpp::Node {
    public:

        Estimation() : Node("acceleration_estimation_node") {
            rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
            accelerations_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("accelerations", qos);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", qos,
                                                                               std::bind(&Estimation::odomCallback,
                                                                                         this, std::placeholders::_1));

            std::vector<int64_t> kf_types;
            ros_param_utils::getParamArray(this, kf_types, "kf_types", {1});
            for (int64_t kf_type : kf_types){
                switch (int(kf_type)){
                    case KFType::kf_linear:
                        kf_interfaces_.push_back(new KFLinearInterface());
                        break;
                    case KFType::ekf:
                        kf_interfaces_.push_back(new EKFInterface());
                        break;
                    case KFType::kf_feedforward:
                        kf_interfaces_.push_back(new KFFeedforwardInterface());
                        break;
                    default:
                        std::cerr << "KF key " << int(kf_type) << " does not match any of the available options!" << std::endl;
                        break;
                }
            }

            bool publish_debug_info;
            ros_param_utils::getParam(this, publish_debug_info, "publish_debug", false);

            for (KFInterface* kf_interface : kf_interfaces_){
                kf_interface->initialize(this, publish_debug_info);
            }
        };

    private:


        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            bool published_first = false;
            for (KFInterface* kf_interface : kf_interfaces_){
                bool was_initialized = kf_interface->isInitialized();
                kf_interface->update(msg);
                if (!was_initialized){
                    continue;
                }
                if (!published_first){
                    publishAccelerations(kf_interface->getAccelerations());
                    published_first = true;
                }
                kf_interface->publishDebugInfo();

            }
        }


        void publishAccelerations(const geometry_msgs::msg::Twist &twist_msg) {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
            msg.twist = twist_msg;
            accelerations_pub_->publish(msg);
        }
        std::vector<KFInterface*> kf_interfaces_; //!< interfaces to KF, need only one but makes option available to compare multiple simultaneously
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr accelerations_pub_;
    };
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bluerov_estimation::Estimation>());
    rclcpp::shutdown();
    return 0;
}
