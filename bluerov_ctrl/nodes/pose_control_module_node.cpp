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

#ifndef BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP
#define BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include "bluerov_ctrl/attitude_skew_symmetric_p_module_interface.hpp"
#include "bluerov_ctrl/position_p_module_interface.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "hippo_common/tf2_utils.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "hippo_msgs/msg/velocity_control_target.hpp"
#include <eigen3/Eigen/Dense>

namespace bluerov_ctrl {
    using std::placeholders::_1;
    class AUVPoseControlModuleNode : public rclcpp::Node {
    public:
        AUVPoseControlModuleNode() : Node("auv_position_control_module_node"){
            RCLCPP_INFO(this->get_logger(), "Declaring parameters.");
            declareParams();
            initController();
            initPublishers();
            initTimers();
            initSubscriptions();
        }

    private:
        void initTimers() {
            setpoint_timeout_timer_ = rclcpp::create_timer(
                    this, get_clock(), std::chrono::milliseconds(500),
                    std::bind(&AUVPoseControlModuleNode::onSetpointTimeout, this));
        }
        void initController(){
            pos_control_module_interface = new PosPModuleInterface();
            att_control_module_interface = new AttSkewSymmetricPModuleInterface();
            pos_control_module_interface->initialize(this);
            att_control_module_interface->initialize(this);
            pos_control_module_interface->declareParams();
            att_control_module_interface->declareParams();
            pos_control_module_interface->initializeParamCallbacks();
            att_control_module_interface->initializeParamCallbacks();
        }
        void declareParams(){

        }
        void initPublishers(){
            std::string topic;

            topic = "velocity_setpoint";
            velocity_setpoint_pub_ = create_publisher<hippo_msgs::msg::VelocityControlTarget>(
                    topic, rclcpp::SensorDataQoS());
        }

        void initSubscriptions(){
            std::string topic;
            rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

            topic = "traj_setpoint";
            target_sub_ = create_subscription<hippo_msgs::msg::ControlTarget>(
                    topic, qos,
                    std::bind(&AUVPoseControlModuleNode::onSetpointTarget, this, _1));

            topic = "odometry";
            odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    topic, qos, std::bind(&AUVPoseControlModuleNode::onOdometry, this, _1));

        }

        void onSetpointTimeout(){
            if (setpoint_timed_out_) {
                return;
            }
            RCLCPP_WARN(get_logger(), "Setpoint timed out. Stop sending commands.");
            setpoint_timed_out_ = true;
        }
        void onSetpointTarget(const hippo_msgs::msg::ControlTarget::SharedPtr _msg){
            if (_msg->header.frame_id != hippo_common::tf2_utils::frame_id::kInertialName) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000, "%s",
                        ("Control Targe frame is " + _msg->header.frame_id + "but only " + hippo_common::tf2_utils::frame_id::kInertialName + "is handled. Ignoring...").c_str());
                return;
            }
            if (!got_first_setpoint_) {
                got_first_setpoint_ = true;
                return;
            }

            setpoint_timeout_timer_->reset();
            if (setpoint_timed_out_) {
                RCLCPP_INFO(get_logger(),
                            "Received setpoint. Setpoint not timed out anymore.");
                setpoint_timed_out_ = false;
            }

            pos_control_module_interface->setControlTarget(_msg);
            att_control_module_interface->setControlTarget(_msg);
        }


        void onOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg){
            if (!got_first_setpoint_){
                return;
            }
            if (setpoint_timed_out_){
                return;
            }
            hippo_msgs::msg::VelocityControlTarget velocity_setpoint_msg;
            velocity_setpoint_msg.header.stamp = this->now();
            velocity_setpoint_msg.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
            pos_control_module_interface->update(_msg->pose.pose.position, _msg->pose.pose.orientation, velocity_setpoint_msg.velocity.linear);
            att_control_module_interface->update(_msg->pose.pose.orientation, velocity_setpoint_msg.velocity.angular);
            velocity_setpoint_msg.use_acceleration = false;
            velocity_setpoint_msg.acceleration.linear.x = 0.0;
            velocity_setpoint_msg.acceleration.linear.y = 0.0;
            velocity_setpoint_msg.acceleration.linear.z = 0.0;
            velocity_setpoint_msg.acceleration.angular.x = 0.0;
            velocity_setpoint_msg.acceleration.angular.y = 0.0;
            velocity_setpoint_msg.acceleration.angular.z = 0.0;
            velocity_setpoint_pub_->publish(velocity_setpoint_msg);

        }



        std::mutex mutex_;

        //////////////////////////////////////////////////////////////////////////////
        // ros params
        //////////////////////////////////////////////////////////////////////////////

        PosPModuleInterface* pos_control_module_interface;
        AttSkewSymmetricPModuleInterface* att_control_module_interface;
        hippo_msgs::msg::ControlTarget setpoint_target_;
        bool setpoint_timed_out_{false};

        bool got_first_setpoint_;

        rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;

        //////////////////////////////////////////////////////////////////////////////
        // publishers
        //////////////////////////////////////////////////////////////////////////////
        rclcpp::Publisher<hippo_msgs::msg::VelocityControlTarget>::SharedPtr velocity_setpoint_pub_;

        //////////////////////////////////////////////////////////////////////////////
        // subscriptions
        //////////////////////////////////////////////////////////////////////////////
        rclcpp::Subscription<hippo_msgs::msg::ControlTarget>::SharedPtr target_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    };
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bluerov_ctrl::AUVPoseControlModuleNode>());
    rclcpp::shutdown();
}

#endif // BLUEROV_CTRL_POSITION_CONTROL_MODULE_NODE_HPP
