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

#ifndef BLUEROV_CTRL_VELOCITY_CONTROL_NODE_HPP
#define BLUEROV_CTRL_VELOCITY_CONTROL_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include "bluerov_ctrl/velocity_control_node_interfaces.hpp"
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_msgs/msg/velocity_control_target.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include "uvms_common/ros_param_utils.hpp"

namespace bluerov_ctrl {
    using std::placeholders::_1;
    using rcl_interfaces::msg::SetParametersResult;

    class AUVVelocityControlNode : public rclcpp::Node {
    public:
        AUVVelocityControlNode();

    private:
        void initTimers();
        void initController();
        void declareParams();
        void initPublishers();
        void initSubscriptions();
        hippo_msgs::msg::ActuatorSetpoint zeroMsg(rclcpp::Time _stamp);
        void onSetpointTimeout();
        void onStateTimeout();
        void onSetpointTarget(const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg);
        void onOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
        void onEstimationDriftShutdown(
                const std_msgs::msg::Bool::SharedPtr _msg);



        std::mutex mutex_;

        //////////////////////////////////////////////////////////////////////////////
        // ros params
        //////////////////////////////////////////////////////////////////////////////

        ControllerNodeInterface* controller_interface_;
        int controller_type_;
        geometry_msgs::msg::TwistStamped setpoint_target_;
        bool setpoint_timed_out_{false};
        bool states_timed_out_{false};

        bool estimation_feasible_ = true;
        bool got_first_setpoint_;
        bool requested_first_control_output_{false};
        rclcpp::Time last_time_;

        rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;
        rclcpp::TimerBase::SharedPtr state_timeout_timer_;

        //////////////////////////////////////////////////////////////////////////////
        // publishers
        //////////////////////////////////////////////////////////////////////////////
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;
        //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;

        //////////////////////////////////////////////////////////////////////////////
        // subscriptions
        //////////////////////////////////////////////////////////////////////////////
        rclcpp::Subscription<hippo_msgs::msg::VelocityControlTarget>::SharedPtr target_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
                estimation_drift_shutdown_sub_;
    };

}

#endif //BLUEROV_CTRL_VELOCITY_CONTROL_NODE_HPP
