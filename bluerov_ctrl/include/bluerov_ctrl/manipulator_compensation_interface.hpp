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

#ifndef BLUEROV_CTRL_MANIPULATOR_COMPENSATION_INTERFACE_HPP
#define BLUEROV_CTRL_MANIPULATOR_COMPENSATION_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <eigen3/Eigen/Dense>
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "manipulator_compensation.hpp"

namespace bluerov_ctrl {
    class ManipulatorCompInterface {
    public:
        ManipulatorCompInterface() = default;

        void initialize(rclcpp::Node *node_ptr);

        const bool &isActive() {
            return active_;
        }

        void setActive(bool active) {
            active_ = active;
        }

        bool isTimedOut() {
            return timed_out_;
        }

        void addCompensation(const geometry_msgs::msg::Quaternion &msg, hippo_msgs::msg::ActuatorSetpoint &out_thrust,
                             hippo_msgs::msg::ActuatorSetpoint &out_torque);

        void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

        void declareParams();

        void initializeParamCallbacks();

        void publishDebugMsgs();

    private:
        void onTimeout();

        rclcpp::Node *node_ptr_;

        rcl_interfaces::msg::SetParametersResult onSetActive(const std::vector<rclcpp::Parameter> &parameters);

        geometry_msgs::msg::Wrench debug_msg_;
        bool active_;
        bool first_wrench_msg_;
        ManipulatorCompensation *manipulator_compensation_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        bool timed_out_ = false;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr debug_compensation_pub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr activation_cb_handle_;
    };

}

#endif //BLUEROV_CTRL_MANIPULATOR_COMPENSATION_INTERFACE_HPP
