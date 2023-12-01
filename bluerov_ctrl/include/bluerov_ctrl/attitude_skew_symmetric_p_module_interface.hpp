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

#ifndef BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_INTERFACE_HPP
#define BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_INTERFACE_HPP

#include "attitude_skew_symmetric_p_module.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/param_utils.hpp"

namespace bluerov_ctrl{

    class AttSkewSymmetricPModuleInterface {
    public:
        AttSkewSymmetricPModuleInterface() = default;
        void initialize(rclcpp::Node* node_ptr);
        void update(const geometry_msgs::msg::Quaternion &att, geometry_msgs::msg::Vector3 &out_rates);

        void setControlTarget(const hippo_msgs::msg::ControlTarget::SharedPtr msg);

        rcl_interfaces::msg::SetParametersResult onSetPgains(const std::vector<rclcpp::Parameter> &parameters);
        void declareParams();
        void initializeParamCallbacks();
    private:
        rclcpp::Node* node_ptr_;
        double gain_roll_p_;
        double gain_pitch_p_;
        double gain_yaw_p_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr p_gains_cb_handle_;
        AttSkewSymmetricPControlModule* controller_;
        std::mutex mutex_;
    };
}

#endif //BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_INTERFACE_HPP
