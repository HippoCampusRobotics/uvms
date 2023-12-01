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

#ifndef BLUEROV_CTRL_TWIST_MODEL_BASED_INTERFACE_HPP
#define BLUEROV_CTRL_TWIST_MODEL_BASED_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "hippo_msgs/msg/actuator_setpoint.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_common/convert.hpp"
#include "twist_model_based.hpp"

namespace bluerov_ctrl {

    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    class TwistModelBasedInterface {
    public:
        TwistModelBasedInterface() = default;

        void initialize(rclcpp::Node *node_ptr);

        void initializeController();

        bool isAccelTimedOut() {
            return accel_timed_out_;
        }

        void update(const double &dt, const nav_msgs::msg::Odometry &msg, hippo_msgs::msg::ActuatorSetpoint &out_thrust,
                    hippo_msgs::msg::ActuatorSetpoint &out_torque);

        void setVelocityTarget(const geometry_msgs::msg::Twist &msg);

        void setAccelerationTarget(const geometry_msgs::msg::Twist &msg);

        void onTimeout();

        void declareParams();

        void initializeParamCallbacks();

        void publishDebugMsgs();

    private:

        void onAccelerations(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

        void onAccelTimeout();

        void publishDebugActuator(const rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr &thrust_pub,
                                  const rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr &torque_pub,
                                  const Vector6d &data);

        rcl_interfaces::msg::SetParametersResult onSetPgains(const std::vector<rclcpp::Parameter> &parameters);

        rcl_interfaces::msg::SetParametersResult onSetIgains(const std::vector<rclcpp::Parameter> &parameters);

        rcl_interfaces::msg::SetParametersResult onSetDgains(const std::vector<rclcpp::Parameter> &parameters);


        rcl_interfaces::msg::SetParametersResult
        onUseAccelFeedforward(const std::vector<rclcpp::Parameter> &parameters);

        rcl_interfaces::msg::SetParametersResult
        onDUseAccelFeedforward(const std::vector<rclcpp::Parameter> &parameters);

        rcl_interfaces::msg::SetParametersResult onDUseAccelEstimation(const std::vector<rclcpp::Parameter> &parameters);


        TwistModelBasedControl *controller_;
        rclcpp::Node *node_ptr_;
        std::mutex mutex_;

        geometry_msgs::msg::TwistStamped acceleration_msg_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr accelerations_sub_;

        rclcpp::TimerBase::SharedPtr accel_timeout_timer_;
        bool accel_timed_out_ = false;

        // debug publishers for gains
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_thrust_p_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_thrust_i_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_thrust_d_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_thrust_model_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_torque_p_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_torque_i_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_torque_d_pub_;
        rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr debug_torque_model_pub_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr p_gains_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr i_gains_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr d_gains_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr use_accel_feedforward_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr d_use_accel_feedforward_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr d_use_accel_estimation_cb_handle_;

        Vector6d gain_p_;
        Vector6d gain_i_;
        Vector6d gain_d_;
    };
}

#endif //BLUEROV_CTRL_TWIST_MODEL_BASED_INTERFACE_HPP
