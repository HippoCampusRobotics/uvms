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

#ifndef BLUEROV_ESTIMATION_KF_INTERFACE_HPP
#define BLUEROV_ESTIMATION_KF_INTERFACE_HPP

#include "kf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "uvms_common/ros_param_utils.hpp"
#include "hippo_common/param_utils.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/actuator_setpoint.hpp"

namespace bluerov_estimation {

    enum KFType : int{
        kf_linear = 1,
        ekf = 2,
        kf_feedforward = 3,
    };

    class KFInterface {
    public:
        KFInterface(){};

        bool isInitialized(){
            return initialized_;
        }

        virtual void initialize(rclcpp::Node* node_ptr, bool publish_debug_info) = 0;

        virtual void initializeKF() = 0;

        virtual void initializeParamCallbacks() = 0;

        const geometry_msgs::msg::Twist& getAccelerations(){
            return out_accelerations_;
        }

        virtual void update(const nav_msgs::msg::Odometry::SharedPtr &msg) = 0;

        virtual void publishDebugInfo() = 0;


    protected:
        virtual void loadNoiseParam() = 0;

        virtual rcl_interfaces::msg::SetParametersResult
        onSetNoiseParamCallback(const std::vector<rclcpp::Parameter> &parameters) = 0;

        rclcpp::Time last_stamp_;
        bool initialized_ = false;
        bool publish_debug_info_ = false;
        rclcpp::Node *node_ptr_;
        std::mutex mutex_;
        geometry_msgs::msg::Twist debug_velocities_;
        geometry_msgs::msg::Twist out_accelerations_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_velocities_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_accelerations_pub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr noise_cb_handle_;
    };


    class KFLinearInterface : public KFInterface {
    public:
        KFLinearInterface(){};
        void initialize(rclcpp::Node* node_ptr, bool publish_debug_info) override;

        void initializeKF() override;

        void initializeParamCallbacks() override;

        void update(const nav_msgs::msg::Odometry::SharedPtr &msg) override;

        void publishDebugInfo() override;
    private:
        void loadNoiseParam() override;

        rcl_interfaces::msg::SetParametersResult
        onSetNoiseParamCallback(const std::vector<rclcpp::Parameter> &parameters) override;

        KFLinear::KFMeasVector z_lin_;
        KFLinear::KFMeasVector z_ang_;
        KFLinear kf_lin_;
        KFLinear kf_ang_;
        KFLinear::vNoise v_noise_lin_;
        KFLinear::vNoise v_noise_ang_;
        KFLinear::wNoise w_noise_lin_;
        KFLinear::wNoise w_noise_ang_;

    };

    class EKFInterface : public KFInterface {
    public:
        EKFInterface(){};
        void initialize(rclcpp::Node* node_ptr, bool publish_debug_info) override;

        void initializeKF() override;

        void initializeParamCallbacks() override;

        void update(const nav_msgs::msg::Odometry::SharedPtr &msg) override;

        void publishDebugInfo() override;
    private:
        void loadNoiseParam() override;

        void instantiateKF();

        rcl_interfaces::msg::SetParametersResult
        onSetNoiseParamCallback(const std::vector<rclcpp::Parameter> &parameters) override;

        void thrustSetpointCallback(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr msg);

        void torqueSetpointCallback(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr msg);

        void manipulatorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

        rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_setpoint_sub_;
        rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_setpoint_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_manipulator_sub_;
        EKF::KFMeasVector z_;
        EKF* kf_;
        EKF::vNoise v_noise_;
        EKF::wNoise w_noise_;
        Vector6d tau_thrusters_;
        Vector6d tau_manipulator_;

    };

    class KFFeedforwardInterface : public KFInterface {
    public:
        KFFeedforwardInterface(){};
        void initialize(rclcpp::Node* node_ptr, bool publish_debug_info) override;

        void initializeKF() override;

        void initializeParamCallbacks() override;

        void update(const nav_msgs::msg::Odometry::SharedPtr &msg) override;

        void publishDebugInfo() override;
    private:
        void loadNoiseParam() override;

        void instantiateKF();

        rcl_interfaces::msg::SetParametersResult
        onSetNoiseParamCallback(const std::vector<rclcpp::Parameter> &parameters) override;

        rcl_interfaces::msg::SetParametersResult
        onSetMovingAverageParamCallback(const std::vector<rclcpp::Parameter> &parameters);

        void thrustSetpointCallback(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr msg);

        void torqueSetpointCallback(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr msg);

        void manipulatorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

        rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_setpoint_sub_;
        rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_setpoint_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_manipulator_sub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr moving_average_cb_handle_;
        KFFeedforward::KFMeasVector z_;
        KFFeedforward* kf_;
        KFFeedforward::vNoise v_noise_;
        KFFeedforward::wNoise w_noise_;
        Vector6d tau_thrusters_;
        Vector6d tau_manipulator_;

    };
}

#endif //BLUEROV_ESTIMATION_KF_INTERFACE_HPP
