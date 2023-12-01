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

#include "bluerov_ctrl/velocity_control_node_interfaces.hpp"

namespace bluerov_ctrl {

    void PIDCompInterface::initialize(rclcpp::Node *node_ptr) {
        node_ptr_ = node_ptr;
        velocity_control_interface_ = new TwistPIDInterface();
        manipulator_comp_interface_ = new ManipulatorCompInterface();
        velocity_control_interface_->initialize(node_ptr_);
        manipulator_comp_interface_->initialize(node_ptr_);
        declareParams();
    }

    void PIDCompInterface::setSetpointTarget(const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) {
        velocity_control_interface_->setVelocityTarget(_msg->velocity);
        if (_msg->use_acceleration) {
            velocity_control_interface_->setAccelerationTarget(_msg->acceleration);
        } else {
            geometry_msgs::msg::Twist zero_acceleration;
            zero_acceleration.linear.x = 0.0;
            zero_acceleration.linear.y = 0.0;
            zero_acceleration.linear.z = 0.0;
            zero_acceleration.angular.x = 0.0;
            zero_acceleration.angular.y = 0.0;
            zero_acceleration.angular.z = 0.0;
            velocity_control_interface_->setAccelerationTarget(zero_acceleration);
        }
    }

    void PIDCompInterface::getControllerOutput(const double &dt, const nav_msgs::msg::Odometry::SharedPtr msg,
                                               hippo_msgs::msg::ActuatorSetpoint &out_thrust,
                                               hippo_msgs::msg::ActuatorSetpoint &out_torque) {
        velocity_control_interface_->update(dt, *msg, out_thrust, out_torque);
        if (manipulator_comp_interface_->isActive()) {
            manipulator_comp_interface_->addCompensation(msg->pose.pose.orientation, out_thrust, out_torque);
        }
    }

    void PIDCompInterface::publishDebugMsgs() {
        velocity_control_interface_->publishDebugMsgs();
        manipulator_comp_interface_->publishDebugMsgs();
    }

    void PIDCompInterface::declareParams() {
        velocity_control_interface_->declareParams();
        manipulator_comp_interface_->declareParams();

        velocity_control_interface_->initializeParamCallbacks();
        manipulator_comp_interface_->initializeParamCallbacks();
    }

    void PIDCompInterface::checkStatus() {
        is_ok_ = !velocity_control_interface_->isAccelTimedOut() &&
                 (!manipulator_comp_interface_->isActive() || !manipulator_comp_interface_->isTimedOut());
    }


    void PIDCompInterface::onTimeout() {
        velocity_control_interface_->onTimeout();
    }


    void ModelBasedCompInterface::initialize(rclcpp::Node *node_ptr) {
        node_ptr_ = node_ptr;
        velocity_control_interface_ = new TwistModelBasedInterface();
        manipulator_comp_interface_ = new ManipulatorCompInterface();
        velocity_control_interface_->initialize(node_ptr_);
        manipulator_comp_interface_->initialize(node_ptr_);
        declareParams();
    }

    void ModelBasedCompInterface::setSetpointTarget(const hippo_msgs::msg::VelocityControlTarget::SharedPtr _msg) {
        velocity_control_interface_->setVelocityTarget(_msg->velocity);
        if (_msg->use_acceleration) {
            velocity_control_interface_->setAccelerationTarget(_msg->acceleration);
        } else {
            geometry_msgs::msg::Twist zero_acceleration;
            zero_acceleration.linear.x = 0.0;
            zero_acceleration.linear.y = 0.0;
            zero_acceleration.linear.z = 0.0;
            zero_acceleration.angular.x = 0.0;
            zero_acceleration.angular.y = 0.0;
            zero_acceleration.angular.z = 0.0;
            velocity_control_interface_->setAccelerationTarget(zero_acceleration);
        }
    }

    void ModelBasedCompInterface::getControllerOutput(const double &dt, const nav_msgs::msg::Odometry::SharedPtr msg,
                                                      hippo_msgs::msg::ActuatorSetpoint &out_thrust,
                                                      hippo_msgs::msg::ActuatorSetpoint &out_torque) {
        velocity_control_interface_->update(dt, *msg, out_thrust, out_torque);
        if (manipulator_comp_interface_->isActive()) {
            manipulator_comp_interface_->addCompensation(msg->pose.pose.orientation, out_thrust, out_torque);
        }
    }

    void ModelBasedCompInterface::publishDebugMsgs() {
        velocity_control_interface_->publishDebugMsgs();
        manipulator_comp_interface_->publishDebugMsgs();
    }

    void ModelBasedCompInterface::declareParams() {
        velocity_control_interface_->declareParams();
        manipulator_comp_interface_->declareParams();

        velocity_control_interface_->initializeParamCallbacks();
        manipulator_comp_interface_->initializeParamCallbacks();
    }


    void ModelBasedCompInterface::checkStatus() {
        is_ok_ = !velocity_control_interface_->isAccelTimedOut() &&
                 (!manipulator_comp_interface_->isActive() || !manipulator_comp_interface_->isTimedOut());
    }

    void ModelBasedCompInterface::onTimeout() {
        velocity_control_interface_->onTimeout();
    }

}

