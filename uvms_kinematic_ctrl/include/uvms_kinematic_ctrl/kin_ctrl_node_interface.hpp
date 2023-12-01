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

#ifndef UVMS_KINEMATIC_CTRL_KIN_CTRL_NODE_INTERFACE_HPP
#define UVMS_KINEMATIC_CTRL_KIN_CTRL_NODE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "hippo_msgs/msg/control_target.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "alpha_msgs/msg/joint_data.hpp"
#include "kin_ctrl.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/param_utils.hpp"
#include "uvms_common/param_utils.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_msgs/msg/task_priority_debug.hpp"


namespace uvms_kin_ctrl {

    class TaskInterface {
    public:
        TaskInterface() = default;

        void initialize(rclcpp::Node *node_ptr, std::mutex *mutex_ptr, const int &task_type,
                        const bool &publish_task_velocities);

        virtual void addReferenceModel(uvms_kinematics::Kinematics *model) = 0;

        virtual void addParameterCallback() {};

        void publishCmds();

        virtual Task *getTaskPtr() = 0;

    protected:
        virtual void initializeTask() = 0;

        rclcpp::Node *node_ptr_;
        std::mutex *mutex_ptr_;
        rclcpp::Publisher<uvms_msgs::msg::TaskPriorityDebug>::SharedPtr debug_pub_;
    };

    class JointLimitTaskInterface : public TaskInterface {
    public:
        JointLimitTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        JointLimitTask task_;
        StateVector gains_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;

    };

    class JointLimitDesiredTaskInterface : public TaskInterface {
    public:
        JointLimitDesiredTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        JointLimitDesiredTask task_;
        StateVector gains_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;

    };

    class JointCenteringTaskInterface : public TaskInterface {
    public:
        JointCenteringTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        JointCenteringTask task_;
        StateVector gains_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;


    };

    class VelocityTaskInterface : public TaskInterface {
    public:
        VelocityTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

    private:

        void initializeTask() override;
        VelocityTask task_;
    };

    class SelfCollisionEllipseTaskInterface : public TaskInterface {
    public:
        SelfCollisionEllipseTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;


    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        SelfCollisionEllipseTask task_;  //!< two tasks as collision is tested for two links of the manipulator
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };


    class RestrictingPlaneTaskInterface : public TaskInterface{
    public:
        RestrictingPlaneTaskInterface() = default;
        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;


    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);
        RestrictingPlaneTask task_;  //!< two tasks as collision is tested for two links of the manipulator
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };



    class AUVConfinedSpaceTaskInterface : public TaskInterface {
    public:
        AUVConfinedSpaceTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;


    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        AUVConfinedSpaceTask task_;  //!< two tasks as collision is tested for two links of the manipulator
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };

    class AUVAttitudeLimitTaskInterface : public TaskInterface {
    public:
        AUVAttitudeLimitTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        AUVAttitudeLimitTask task_;
        Eigen::Matrix<double, 2, 1> gains_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;

    };

    class AUVAttitudeInclinationLimitTaskInterface : public TaskInterface {
    public:
        AUVAttitudeInclinationLimitTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGain(const std::vector<rclcpp::Parameter> &parameters);

        AUVAttitudeInclinationLimitTask task_;
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };

    class EndeffectorTrackingTaskInterface : public TaskInterface {
    public:
        EndeffectorTrackingTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addReferences(Eigen::Vector3d *p_eef_des, Eigen::Quaterniond *att_eef_des,
                           Eigen::Matrix<double, 6, 1> *vels_eef_des, Eigen::Matrix<bool, 6, 1> *mask) {
            task_.addReferences(p_eef_des, att_eef_des, vels_eef_des, mask);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetPgains(const std::vector<rclcpp::Parameter> &parameters);

        EndeffectorTrackingTask task_;
        Vector6d p_gains_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr p_gains_cb_handle_;
    };


    class ManipulabilityTaskInterface : public TaskInterface {
    public:
        ManipulabilityTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        ManipulabilityTask task_;
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };


    class ManipulabilityLimitTaskInterface : public TaskInterface {
    public:
        ManipulabilityLimitTaskInterface() = default;

        Task *getTaskPtr() override {
            return &task_;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) override {
            task_.addReferenceModel(model);
        }

        void addParameterCallback() override;

    private:
        void initializeTask() override;

        rcl_interfaces::msg::SetParametersResult onSetGains(const std::vector<rclcpp::Parameter> &parameters);

        ManipulabilityLimitTask task_;
        double gain_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_cb_handle_;

    };

    class UVMSKinematicControlInterface {
    public:
        UVMSKinematicControlInterface() = default;

        void initialize(rclcpp::Node *node_ptr);

        void initializeController();

        //! Request end-effector pose, assumes that the pose was manually updated before based on given state information
        //! \param out end-effector position and orientation in world-COS
        void getEndeffectorPose(geometry_msgs::msg::Pose &out);

        //! Request end-effector pose based on given state information
        //! \param auv_msg information about auv pose
        //! \param manipulator_msg joint state information
        //! \param out end-effector pose in world-COS
        void getEndeffectorPose(const nav_msgs::msg::Odometry &auv_msg,
                                const sensor_msgs::msg::JointState &manipulator_msg,
                                geometry_msgs::msg::Pose &out);

        void setSetpointTarget(const hippo_msgs::msg::ControlTarget::SharedPtr _msg);

        void setSetpointTargetForward(const hippo_msgs::msg::ControlTarget::SharedPtr _msg, const double &dt);

        void getControllerOutput(const nav_msgs::msg::Odometry &auv_msg,
                                 const sensor_msgs::msg::JointState &manipulator_msg,
                                 geometry_msgs::msg::TwistStamped &out_auv_cmds,
                                 alpha_msgs::msg::JointData &out_manipulator_cmds);

        void getControllerOutputWithDerivative(const double &dt, const nav_msgs::msg::Odometry &auv_msg,
                                          const sensor_msgs::msg::JointState &manipulator_msg,
                                          geometry_msgs::msg::Twist &out_auv_vel_cmds,
                                          geometry_msgs::msg::Twist &out_auv_acc_cmds,
                                          alpha_msgs::msg::JointData &out_manipulator_cmds);

        rcl_interfaces::msg::SetParametersResult onSetWeightingParams(const std::vector<rclcpp::Parameter> &parameters);


        rcl_interfaces::msg::SetParametersResult onSetAccelSmoothingFactors(const std::vector<rclcpp::Parameter> &parameters);

        void onSetpointTimeout();

    private:
        void addParameterCallback();

        rcl_interfaces::msg::SetParametersResult onSetSVDParam(const std::vector<rclcpp::Parameter> &parameters);
        rcl_interfaces::msg::SetParametersResult onSetEta(const std::vector<rclcpp::Parameter> &parameters);

        rclcpp::Node *node_ptr_;
        bool publish_task_velocities_;  //!< decides if for each task a publisher is created that publishes the (unsaturated!!) cmd vel
        UVMSKinematicCtrl controller_;
        std::vector<TaskInterface *> task_interfaces_;
        std::mutex mutex_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr svd_param_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr eta_param_cb_handle_;
        UVMSStateVector weighting_params_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr weighting_cb_handle_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr accel_smoothing_cb_handle_;

    };


}


#endif //UVMS_KINEMATIC_CTRL_KIN_CTRL_NODE_INTERFACE_HPP
