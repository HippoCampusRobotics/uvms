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

#ifndef UVMS_CTRL_KIN_CTRL_HPP
#define UVMS_CTRL_KIN_CTRL_HPP

#include "kinematics.hpp"
#include "uvms_common/quartic.hpp"
#include <chrono>

namespace uvms_kin_ctrl {
    enum TaskKey : int {
        joint_limits = 1,
        endeffector_tracking = 2,
        self_collision_planes = 4,
        joint_centering = 5,
        manipulability = 6,
        auv_attitude_limits = 7,
        self_collision_ellipse = 8,
        auv_confined_space = 9,
        joint_velocity = 10,
        manipulability_limit = 11,
        auv_attitude_inclination_limits = 12,
        joint_limits_desired = 13,
        restricting_plane = 14
    };

    const std::unordered_map<int, std::string> taskNames = {{TaskKey::joint_limits,           "joint_limits"},
                                                            {TaskKey::endeffector_tracking,   "eef_tracking"},
                                                            {TaskKey::self_collision_planes,  "self_collision_planes"},
                                                            {TaskKey::joint_centering,        "joint_centering"},
                                                            {TaskKey::manipulability,         "manipulability"},
                                                            {TaskKey::manipulability_limit,   "manipulability_limit"},
                                                            {TaskKey::auv_attitude_limits,    "auv_attitude_limits"},
                                                            {TaskKey::self_collision_ellipse, "self_collision_ellipse"},
                                                            {TaskKey::auv_confined_space,     "auv_confined_space"},
                                                            {TaskKey::joint_velocity,         "joint_velocity"},
                                                            {TaskKey::auv_attitude_inclination_limits,    "auv_attitude_inclination_limits"},
                                                            {TaskKey::joint_limits_desired,         "joint_limits_desired"},
                                                            {TaskKey::restricting_plane,         "restricting_plane"}};

    enum AlgorithmType {
        siciliano = 0,
        nakamura = 1,
        singularity_robust = 2,
        singularity_robust_augmented = 3,
        continuous = 4,
    };

    using param_utils::StateVector;
    using param_utils::n_active_joints;
    using param_utils::n_states_uvms;
    using param_utils::UVMSStateVector;
    using param_utils::UVMSStateMatrix;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    //Eigen::Matrix<double, n_states_uvms, n_states_uvms> &getProjector(const Eigen::MatrixBase<double> &J);
    enum TaskType : uint8_t {
        critical = 1,
        high_priority = 2,  //!< critical tasks (e.g. for safety). These tasks must be independent, which is tested onlinefMan
        low_priority = 3,  //!< general lower priority objectives
        optimization = 4  //!< optimization tasks are used for the remaining DOF at the end of the task-priority chain.
        //!< All assigned tasks are added as a combined optimization goal
    };


    inline Eigen::MatrixXd
    pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd> &J) {
        return J.completeOrthogonalDecomposition().pseudoInverse();
    }

    inline double sigmoid(const double &x, const double &x_min, const double &buffer) {
        if (x < x_min) {
            return 1.0;
        } else if (x > x_min + buffer) {
            return 0.0;
        } else {
            return 0.5 * (cos((x - x_min) * M_PI / buffer) + 1);
        }
    }

    inline double lowerBoundSigmoid(const double &x, const double &x_min, const double &buffer) {
        return sigmoid(x, x_min, buffer);
    }

    inline double upperBoundSigmoid(const double &x, const double &x_max, const double &buffer) {
        return sigmoid(-x, -x_max, buffer);
    }

    void
    checkAlgorithmicSingularity(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> &jacobian,
                                const UVMSStateMatrix &projector, int task_idx, int task_type);


    class Task {
    public:
        Task() = default;

        virtual int getTaskIdx() = 0;

        bool isActive() {
            return active_ > 0;
        }

        int countActive() {
            return active_;
        }

        void setIsContinuous(bool is_continuous) {
            is_continuous_ = is_continuous;
        }

        void addReferenceModel(uvms_kinematics::Kinematics *model) {
            model_ = model;
        }

        int getActiveTaskDimension() {
            return int(map_.rows());
        }

        int getFullTaskDimension() {
            return int(J_.rows());
        }

        int getActiveJacobianRank() {
            return int((map_ * J_).colPivHouseholderQr().rank());
        }

        void getActiveJacobian(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> J) {
            J = map_ * J_;
        }


        Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> getActiveJacobian() {
            return map_ * J_;
        }

        Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> getFullJacobian() {
            return J_;
        }

        Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> getActiveDesiredTaskVelocity() {
            return map_ * dx_;
        }

        Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> getFullDesiredTaskVelocity() {
            return dx_;
        }

        const Eigen::DiagonalMatrix<double, Eigen::Dynamic> &getActivationMatrix() {
            return A_;
        }

        const Eigen::DiagonalMatrix<double, Eigen::Dynamic> &getTaskWeightingMatrix() {
            return P0_sqrt_;
        }

        virtual void update() = 0;

        const UVMSStateVector &getDesiredVelocities() {
            return desired_vel_;
        }

        void setProjectedVelocities(const UVMSStateVector &vel) {
            projected_vel_ = vel;
        }

        const UVMSStateVector &getProjectedVelocities() {
            return projected_vel_;
        }

        void setSVDDamping(const double &k_svd_damping) {
            k_svd_damping_ = k_svd_damping;
        }

        void setMinSingularValue(const double &sigma_min) {
            sigma_min_ = sigma_min;
        }

        void setKAlphaDyn(const double &k_alpha_dyn) {
            k_alpha_dyn_ = k_alpha_dyn;
        }

        void setKAlphaMax(const double &k_alpha_max) {
            k_alpha_max_ = k_alpha_max;
        }

        Eigen::Matrix<double, n_states_uvms, Eigen::Dynamic>
        continuityOperator(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> &X,
                           const UVMSStateMatrix &Q, const UVMSStateMatrix &W_sqrt_inv, const double &eta);

    private:
        double singularValueRegularization(const double &sigma, const double &weighting_factor);

    protected:
        void updateMapping();

        void updateActiveCounter();

        bool
        lowerBoundActivationFunction(const double &state, const double &limit, const double &delta, const double &alpha,
                                     double &activation);

        bool
        upperBoundActivationFunction(const double &state, const double &limit, const double &delta, const double &alpha,
                                     double &activation);

        bool is_continuous_;
        int active_;  //!< number of active tasks
        uvms_kinematics::Kinematics *model_;
        param_utils::UVMSStateVector desired_vel_; //!< desired velocity from task
        param_utils::UVMSStateVector projected_vel_; //!< desired velocity from task
        Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms> J_; //!< task jacobian
        Eigen::Matrix<double, Eigen::Dynamic, 1> dx_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> map_; //!< map mapping jacobian to only active states
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> A_;
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> P0_sqrt_; //!< task weighting matrix (only used for final velocity minimization)
        double k_svd_damping_; //!< damping factor for SVD regularization
        double sigma_min_; //!< minimum singular value for SVD regularization
        double sigma_max_ = 1e-6; //!< maximum singular value occuring in SVD composition, limited to 1.
                    //!< The idea is to keep a unitary svd damping value for all tasks by normalizing with the maximum singular value.
                    //!< However, if the task function is small, even the maximum singular values are small => damping too strong
                    //!< value is updated in each iteration
        double k_alpha_dyn_; //!< relative increment that is used to push bounds iteratively away from limits to bring system back to feasible state
        double k_alpha_max_; //!< multiplicative limit of how far the bound is pushed over the deactivation limit

    };


    class JointLimitTask : public Task {
    public:
        JointLimitTask() = default;

        virtual int getTaskIdx() override {
            return task_idx_;
        }

        void initialize(const double &safety, const double &delta, const double &alpha, const StateVector &min,
                        const StateVector &max,
                        const StateVector &gains);

        void update() override;

        void setTaskGain(const double &gain, int idx);


    protected:
        int task_idx_ = TaskKey::joint_limits;
        double delta_; //!< value from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        StateVector alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        StateVector max_;
        StateVector min_;
        StateVector gains_;
    };

    class JointLimitDesiredTask : public JointLimitTask {
    public:
        JointLimitDesiredTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }
    private:
        int task_idx_ = TaskKey::joint_limits_desired;
    };

    class JointCenteringTask : public Task {
    public:
        JointCenteringTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void initialize(const StateVector &q_low, const StateVector &q_up, const StateVector &gains);

        void update() override;

        void setTaskGain(const double &gain, int idx);

    private:
        int task_idx_ = TaskKey::joint_centering;
        Eigen::Matrix<double, Eigen::Dynamic, 1> q_des_;
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> gains_;
        Eigen::Matrix<double, Eigen::Dynamic, n_active_joints> mapping_; //!< selection matrix extracting only the relevant states for jacobian
        std::vector<int> idxs_;
    };

    class VelocityTask : public Task {
    public:
        VelocityTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void setTaskWeightingParam(const double &param, int idx);

        void initialize();

        void update() override;

    private:
        int task_idx_ = TaskKey::joint_velocity;
    };


    class SelfCollisionEllipseTask : public Task {
    public:
        SelfCollisionEllipseTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void
        initialize(const std::vector<int> &idxs, const double &safety, const double &delta, const double &alpha,
                   const double &ax, const double &az,
                   const double &cx, const double &cz, const double &gain);

        void update() override;

        void setTaskGain(const double &gain) {
            gain_ = gain;
        }

    private:
        void findNearestPoint(const double &x, const double &z, double &dist, double &normal_x, double &normal_z);
        int task_idx_ = TaskKey::self_collision_ellipse;
        double gain_;
        std::vector<int> idxs_; //!< indexes for the number of the DH-Link in the chain to be checked for plane collisions
        double safety_; //!< safety distance to actual limits
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        std::vector<double> alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        double ax_;  //!< ellipse main axis scaling along x
        double az_;  //!< ellipse main axis scaling along z
        double cx_;  //!< ellipse center position x in manipulator base link coordinate system
        double cz_;  //!< ellipse center position z in manipulator base link coordinate system
        Eigen::Matrix<double, 3, n_states_uvms> J_link_pos_;
    };

    //! Expects planes bounding a convex self-collision space that should not be entered by the links
    class RestrictingPlaneTask : public Task {
    public:
        RestrictingPlaneTask() = default;

        void
        initialize(const std::vector<int> &idxs, const double &safety, const double &delta, const double &alpha, const Eigen::Vector3d &n,
                   const Eigen::Vector3d &p, const double &gain);
        int getTaskIdx() override {
            return task_idx_;
        }
        void update() override;

        void setTaskGain(const double &gain) {
            gain_ = gain;
        }

    private:
        int task_idx_ = TaskKey::restricting_plane;
        double gain_;
        std::vector<int> idxs_; //!< index for the number of the DH-Link in the chain to be checked for plane collisions
        double safety_; //!< safety distance to actual limits
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        std::vector<double> alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        Eigen::Vector3d n_; //!< normal vector for restricting plane, pointing towards feasible direction
        Eigen::Vector3d p_; //!< point for defining plane
        Eigen::Matrix<double, 3, n_states_uvms> J_link_pos_;
    };

    class AUVAttitudeLimitTask : public Task {
    public:
        AUVAttitudeLimitTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void
        initialize(const std::vector<int> &idxs, const double &delta, const double &alpha,
                   const std::vector<double> &min, const std::vector<double> &max, const std::vector<double> &gains);

        void update() override;

        void setTaskGain(const double &gain, int idx);


    private:
        int task_idx_ = TaskKey::auv_attitude_limits;
        std::vector<int> idxs_;
        std::vector<double> min_;
        std::vector<double> max_;
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        std::vector<double> alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        std::vector<double> gains_;
    };

    class AUVAttitudeInclinationLimitTask : public Task {
    public:
        AUVAttitudeInclinationLimitTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void
        initialize(const double &delta, const double &alpha, const double &max, const double &gain);

        void update() override;

        void setTaskGain(const double &gain);


    private:
        int task_idx_ = TaskKey::auv_attitude_inclination_limits;
        double max_;
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        double alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        double gain_;
    };

    //! expects planes to restrict a convex feasible space that the AUV must stay within
    class AUVConfinedSpaceTask : public Task {
    public:
        AUVConfinedSpaceTask() = default;

        int getTaskIdx() override {
            return task_idx_;
        }

        void
        initialize(const double &safety, const double &delta, const double &alpha,
                   const std::vector<Eigen::Vector3d> &n,
                   const std::vector<Eigen::Vector3d> &p, const double &gain);

        void update() override;

        void setTaskGain(const double &gain) {
            gain_ = gain;
        }

    private:
        int task_idx_ = TaskKey::auv_confined_space;
        double gain_;
        double safety_; //!< safety distance to actual limits
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        std::vector<double> alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
        //std::vector<bool> is_active_;
        std::vector<Eigen::Vector3d> n_; //!< normal vector for restricting plane, pointing towards feasible direction
        std::vector<Eigen::Vector3d> p_; //!< point for defining plane
        Eigen::Matrix<double, 3, n_states_uvms> J_pos_; //!< jacobian for the global position of the auv

    };

    class EndeffectorTrackingTask : public Task {
    public:
        EndeffectorTrackingTask();

        int getTaskIdx() override {
            return task_idx_;
        }

        void addReferences(Eigen::Vector3d *p_eef_des, Eigen::Quaterniond *att_eef_des,
                           Eigen::Matrix<double, 6, 1> *vels_eef_des, Eigen::Matrix<bool, 6, 1> *mask);

        void update() override;

        void setEefGain(const double &gain, int idx);

        void setEefGain(const Eigen::Matrix<double, 6, 1> &gain);

    private:
        int task_idx_ = TaskKey::endeffector_tracking;
        // entities provided via pointer to overlaying kinematic control
        Eigen::Vector3d *p_eef_des_; //!< target end-effector position (world-COS)
        Eigen::Quaterniond *att_eef_des_; //!< target end-effector orientation (world-COS)
        Eigen::Matrix<double, 6, 1> *vels_eef_des_; //!< desired end-effector linear and angular velocity (world-COS)
        Eigen::Matrix<bool, 6, 1> *mask_; //!< mask for ruling out endeffector states not to be tracked
        Eigen::Matrix<double, 6, 6> mapping_; //!< mapping between task jacobian and endeffector kinematic jacobian
        Eigen::DiagonalMatrix<double, 6> K_eef;
        Eigen::Matrix<double, 6, n_states_uvms> J_eef_; //!< current end-effector Jacobian matrix of position and rotation
    };

    //! Manipulability task, heuristic only based on position jacobian matrix of the manipulator. Last continuous joint is
    //! is independent of previous joints, therefore rank loss if position jacobian loses rank
    class ManipulabilityTask : public Task {
    public:
        int getTaskIdx() override {
            return task_idx_;
        }

        ManipulabilityTask() = default;

        void initialize(double &gain);

        void update() override;

        void setTaskGain(const double &gain) {
            gain_ = gain;
        }
        
    private:
        int task_idx_ = TaskKey::manipulability;
        StateVector q_tmp_;
        Eigen::Matrix<double, 3, n_active_joints> J_pos_manipulator_;
        Eigen::Matrix<double, 3, n_active_joints> J_rot_manipulator_;
        std::array<Eigen::Matrix<double, 3, n_active_joints>, n_active_joints> dJ_pos_manipulator_; //!< translational hessian
        Eigen::Matrix<double, 1, n_active_joints> J_manipulability_;
        const double eps_ = 1e-8;
        double gain_;
    };

    //! Manipulability limit task, heuristic only based on position jacobian matrix of the manipulator. Last continuous joint is
    //! is independent of previous joints, therefore rank loss if position jacobian loses rank
    class ManipulabilityLimitTask : public Task {
    public:
        int getTaskIdx() override {
            return task_idx_;
        }

        ManipulabilityLimitTask() = default;

        void initialize(const double &delta, const double &alpha, const double &min, const double &gain);

        void update() override;

        void setTaskGain(const double &gain) {
            gain_ = gain;
        }

    private:
        int task_idx_ = TaskKey::manipulability_limit;
        StateVector q_tmp_;
        Eigen::Matrix<double, 3, n_active_joints> J_pos_manipulator_;
        Eigen::Matrix<double, 3, n_active_joints> J_rot_manipulator_;
        std::array<Eigen::Matrix<double, 3, n_active_joints>, n_active_joints> dJ_pos_manipulator_; //!< translational hessian
        const double eps_ = 1e-8;
        double gain_;
        double min_;  //!< minimum manipulability value
        double delta_; //!< distance from which on the task will be activated
        double alpha_; //!< value up to which the task will be pushed in order to bring it back into the feasible region
        double alpha_dyn_; //!< temporary value that converges to alpha_max_ when the task is activated
    };

    class UVMSKinematicCtrl {
    public:
        UVMSKinematicCtrl();

        void getEefPosition(double &x, double &y, double &z) {
            kinematics->getEefPosition(x, y, z);
        }

        void getEefAttitude(double &w, double &x, double &y, double &z) {
            kinematics->getEefAttitude(w, x, y, z);
        }

        void initialize(const std::vector<param_utils::TFParam> &tf_param, int algorithm_type);

        void addTask(Task *task, const TaskType &taskType);

        Eigen::Vector3d *getPositionTargetReference() {
            return &p_eef_des_;
        }

        Eigen::Quaterniond *getAttitudeTargetReference() {
            return &att_eef_des_;
        }

        Vector6d *getVelocitiesTargetReference() {
            return &vels_eef_des_;
        }

        Eigen::Matrix<bool, 6, 1> *getTrackingMaskReference() {
            return &mask_;
        }

        uvms_kinematics::Kinematics *getKinematicsPtr() {
            return kinematics;
        }

        void setControlTarget(const Eigen::Vector3d &pos_des, const Eigen::Quaterniond &att_des,
                              const Eigen::Vector3d &vel_des, const Eigen::Vector3d &ang_vel_des);

        void setControlTargetForward(const Eigen::Vector3d &pos_des, const Eigen::Quaterniond &att_des,
                              const Eigen::Vector3d &vel_des, const Eigen::Vector3d &ang_vel_des, const double &dt);

        void setTrackingMask(const Eigen::Matrix<bool, 6, 1> &mask);

        void setVelocityLimits(const UVMSStateVector &zeta_min, const UVMSStateVector &zeta_max);

        void setAccelerationSmoothingFactors(double accel_smooth_fac_min, double accel_smooth_fac_max,
                                             double delta_accel_min, double delta_accel_max);

        void setAccelSmoothingFactorMin(const double &factor){
            accel_smooth_fac_min_ = factor;
        }

        void setAccelSmoothingFactorMax(const double &factor){
            accel_smooth_fac_max_ = factor;
        }

        void setEta(double eta){
            std::cout << "Changed eta to: " << eta << std::endl;
            eta_ = eta;
        }

        void setNullSpaceWeighting(bool nullspace_weighting){
            nullspace_weighting_ = nullspace_weighting;
        }

        void setWeightingParam(const double &param, int idx);


        //! Used for only updating the end-effector without calculating control commands
        //! -> as trajectory planner expects states in the beginning
        //! \param pos
        //! \param rot
        //! \param q
        void updateEef(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot, const StateVector &q);

        void getControlCmd(const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, const StateVector &q,
                           UVMSStateVector &cmd_vel);


        void
        getControlCmdWithDerivative(const double &dt, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, const StateVector &q,
                                    UVMSStateVector &cmd_vel, UVMSStateVector &cmd_acc);


    private:
        double getAccelerationSmoothingFactor(const double &delta);

        void updateProjector(Task *task, UVMSStateMatrix &projector, const int &task_dim_counter);

        UVMSStateVector
        getTaskVelocity(int task_idx, const UVMSStateMatrix &projector, const UVMSStateVector &velocity);

        int algorithm_type_;  //!< decides which algorithm type is used for calculating the projectors and velocities
        Eigen::Vector3d p_eef_des_; //!< target end-effector position (world-COS)
        Eigen::Quaterniond att_eef_des_; //!< target end-effector orientation (world-COS)
        Vector6d vels_eef_des_; //!< desired end-effector linear and angular velocity (world-COS)
        Eigen::Vector3d next_p_eef_des_; //!< target end-effector position (world-COS)
        Eigen::Quaterniond next_att_eef_des_; //!< target end-effector orientation (world-COS)
        Vector6d next_vels_eef_des_; //!< desired end-effector linear and angular velocity (world-COS)
        double dt_next_;
        Eigen::Matrix<bool, 6, 1> mask_; //!< mask for tracked states of the endeffector

        double eta_ = 1.0;
        bool nullspace_weighting_ = false;
        uvms_kinematics::Kinematics *kinematics;
        std::vector<Task *> tasks_; //!< tasks assigned for the control algorithm
        std::vector<TaskType> task_types_; //!< decides the kind of a task and therefore how they are composed
        UVMSStateVector zeta_min_; //!< lower bound on velocity states
        UVMSStateVector zeta_max_; //!< upper bound on velocity states
        Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms> J_aug_; //!< transpose of the stacked task jacobians
        UVMSStateMatrix W_sqrt_inv_;  //!< inverse of matrix root of weighting matrix for distributing motion between manipulator and vehicle

        //parameters for acceleration calculation
        UVMSStateVector predicted_state_;
        UVMSStateVector prediction_step_;
        UVMSStateVector last_vel_cmd_;
        // parameters for acceleration smoothing:
        UVMSStateVector last_accel_cmd_;
        bool initialized_last_vel_cmd_ = false;
        bool calculated_first_acceleration_ = false;
        double accel_smooth_fac_min_ = 0.5;
        double accel_smooth_fac_max_ = 0.9;
        double delta_accel_min_ = 0.3;
        double delta_accel_max_ = 1.0;
    };


}

#endif //UVMS_CTRL_KIN_CTRL_HPP
