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

#include "uvms_kinematic_ctrl/kin_ctrl.hpp"


namespace uvms_kin_ctrl {
    void
    checkAlgorithmicSingularity(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> &jacobian,
                                const UVMSStateMatrix &projector, int task_idx, int task_type) {
        Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> svd(jacobian * projector);
        int rank_counter = 0;
        for (const double &singular_value: svd.singularValues()) {
            if (singular_value > 1e-4) {
                rank_counter++;
            }
        }
        if (rank_counter < jacobian.rows() && task_type != TaskType::optimization) {
            std::cerr << "Algorithmic singularity of task " << taskNames.at(task_idx)
                      << " with upstream tasks! Rank is " << rank_counter << " but should be " <<
                      jacobian.rows() << std::endl;
        }
    }

    void Task::updateActiveCounter() {
        active_ = 0;
        for (int i = 0; i < A_.cols(); i++) {
            active_ += int(A_.diagonal()(i) > 0);
        }
    }

    void Task::updateMapping() {
        // assumes that updateActiveCounter was called before
        if (map_.rows() != active_) {
            map_.resize(active_, J_.rows());
        }
        map_.setZero();
        int counter = 0;
        for (int i = 0; i < J_.rows(); i++) {
            if (A_.diagonal()(i) > 0) {
                map_(counter, i) = 1.0;
                counter++;
            }
        }
    }

    bool Task::lowerBoundActivationFunction(const double &state, const double &limit, const double &delta,
                                            const double &alpha, double &activation) {
        if (!is_continuous_) {
            if (activation == 0.0 && state <= limit + delta) { // activate task if limit reached
                activation = 1;
                return true;
            } else if (activation == 1.0 && state <= limit + delta + alpha) {
                activation = 1;
                return true;
            } else {
                activation = 0;
                return false;
            }
        } else {
            activation = lowerBoundSigmoid(state, limit, delta + alpha);
            return activation > 0;
        }
    }

    bool Task::upperBoundActivationFunction(const double &state, const double &limit, const double &delta,
                                            const double &alpha, double &activation) {
        if (!is_continuous_) {
            if (activation == 0.0 && state >= limit - delta) { // activate task if limit reached
                activation = 1;
                return true;
            } else if (activation == 1.0 && state >= limit - delta - alpha) {
                activation = 1;
                return true;
            } else {
                activation = 0;
                return false;
            }
        } else {
            activation = upperBoundSigmoid(state, limit, delta + alpha);
            return activation > 0;
        }
    }

    Eigen::Matrix<double, n_states_uvms, Eigen::Dynamic>
    Task::continuityOperator(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> &X,
                             const UVMSStateMatrix &Q, const UVMSStateMatrix &W_sqrt_inv, const double &eta) {
        UVMSStateMatrix tmp = X.transpose() * P0_sqrt_ * A_ * P0_sqrt_ * X +
                              eta * (UVMSStateMatrix::Identity() - Q).transpose() * (UVMSStateMatrix::Identity() - Q);

        Eigen::JacobiSVD<UVMSStateMatrix> svd_no_weight(tmp, Eigen::ComputeFullV);  // calculate svd without waiting
        // to update maximum singular value and for considering the effect of the weighting onto the singular values
        Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, n_states_uvms>> svd_jacobian(J_, Eigen::ComputeFullV);

        sigma_max_ = std::min(1.0, std::max(sigma_max_, svd_jacobian.singularValues()(0)));

        tmp = W_sqrt_inv * tmp * W_sqrt_inv;  // apply weighting

        Eigen::JacobiSVD<UVMSStateMatrix> svd(tmp, Eigen::ComputeFullV);
        Eigen::DiagonalMatrix<double, n_states_uvms> P;
        P.setZero();
        for (int i = 0; i < svd.singularValues().size(); i++) {
            if (svd.singularValues()(i) < 0.0) {
                std::cerr << "Negative singular value in decomposition" << std::endl;
            }
            if (svd.singularValues()(i) < 1e-12) {
                continue;  // treat numerically as zero
            }
            P.diagonal()(i) = singularValueRegularization(svd.singularValues()(i), svd.singularValues()(i) / std::max(svd_no_weight.singularValues()(i), 1e-10));
        }
        return pseudoInverse(tmp + svd.matrixV() * P * svd.matrixV().transpose()) * W_sqrt_inv * X.transpose() *
               P0_sqrt_ * A_ * A_ * P0_sqrt_;
    }

    double Task::singularValueRegularization(
            const double &sigma, const double &weighting_factor) { // from Namakura, Inverse Kinematic Solutions with Singularity Robustness for Robot manipulators
        if (sigma / sigma_max_ < weighting_factor * sigma_min_) {
            return weighting_factor * sigma_max_ * k_svd_damping_ * std::pow(1 - sigma / (sigma_min_ * weighting_factor * sigma_max_), 2);
        } else {
            return 0;
        }
    }

    void JointLimitTask::initialize(const double &safety, const double &delta, const double &alpha,
                                    const StateVector &min,
                                    const StateVector &max, const StateVector &gains) {
        desired_vel_.setZero();
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_.setConstant(alpha_);
        } else {
            alpha_dyn_.setZero();
        }
        max_ = max - StateVector::Ones() * safety;
        min_ = min + StateVector::Ones() * safety;
        gains_ = gains;
        J_.resize(n_active_joints, n_states_uvms);
        J_.setZero();
        for (int idx = 0; idx < int(n_active_joints); idx++) {
            J_(idx, 6 + idx) = 1.0;
        }
        dx_.resize(n_active_joints);
        dx_.setZero(); // initially not active
        A_.resize(n_active_joints);
        A_.setZero(); // initially not active
        P0_sqrt_.resize(n_active_joints);
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }


    void JointLimitTask::update() {
        for (int idx = 0; idx < int(n_active_joints); idx++) {
            if (upperBoundActivationFunction(model_->q_(idx), max_(idx), delta_, alpha_, A_.diagonal()(idx))) {
                dx_(idx) = gains_(idx) * (max_(idx) - delta_ - alpha_dyn_(idx) - model_->q_(idx));
                alpha_dyn_(idx) = std::min(k_alpha_max_ * alpha_, alpha_dyn_(idx) + k_alpha_dyn_ * alpha_);
            } else if (lowerBoundActivationFunction(model_->q_(idx), min_(idx), delta_, alpha_, A_.diagonal()(idx))) {
                dx_(idx) = gains_(idx) * (min_(idx) + delta_ + alpha_dyn_(idx) - model_->q_(idx));
                alpha_dyn_(idx) = std::min(k_alpha_max_ * alpha_, alpha_dyn_(idx) + k_alpha_dyn_ * alpha_);
            } else {
                dx_(idx) = 0;
                if (is_continuous_) {
                    alpha_dyn_(idx) = alpha_;
                } else {
                    alpha_dyn_(idx) = 0.0;
                }
            }
        }
        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    void JointLimitTask::setTaskGain(const double &gain, int idx) {
        gains_(idx) = gain;
    }

    void JointCenteringTask::initialize(const StateVector &q_low, const StateVector &q_up, const StateVector &gains) {
        desired_vel_.setZero();
        int counter = 0; // counter for joints which are bounded from both sides and are therefore added to the task

        for (int i = 0; i < int(n_active_joints); i++) {
            if (abs(q_low(i)) < 2 * M_PI && abs(q_up(i)) < 2 * M_PI) { // interpret this as limited
                counter += 1;
                idxs_.push_back(i);
            }
        }
        q_des_.resize(counter, 1);
        mapping_.resize(counter, n_active_joints);
        gains_.resize(counter);
        J_.resize(counter, n_states_uvms);
        J_.setZero();
        A_.resize(counter);
        A_.setZero();
        for (int i = 0; i < int(counter); i++) {
            q_des_(i) = 0.5 * (q_up(idxs_[i]) - q_low(idxs_[i]));
            J_(i, idxs_[i] + 6) = 1.0;
            mapping_(i, idxs_[i]) = 1.0;
            A_.diagonal()(i) = 1.0;
            gains_.diagonal()(i) = gains(idxs_[i]);
        }
        P0_sqrt_.resize(counter);
        P0_sqrt_.setIdentity();
        dx_.resize(counter);
        dx_.setZero();
        updateActiveCounter();
        updateMapping();
    }

    void JointCenteringTask::update() {
        updateActiveCounter();
        updateMapping();
        dx_ = gains_ * (q_des_ - mapping_ * model_->q_);
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    void JointCenteringTask::setTaskGain(const double &gain, int idx) {
        for (int i = 0; i < int(idxs_.size()); i++) {
            if (idxs_[i] == idx) {
                gains_.diagonal()(i) = gain;
                return;
            }
        }
        std::cout << "Required idx for joint centering task not limited, gain not set" << std::endl;
    }

    void VelocityTask::initialize() {
        desired_vel_.setZero();
        J_.resize(n_states_uvms, n_states_uvms);
        J_.setIdentity();
        A_.resize(n_states_uvms);
        A_.setIdentity();
        //for (int i = 6; i < int(n_states_uvms); i++){
        //    A_.diagonal()(i) = 0.0;
        //}
        dx_.resize(n_states_uvms);
        dx_.setZero();
        updateActiveCounter();
        updateMapping();
        P0_sqrt_.resize(n_states_uvms);
        P0_sqrt_.setIdentity();
    }

    void VelocityTask::setTaskWeightingParam(const double &param, int idx) {
        if (idx >= P0_sqrt_.cols()) {
            std::cerr << "Index " << idx << " specified for setting weighting matrix entry outside of valid range"
                      << std::endl;
            return;
        }
        P0_sqrt_.diagonal()(idx) = sqrt(param);
        std::cout << "Set weighting parameter " << param << " to " << P0_sqrt_.diagonal()(idx) << std::endl;
    }

    void VelocityTask::update() {}


    void
    SelfCollisionEllipseTask::initialize(const std::vector<int> &idxs, const double &safety, const double &delta,
                                         const double &alpha,
                                         const double &ax, const double &az, const double &cx, const double &cz,
                                         const double &gain) {
        desired_vel_.setZero();
        idxs_ = idxs;
        safety_ = safety;
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_ = std::vector<double>(idxs_.size(), alpha_);
        } else {
            alpha_dyn_ = std::vector<double>(idxs_.size(), 0.0);
        }
        ax_ = ax + safety_;
        az_ = az + safety_;
        cx_ = cx;
        cz_ = cz;
        gain_ = gain;
        J_link_pos_.setZero();
        J_.resize(int(idxs.size()), n_states_uvms);
        J_.setZero();
        dx_.resize(int(idxs.size()));
        dx_.setZero();
        A_.resize(int(idxs.size()));
        A_.setZero();
        P0_sqrt_.resize(int(idxs.size()));
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }

    void SelfCollisionEllipseTask::update() {
        for (int i = 0; i < int(idxs_.size()); i++) {
            model_->getLinkBaseRefPositionJacobian(idxs_[i], J_link_pos_);
            double dist, normal_x, normal_z;
            findNearestPoint(model_->getLinkBaseRefPosition(idxs_[i])(0), model_->getLinkBaseRefPosition(idxs_[i])(2), dist, normal_x, normal_z);
            J_.row(i) = (normal_x * Eigen::Vector3d::UnitX().transpose() + normal_z * Eigen::Vector3d::UnitZ().transpose()) * J_link_pos_;

            if (lowerBoundActivationFunction(dist, 0.0, delta_, alpha_, A_.diagonal()(i))) {
                dx_(i) = gain_ * (delta_ + alpha_dyn_[i] - dist);
                alpha_dyn_[i] = std::min(k_alpha_max_ * alpha_, alpha_dyn_[i] + k_alpha_dyn_ * alpha_);
            } else {
                dx_(i) = 0.0;
                if (is_continuous_) {
                    alpha_dyn_[i] = alpha_;
                } else {
                    alpha_dyn_[i] = 0.0;
                }
                continue;
            }

        }
        updateActiveCounter();
        updateMapping();

        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }


    void SelfCollisionEllipseTask::findNearestPoint(const double &x, const double &z, double &dist, double &normal_x, double &normal_z) {
        //const double ax = ax_ + safety_;
        //const double az = az_ + safety_;
        const double ax = ax_;
        const double az = az_;
        const double x_centered = x - cx_;
        const double z_centered = z - cz_;
        //  polynomial coefficients for x^4 + a*x^3 + b*x^2 + c*x + d
        const double a = 2 * std::pow(ax, 2) + 2 * std::pow(az, 2);
        const double b = std::pow(ax, 4) + 4 * std::pow(ax, 2) * std::pow(az, 2) - std::pow(ax, 2) * std::pow(x_centered, 2) + std::pow(az, 4) - std::pow(az, 2) * std::pow(z_centered, 2);
        const double c = 2 * std::pow(ax, 4) * std::pow(az, 2) + 2 * std::pow(ax, 2) * std::pow(az, 4) - 2 * std::pow(ax, 2) * std::pow(az, 2) * std::pow(x_centered, 2) -
            2 * std::pow(ax, 2) * std::pow(az, 2) * std::pow(z_centered, 2);
        const double d = std::pow(ax, 4) * std::pow(az, 4) - std::pow(ax, 4) * std::pow(az, 2) * std::pow(z_centered, 2) - std::pow(ax, 2) * std::pow(az, 4) * std::pow(x_centered, 2);
        std::vector<double> roots = uvms_common::quartic::solve_quartic(a, b, c, d);
        const double t = *std::max_element(roots.begin(), roots.end());
        double e_x = std::pow(ax, 2) * x_centered / (t + std::pow(ax, 2)) + cx_;
        double e_z = std::pow(az, 2) * z_centered / (t + std::pow(az, 2)) + cz_;
        normal_x = (e_x - cx_) / std::pow(ax, 2);  // calculate gradient
        normal_z = (e_z - cz_) / std::pow(az, 2);
        const double gradient_norm = sqrt(std::pow(normal_x, 2) + std::pow(normal_z, 2));
        dist = t * gradient_norm;
        normal_x /= gradient_norm;
        normal_z /= gradient_norm;
    }


    void
    RestrictingPlaneTask::initialize(const std::vector<int> &idxs, const double &safety, const double &delta, const double &alpha,
                                        const Eigen::Vector3d &n,
                                        const Eigen::Vector3d &p,
                                        const double &gain) {
        desired_vel_.setZero();
        idxs_ = idxs;
        safety_ = safety;
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_ = std::vector<double>(idxs_.size(), alpha_);
        } else {
            alpha_dyn_ = std::vector<double>(idxs_.size(), 0.0);
        }
        n_ = n;
        p_ = p;
        gain_ = gain;
        J_link_pos_.setZero();
        J_.resize(int(idxs.size()), n_states_uvms);
        J_.setZero();
        dx_.resize(int(idxs.size()));
        dx_.setZero();
        A_.resize(int(idxs.size()));
        A_.setZero();
        P0_sqrt_.resize(int(idxs.size()));
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }

    void RestrictingPlaneTask::update() {
        double tmp_dist;
        for (int i = 0; i < int(idxs_.size()); i++) {
            tmp_dist =
                    n_.transpose() * (model_->getLinkBaseRefPosition(idxs_[i]) - p_);
            model_->getLinkBaseRefPositionJacobian(idxs_[i], J_link_pos_);
            J_.row(i) = n_.transpose() * J_link_pos_;

            if (lowerBoundActivationFunction(tmp_dist, safety_, delta_, alpha_, A_.diagonal()(i))) {
                dx_(i) = gain_ * (delta_ + safety_ + alpha_dyn_[i] - tmp_dist);
                alpha_dyn_[i] = std::min(k_alpha_max_ * alpha_, alpha_dyn_[i] + k_alpha_dyn_ * alpha_);
            } else {
                dx_(i) = 0.0;
                if (is_continuous_) {
                    alpha_dyn_[i] = alpha_;
                } else {
                    alpha_dyn_[i] = 0.0;
                }
                continue;
            }
        }
        updateActiveCounter();
        updateMapping();

        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    void
    AUVConfinedSpaceTask::initialize(const double &safety, const double &delta, const double &alpha,
                                     const std::vector<Eigen::Vector3d> &n, const std::vector<Eigen::Vector3d> &p,
                                     const double &gain) {
        active_ = false;
        desired_vel_.setZero();
        safety_ = safety;
        delta_ = delta;
        alpha_ = alpha;
        n_ = n;
        p_ = p;
        if (is_continuous_) {
            alpha_dyn_ = std::vector<double>(n_.size(), alpha_);
        } else {
            alpha_dyn_ = std::vector<double>(n_.size(), 0.0);
        }
        //is_active_ = std::vector<bool>(n_.size(), false);
        gain_ = gain;
        J_pos_.setZero();
        J_.resize(int(n_.size()), n_states_uvms);
        J_.setZero();
        dx_.resize(int(n_.size()));
        dx_.setZero();
        A_.resize(int(n_.size()));
        A_.setZero();
        P0_sqrt_.resize(int(n_.size()));
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }

    void AUVConfinedSpaceTask::update() {
        if (n_.size() != p_.size()) {
            std::cerr << "Plane normals and points don't have the same dimension, abort calculation" << std::endl;
        }
        double tmp_dist;
        J_pos_.block<3, 3>(0, 0) = model_->getAUVAttitudeRotMat();
        for (int i = 0; i < int(n_.size()); i++) {
            J_.row(i) = n_[i].transpose() * J_pos_;
            tmp_dist = n_[i].transpose() * (model_->getAUVPosition() - p_[i]);
            if (lowerBoundActivationFunction(tmp_dist, safety_, delta_, alpha_, A_.diagonal()(i))) {
                dx_(i) = gain_ * (delta_ + safety_ + alpha_dyn_[i] - tmp_dist);
                alpha_dyn_[i] = std::min(k_alpha_max_ * alpha_, alpha_dyn_[i] + k_alpha_dyn_ * alpha_);
            } else {
                dx_(i) = 0.0;
                if (is_continuous_) {
                    alpha_dyn_[i] = alpha_;
                } else {
                    alpha_dyn_[i] = 0.0;
                }
                continue;
            }
        }
        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }


    void AUVAttitudeLimitTask::initialize(const std::vector<int> &idxs, const double &delta, const double &alpha,
                                          const std::vector<double> &min, const std::vector<double> &max,
                                          const std::vector<double> &gains) {
        desired_vel_.setZero();
        idxs_ = idxs;
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_ = std::vector<double>(idxs_.size(), alpha_);
        } else {
            alpha_dyn_ = std::vector<double>(idxs_.size(), 0.0);
        }
        min_ = min;
        max_ = max;
        gains_ = gains;
        J_.resize(int(idxs.size()), n_states_uvms);
        J_.setZero();
        dx_.resize(int(idxs.size()));
        dx_.setZero();
        A_.resize(int(idxs.size()));
        A_.setZero();
        P0_sqrt_.resize(int(idxs.size()));
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }

    void AUVAttitudeLimitTask::update() {
        for (int i = 0; i < int(idxs_.size()); i++) {
            switch (idxs_[i]) {
                case 0:
                    J_.row(i) << 0.0, 0.0, 0.0, 1, sin(model_->auv_att_(0)) * tan(model_->auv_att_(1)),
                            cos(model_->auv_att_(0)) *
                            tan(model_->auv_att_(
                                    1)), 0.0, 0.0, 0.0, 0.0;
                    break;
                case 1:
                    J_.row(i) << 0.0, 0.0, 0.0, 0, cos(model_->auv_att_(0)), -sin(
                            model_->auv_att_(0)), 0.0, 0.0, 0.0, 0.0;
                    break;
                case 2:
                    J_.row(i) << 0.0, 0.0, 0.0, 0, sin(model_->auv_att_(0)) / cos(model_->auv_att_(1)),
                            cos(model_->auv_att_(0)) /
                            cos(model_->auv_att_(
                                    1)), 0.0, 0.0, 0.0, 0.0;
                    break;
                default:
                    std::cerr << "No valid idx chosen, idx is " << idxs_[i] << " but must be between 0 and 2"
                              << std::endl;
                    break;
            }
            if (upperBoundActivationFunction(model_->auv_att_(idxs_[i]), max_[i], delta_, alpha_, A_.diagonal()(i))) {
                dx_(i) = gains_[i] * (max_[i] - delta_ - alpha_dyn_[i] - model_->auv_att_(idxs_[i]));
                alpha_dyn_[i] = std::min(k_alpha_max_ * alpha_, alpha_dyn_[i] + k_alpha_dyn_ * alpha_);
            } else if (lowerBoundActivationFunction(model_->auv_att_(idxs_[i]), min_[i], delta_, alpha_,
                                                    A_.diagonal()(i))) {
                dx_(i) = gains_[i] * (min_[i] + delta_ + alpha_dyn_[i] - model_->auv_att_(idxs_[i]));
                alpha_dyn_[i] = std::min(k_alpha_max_ * alpha_, alpha_dyn_[i] + k_alpha_dyn_ * alpha_);
            } else {
                dx_(i) = 0.0;
                if (is_continuous_) {
                    alpha_dyn_[i] = alpha_;
                } else {
                    alpha_dyn_[i] = 0.0;
                }
            }
        }
        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    void AUVAttitudeLimitTask::setTaskGain(const double &gain, int idx) {
        gains_[idx] = gain;
    }



    void AUVAttitudeInclinationLimitTask::initialize(const double &delta, const double &alpha, const double &max,
                                                     const double &gain) {
        desired_vel_.setZero();
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_ = alpha_;
        } else {
            alpha_dyn_ = 0.0;
        }
        max_ = max;
        gain_ = gain;
        J_.resize(1, n_states_uvms);
        J_.setZero();
        dx_.resize(1);
        dx_.setZero();
        A_.resize(1);
        A_.setZero();
        P0_sqrt_.resize(1);
        P0_sqrt_.setIdentity();
        updateActiveCounter();
        updateMapping();
    }

    void AUVAttitudeInclinationLimitTask::update() {

        Eigen::Matrix3d attitude_error = model_->getAUVAttitudeRotMat(); // R_e = R_des^{-1} R with R_des = I
        double angle = acos(Eigen::Vector3d::UnitZ().transpose() * attitude_error * Eigen::Vector3d::UnitZ());
        Eigen::Vector3d axis;
        if (std::abs(angle) < 1e-6) {
            axis = Eigen::Vector3d::UnitX();
        } else {
            axis = (attitude_error.transpose() * Eigen::Vector3d::UnitZ()).cross(Eigen::Vector3d::UnitZ()) /
                    ((attitude_error.transpose() * Eigen::Vector3d::UnitZ()).cross(Eigen::Vector3d::UnitZ())).norm();
        }

        J_.block<1, 3>(0, 3) = axis.transpose();

        if (upperBoundActivationFunction(angle, max_, delta_, alpha_, A_.diagonal()(0))) {
            dx_(0) = gain_ * (max_ - delta_ - alpha_dyn_ - angle);
            alpha_dyn_ = std::min(k_alpha_max_ * alpha_, alpha_dyn_ + k_alpha_dyn_ * alpha_);
        } else {
            dx_(0) = 0.0;
            if (is_continuous_) {
                alpha_dyn_ = alpha_;
            } else {
                alpha_dyn_ = 0.0;
            }
        }
        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    void AUVAttitudeInclinationLimitTask::setTaskGain(const double &gain) {
        gain_ = gain;
    }

    EndeffectorTrackingTask::EndeffectorTrackingTask() {
        desired_vel_.setZero();
        p_eef_des_ = nullptr;
        att_eef_des_ = nullptr;
        vels_eef_des_ = nullptr;
        mapping_.setIdentity();
        A_.resize(6);
        A_.setIdentity();
        P0_sqrt_.resize(6);
        P0_sqrt_.setIdentity();
        J_.resize(6, n_states_uvms);
        J_.setZero();
        dx_.resize(6);
        dx_.setZero();
        updateActiveCounter();
        updateMapping();
    }

    void EndeffectorTrackingTask::addReferences(Eigen::Vector3d *p_eef_des, Eigen::Quaterniond *att_eef_des,
                                                Eigen::Matrix<double, 6, 1> *vels_eef_des,
                                                Eigen::Matrix<bool, 6, 1> *mask) {
        p_eef_des_ = p_eef_des;
        att_eef_des_ = att_eef_des;
        vels_eef_des_ = vels_eef_des;
        mask_ = mask;
    }

    void EndeffectorTrackingTask::update() {
        model_->getEefJacobian(J_eef_);
        Eigen::Matrix<double, 6, 1> pose_error;
        pose_error.segment<3>(0) = *p_eef_des_ - model_->getEefPosition();
        uvms_kinematics::quaternionError(*att_eef_des_, model_->getEefAttitude(), pose_error.segment<3>(3));
        mapping_ = Eigen::Matrix<double, 6, 6>::Identity();
        for (int i = 0; i < 6; i++) {
            A_.diagonal()(i) = int((*mask_)(i));
        }
        mapping_.block<3, 3>(3, 3) = mapping_.block<3, 3>(3, 3) * model_->getEefAttitude().toRotationMatrix().inverse();
        //mapping_.block<3, 3>(3, 3) = mapping_.block<3, 3>(3, 3) * (*att_eef_des_).toRotationMatrix().inverse();
        J_ = mapping_ * J_eef_;

        dx_ = mapping_ * (*vels_eef_des_ + K_eef * pose_error);

        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
        // on the assumption that J is full rank
    }

    void EndeffectorTrackingTask::setEefGain(const double &gain, int idx) {
        if (idx >= K_eef.cols()) {
            std::cerr << "Index " << idx << " specified for setting gain value outside of valid range" << std::endl;
            return;
        }
        K_eef.diagonal()(idx) = gain;
    }

    void EndeffectorTrackingTask::setEefGain(const Eigen::Matrix<double, 6, 1> &gain) {
        K_eef.diagonal() = gain;
    }

    void ManipulabilityTask::initialize(double &gain) {
        desired_vel_.setZero();
        J_.resize(n_active_joints, n_states_uvms);
        for (int i = 0; i < int(n_active_joints); i++) {
            J_(i, i + 6) = 1.0;
        }
        A_.resize(n_active_joints);
        A_.setIdentity();
        P0_sqrt_.resize(n_active_joints);
        P0_sqrt_.setIdentity();
        gain_ = gain;
        dx_.resize(n_active_joints);
        dx_.setZero();
        updateActiveCounter();
        updateMapping();
    }

    void ManipulabilityTask::update() {
        double manipulability;
        J_manipulability_.setZero();
        model_->manipulator_kin_->getPositionJacobian(J_pos_manipulator_);

        manipulability = sqrt((J_pos_manipulator_ * J_pos_manipulator_.transpose()).determinant());

        model_->manipulator_kin_->getPositionJacobianHessian(J_pos_manipulator_, J_rot_manipulator_, dJ_pos_manipulator_);
        J_manipulability_.setZero();
        for (int i = 0; i < int(n_active_joints); i++){
            J_manipulability_(0,i + 6) = manipulability * (dJ_pos_manipulator_[i] * pseudoInverse(J_pos_manipulator_)).trace();
        }

        dx_ = gain_ * J_manipulability_.transpose();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }

    }


    void ManipulabilityLimitTask::initialize(const double &delta, const double &alpha, const double &min,
                                             const double &gain) {
        desired_vel_.setZero();
        J_.resize(1, n_states_uvms);
        J_.setZero();
        A_.resize(1);
        A_.setZero();
        P0_sqrt_.resize(1);
        P0_sqrt_.setIdentity();
        delta_ = delta;
        alpha_ = alpha;
        if (is_continuous_) {
            alpha_dyn_ = alpha_;
        } else {
            alpha_dyn_ = 0.0;
        }
        min_ = min;
        gain_ = gain;
        dx_.resize(1);
        dx_.setZero();
        updateActiveCounter();
        updateMapping();
    }

    void ManipulabilityLimitTask::update() {
        double manipulability;
        model_->manipulator_kin_->getPositionJacobian(J_pos_manipulator_);
        model_->manipulator_kin_->getRotationJacobian(J_rot_manipulator_);

        manipulability = sqrt((J_pos_manipulator_ * J_pos_manipulator_.transpose()).determinant());
        if (lowerBoundActivationFunction(manipulability, min_, delta_, alpha_,
                                         A_.diagonal()(0))) {
            dx_(0) = gain_ * (min_ + delta_ + alpha_dyn_ - manipulability);
            alpha_dyn_ = std::min(k_alpha_max_ * alpha_, alpha_dyn_ + k_alpha_dyn_ * alpha_);
        } else {
            dx_(0) = 0.0;
            if (is_continuous_) {
                alpha_dyn_ = alpha_;
            } else {
                alpha_dyn_ = 0.0;
            }
        }

        model_->manipulator_kin_->getPositionJacobianHessian(J_pos_manipulator_, J_rot_manipulator_, dJ_pos_manipulator_);
        J_.setZero();
        for (int i = 0; i < int(n_active_joints); i++){
            J_(0,i + 6) = manipulability * (dJ_pos_manipulator_[i] * pseudoInverse(J_pos_manipulator_)).trace();
        }

        updateActiveCounter();
        updateMapping();
        if (active_ > 0) {
            desired_vel_ = pseudoInverse(getActiveJacobian()) * getActiveDesiredTaskVelocity();
        } else {
            desired_vel_.setZero();
        }
    }

    UVMSKinematicCtrl::UVMSKinematicCtrl() {

    }

    void
    UVMSKinematicCtrl::initialize(const std::vector<param_utils::TFParam> &tf_param, int algorithm_type) {
        kinematics = new uvms_kinematics::Kinematics(tf_param);
        algorithm_type_ = algorithm_type;
        last_accel_cmd_.setZero();
        last_vel_cmd_.setZero();
    }

    void UVMSKinematicCtrl::addTask(Task *task, const TaskType &taskType) {
        task->addReferenceModel(kinematics);
        tasks_.push_back(task);
        task_types_.push_back(taskType);
        if (taskType != TaskType::optimization) {
            J_aug_.resize(J_aug_.rows() + tasks_.back()->getFullTaskDimension(), n_states_uvms);
        }
    }

    void UVMSKinematicCtrl::updateEef(const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, const StateVector &q) {
        kinematics->update(q, pos, att);
    }

    void UVMSKinematicCtrl::setControlTarget(const Eigen::Vector3d &pos_des, const Eigen::Quaterniond &att_des,
                                             const Eigen::Vector3d &vel_des, const Eigen::Vector3d &ang_vel_des) {
        p_eef_des_ = pos_des;
        att_eef_des_ = att_des;
        vels_eef_des_.segment<3>(0) = vel_des;
        vels_eef_des_.segment<3>(3) = ang_vel_des;
    }

    void UVMSKinematicCtrl::setControlTargetForward(const Eigen::Vector3d &pos_des, const Eigen::Quaterniond &att_des,
                                             const Eigen::Vector3d &vel_des, const Eigen::Vector3d &ang_vel_des, const double &dt) {
        next_p_eef_des_ = pos_des;
        next_att_eef_des_ = att_des;
        next_vels_eef_des_.segment<3>(0) = vel_des;
        next_vels_eef_des_.segment<3>(3) = ang_vel_des;
        dt_next_ = dt;
    }

    void UVMSKinematicCtrl::setTrackingMask(const Eigen::Matrix<bool, 6, 1> &mask) {
        mask_ = mask;
    }

    void UVMSKinematicCtrl::setVelocityLimits(const UVMSStateVector &zeta_min, const UVMSStateVector &zeta_max) {
        zeta_min_ = zeta_min;
        zeta_max_ = zeta_max;
    }

    void UVMSKinematicCtrl::setWeightingParam(const double &param, int idx) {
        if (idx >= W_sqrt_inv_.cols()) {
            std::cerr << "Index " << idx << " specified for setting weighting matrix entry outside of valid range"
                      << std::endl;
            return;
        }
        W_sqrt_inv_.diagonal()(idx) = 1 / sqrt(param);
        std::cout << "Set weighting parameter " << param << " to " << W_sqrt_inv_.diagonal()(idx) << std::endl;
        for (Task *task: tasks_) {
            if (task->getTaskIdx() == TaskKey::joint_velocity) {
                if (nullspace_weighting_) {
                    dynamic_cast<VelocityTask *>(task)->setTaskWeightingParam(1.0, idx);
                } else {
                    dynamic_cast<VelocityTask *>(task)->setTaskWeightingParam(param, idx);
                }
            }
        }
    }

    void
    UVMSKinematicCtrl::getControlCmd(const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, const StateVector &q,
                                     UVMSStateVector &cmd_vel) {
        kinematics->update(q, pos, att);

        UVMSStateVector zeta_low, zeta_high, task_cmd_vel;
        double scaling, scaling_tmp;
        zeta_low = zeta_min_;
        zeta_high = zeta_max_;
        cmd_vel.setZero();
        UVMSStateMatrix projector;
        if (nullspace_weighting_) {
            projector = W_sqrt_inv_;
        } else {
            projector = UVMSStateMatrix::Identity();
        }
        //projector.diagonal() = UVMSStateVector(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 20, 20, 20, 20);
        int task_dim_counter = 0;
        int prev_rank = 0;
        int rank = 0;

        for (int task_idx = 0; task_idx < int(tasks_.size()); task_idx++) {
            tasks_[task_idx]->update();
            if (!tasks_[task_idx]->isActive()) {
                continue;
            }


            // if normal task, get velocity and proceed, for optimization tasks the objectives are merged
            if (task_types_[task_idx] == TaskType::optimization && task_idx < 1) {
                std::cerr << "Only optimization task chosen, there should be preceding tasks..." << std::endl;
            }


            if (task_types_[task_idx] == TaskType::optimization && task_idx > 0 &&
                task_types_[task_idx - 1] != TaskType::optimization) { // first optimization task

                /*
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W = tasks_[task_idx]->getFullJacobian() * projector *
                                                                                tasks_[task_idx]->continuityOperator(
                                                                                        tasks_[task_idx]->getFullJacobian() *
                                                                                        projector, projector,
                                                                                        W_sqrt_inv_, eta_);
                UVMSStateMatrix task_weighting;
                task_weighting.setZero();
                task_weighting.diagonal() << 100, 100, 100, 100, 100, 100, 1.0, 1.0, 1.0, 1.0;
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_weight = tasks_[task_idx]->getFullJacobian() * projector *
                                                                                tasks_[task_idx]->continuityOperatorTest(
                                                                                        tasks_[task_idx]->getFullJacobian() *
                                                                                        projector, projector,
                                                                                        W_sqrt_inv_, eta_, task_weighting);
                std::cout << "Weighting matrix without : \n" << W << std::endl;
                std::cout << "Weighting matrix with : \n" << W_weight << std::endl;
                //std::cout << "Jacobian: \n" << tasks_[task_idx]->getFullJacobian() << std::endl;
                //UVMSStateMatrix is_this_identity = projector * pseudoInverse(projector.transpose() * projector, weighting) * projector.transpose();
                //UVMSStateMatrix is_this_identity2 = projector.transpose() * pseudoInverse(projector * projector.transpose(), weighting) * projector;
                        //tasks_[task_idx]->continuityOperator(tasks_[task_idx]->getFullJacobian() * projector, UVMSStateMatrix::Identity(),
                        //                                     weighting);
                //std::cout << "Test matrix: \n" << is_this_identity << std::endl;
                //std::cout << "Test matrix2: \n" << is_this_identity2 << std::endl;
                UVMSStateVector  velocity =  projector *
                                                    W_sqrt_inv_ * tasks_[task_idx]->continuityOperator(tasks_[task_idx]->getFullJacobian() * projector,
                                                                                                     UVMSStateMatrix::Identity(),
                                                                                                     W_sqrt_inv_, eta_) * W *
                                                    (tasks_[task_idx]->getFullDesiredTaskVelocity() -
                                                     tasks_[task_idx]->getFullJacobian() * cmd_vel);
                UVMSStateVector  velocity_weighted =  projector *
                                                    W_sqrt_inv_ * tasks_[task_idx]->continuityOperatorTest(tasks_[task_idx]->getFullJacobian() * projector,
                                                                                                     UVMSStateMatrix::Identity(),
                                                                                                     W_sqrt_inv_, eta_, task_weighting) * W_weight *
                                                    (tasks_[task_idx]->getFullDesiredTaskVelocity() -
                                                     tasks_[task_idx]->getFullJacobian() * cmd_vel);
                std::cout << "Velocity wihtout : \n " << velocity << std::endl;
                std::cout << "Velocity with : \n " << velocity_weighted << std::endl;


                std::cout << "projector: \n " << projector << std::endl;
                std::cout << "jacobian: \n " << tasks_[task_idx]->getFullJacobian() << std::endl;
                UVMSStateMatrix projection_without = projector *
                                                     tasks_[task_idx]->continuityOperator(tasks_[task_idx]->getFullJacobian() * projector, UVMSStateMatrix::Identity(),
                                                                                          UVMSStateMatrix::Identity());
                UVMSStateMatrix projection_with = projector *
                                                     tasks_[task_idx]->continuityOperator(tasks_[task_idx]->getFullJacobian() * projector, UVMSStateMatrix::Identity(),
                                                                                          weighting);
                std::cout << "projection without: \n " << projection_without << std::endl;
                std::cout << "projection with: \n " << projection_with << std::endl;
                std::cout << "projection weighting without: \n " << projection_without * W << std::endl;
                std::cout << "projection weighting with: \n " << projection_with * W_weight<< std::endl;
                 */


                // ---------------------------------------------------
                task_cmd_vel = getTaskVelocity(task_idx, projector, cmd_vel);
                tasks_[task_idx]->setProjectedVelocities(task_cmd_vel);
            } else if (task_types_[task_idx] == TaskType::optimization && task_idx > 0 &&
                       task_types_[task_idx - 1] == TaskType::optimization) {  // later of multiple optimization tasks
                UVMSStateVector tmp_task_cmd_vel = getTaskVelocity(task_idx, projector, cmd_vel);
                task_cmd_vel += tmp_task_cmd_vel;
                tasks_[task_idx]->setProjectedVelocities(tmp_task_cmd_vel);
            } else {
                task_cmd_vel = getTaskVelocity(task_idx, projector, cmd_vel);
            }


            if (task_types_[task_idx] == TaskType::optimization && task_idx < int(tasks_.size()) - 1) {

                if (task_types_[task_idx + 1] == TaskType::optimization && tasks_[task_idx + 1]->isActive()) {
                    continue; // add next command velocity to current velocity
                } else {
                    std::cerr
                            << "No other task types should be preceding after an optimization task and should be active"
                            << std::endl;
                }
            }

            // velocity scaling:
            scaling = 1.0;
            for (int i = 0; i < int(n_states_uvms); i++) {
                if (task_cmd_vel(i) > zeta_high(i)) {
                    scaling_tmp = zeta_high(i) / task_cmd_vel(i);
                    if ((scaling_tmp < 0) || (scaling_tmp > 1.0)) {
                        scaling = 0.0;
                        break;
                    }
                    scaling = std::min(scaling_tmp, scaling);
                } else if (task_cmd_vel(i) < zeta_low(i)) {
                    scaling_tmp = zeta_low(i) / task_cmd_vel(i);
                    if ((scaling_tmp < 0) || (scaling_tmp > 1.0)) {
                        scaling = 0.0;
                        break;
                    }
                    scaling = std::min(scaling_tmp, scaling);
                }
            }
            zeta_low = zeta_low - scaling * task_cmd_vel;
            zeta_high = zeta_high - scaling * task_cmd_vel;
            cmd_vel += scaling * task_cmd_vel;


            if (task_types_[task_idx] == TaskType::optimization) {
                // previously projected velocities with scaling were temporarily stored in projected velocity attribute of all optimization tasks
                int idx = task_idx;
                while (task_types_[idx] == TaskType::optimization && idx > 0) {
                    tasks_[idx]->setProjectedVelocities(scaling * tasks_[idx]->getProjectedVelocities());
                    idx--;
                }
                continue;  // no successive projection, therefore projection is not relevant
            }

            // for debugging and publishing access:
            tasks_[task_idx]->setProjectedVelocities(scaling * task_cmd_vel);


            // check rank of task
            if (tasks_[task_idx]->getActiveJacobianRank() != tasks_[task_idx]->countActive()) {
                std::cerr << "Task rank with idx " << task_idx << " is rank deficient!" << std::endl;
            }

            tasks_[task_idx]->getActiveJacobian(
                    J_aug_.block(task_dim_counter, 0, tasks_[task_idx]->getActiveTaskDimension(), n_states_uvms));
            task_dim_counter += tasks_[task_idx]->getActiveTaskDimension();

            // check how the current task jacobian is assembled with the one of the preceding tasks:
            if (task_types_[task_idx] == TaskType::critical || task_types_[task_idx] == TaskType::high_priority) {
                rank = int(J_aug_.block(0, 0, task_dim_counter, n_states_uvms).colPivHouseholderQr().rank());
                if (task_dim_counter > 0) {
                    if (prev_rank + tasks_[task_idx]->getActiveJacobianRank() != rank) {
                        if (task_types_[task_idx] == TaskType::critical) {
                            std::cerr
                                    << "Critical task is not independent of upstream tasks! Fulfilling the task cannot be guaranteed!"
                                    << std::endl;
                        } else if (task_types_[task_idx] == TaskType::high_priority) {
                            std::cout
                                    << "High priority task not independent of upstream tasks, convergence not guaranteed"
                                    << std::endl;
                        }
                    }
                }
                prev_rank = rank;
            }

            // update projector
            updateProjector(tasks_[task_idx], projector, task_dim_counter);
        }
    }


    void UVMSKinematicCtrl::setAccelerationSmoothingFactors(double accel_smooth_fac_min, double accel_smooth_fac_max,
                                                            double delta_accel_min, double delta_accel_max) {
        accel_smooth_fac_min_ = accel_smooth_fac_min;
        accel_smooth_fac_max_ = accel_smooth_fac_max;
        delta_accel_min_ = delta_accel_min;
        delta_accel_max_ = delta_accel_max;
    }

    double UVMSKinematicCtrl::getAccelerationSmoothingFactor(const double &delta) {
        double factor = accel_smooth_fac_max_ * (1 + (accel_smooth_fac_max_ - accel_smooth_fac_min_) / accel_smooth_fac_min_ *
                                                     std::min((std::abs(delta) - delta_accel_min_) /
                                                              (delta_accel_max_ - delta_accel_min_), 1.0)) *
                        accel_smooth_fac_min_ / accel_smooth_fac_max_ * int(std::abs(delta) > delta_accel_min_);
        return factor;

        //return accel_smooth_fac_min_;
    }

    void
    UVMSKinematicCtrl::getControlCmdWithDerivative(const double &dt, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att,
                                                   const StateVector &q,
                                                   UVMSStateVector &cmd_vel, UVMSStateVector &cmd_acc) {
        //auto start_time = std::chrono::high_resolution_clock::now();
        getControlCmd(pos, att, q, cmd_vel);  // calculate control command based on current states

        // calculate prediction based on perfectly tracked velocity command for next step

        if (!initialized_last_vel_cmd_){
            last_vel_cmd_ = cmd_vel;
            initialized_last_vel_cmd_ = true;
            cmd_acc.setZero();
            return;
        }

        /*
        // prediction
        UVMSStateMatrix jacobian;
        kinematics->getJacobian(q, pos, att, jacobian);
        prediction_step_ = jacobian * cmd_vel;
        UVMSStateVector state;
        state.segment<3>(0) = pos;  // store current state for calculation of predicted state in next step
        rot_utils::quat_to_rpy(att, state.segment<3>(3));
        state.segment<n_active_joints>(6) = q;
        predicted_state_ = state + dt_next_ * prediction_step_;
        Eigen::Quaterniond predicted_att;
        rot_utils::rpy_to_quat(predicted_state_(3), predicted_state_(4), predicted_state_(5), predicted_att);
        setControlTarget(next_p_eef_des_, next_att_eef_des_, next_vels_eef_des_.segment<3>(0), next_vels_eef_des_.segment<3>(3)); // set most recent control target which is stored as last target again as recent one

        UVMSStateVector cmd_vel_predicted;
        getControlCmd(predicted_state_.segment<3>(0), predicted_att, predicted_state_.segment<n_active_joints>(6), cmd_vel_predicted);  // calculate control command based on current states


        cmd_acc = (cmd_vel_predicted - cmd_vel) /
                  dt_next_; // calculate acceleration based on perfectly tracked velocity propagation
        cmd_acc.segment<3>(0) += cmd_vel.segment<3>(3).cross(cmd_vel.segment<3>(0));
        last_accel_cmd_ = cmd_acc;
        if (!calculated_first_acceleration_) {
            calculated_first_acceleration_ = true;

        } else {
            for (int i = 0; i < int(n_states_uvms); i++) {
                cmd_acc(i) = (1 - getAccelerationSmoothingFactor(cmd_acc(i) - last_accel_cmd_(i))) *
                             (cmd_acc(i) - last_accel_cmd_(i)) + last_accel_cmd_(i);
            }
        }

        last_vel_cmd_ = cmd_vel;
        */
        // direct derivative from command lines

        cmd_acc = (cmd_vel - last_vel_cmd_) /
                  dt; // calculate acceleration based on perfectly tracked velocity propagation
        cmd_acc.segment<3>(0) += cmd_vel.segment<3>(3).cross(cmd_vel.segment<3>(0));
        if (!calculated_first_acceleration_) {
            calculated_first_acceleration_ = true;

        } else {
            for (int i = 0; i < int(n_states_uvms); i++) {
                cmd_acc(i) = (1 - getAccelerationSmoothingFactor(cmd_acc(i) - last_accel_cmd_(i))) *
                             (cmd_acc(i) - last_accel_cmd_(i)) + last_accel_cmd_(i);
            }
        }
        last_accel_cmd_ = cmd_acc;
        last_vel_cmd_ = cmd_vel;

        /*
        last_vel_cmd_ = cmd_vel;
        dt_last_accel_update_ = 0.0;
        updated_setpoint_ = false;
         */
    }

    void UVMSKinematicCtrl::updateProjector(Task *task, UVMSStateMatrix &projector, const int &task_dim_counter) {
        switch (algorithm_type_) {
            case AlgorithmType::siciliano:
                projector = UVMSStateMatrix::Identity() -
                            W_sqrt_inv_ *
                            pseudoInverse(J_aug_.block(0, 0, task_dim_counter, n_states_uvms) * W_sqrt_inv_) *
                            J_aug_.block(0, 0, task_dim_counter, n_states_uvms);  // checked
                break;
            case AlgorithmType::nakamura:
                projector = projector * (UVMSStateMatrix::Identity() -
                                         W_sqrt_inv_ *
                                         pseudoInverse(task->getActiveJacobian() * projector * W_sqrt_inv_) *
                                         task->getActiveJacobian() * projector);  // checked for two tasks
                break;

            case AlgorithmType::singularity_robust:
                projector = projector * (UVMSStateMatrix::Identity() -
                                         W_sqrt_inv_ * pseudoInverse(task->getActiveJacobian() * W_sqrt_inv_) *
                                         task->getActiveJacobian());
                break;
            case AlgorithmType::singularity_robust_augmented:
                projector = UVMSStateMatrix::Identity() -
                            W_sqrt_inv_ *
                            pseudoInverse(J_aug_.block(0, 0, task_dim_counter, n_states_uvms) * W_sqrt_inv_) *
                            J_aug_.block(0, 0, task_dim_counter, n_states_uvms);
                break;
            case AlgorithmType::continuous:
                if (nullspace_weighting_) {
                    // weighting included as projector initialized with weighting matrix, don't apply further weighting
                    projector = projector * (UVMSStateMatrix::Identity() -
                                             task->continuityOperator(task->getFullJacobian() * projector,
                                                                      UVMSStateMatrix::Identity(),
                                                                      UVMSStateMatrix::Identity(), eta_) *
                                             task->getFullJacobian() * projector);
                } else {
                    // consider only changed direction in the projection due to the weighting, but don't weight downstream tasks
                    projector = projector * (UVMSStateMatrix::Identity() -
                                             W_sqrt_inv_ * task->continuityOperator(task->getFullJacobian() * projector,
                                                                                    UVMSStateMatrix::Identity(),
                                                                                    W_sqrt_inv_, eta_) *
                                             task->getFullJacobian() * projector);
                }
        }

    }

    UVMSStateVector
    UVMSKinematicCtrl::getTaskVelocity(int task_idx, const UVMSStateMatrix &projector,
                                       const UVMSStateVector &velocity) {
        switch (algorithm_type_) {
            case AlgorithmType::siciliano:
                checkAlgorithmicSingularity(tasks_[task_idx]->getActiveJacobian(), projector,
                                            tasks_[task_idx]->getTaskIdx(), task_types_[task_idx]);
                return W_sqrt_inv_ * pseudoInverse(tasks_[task_idx]->getActiveJacobian() * projector * W_sqrt_inv_) *
                       (tasks_[task_idx]->getActiveDesiredTaskVelocity() -
                        tasks_[task_idx]->getActiveJacobian() * velocity);  // checked
            case AlgorithmType::nakamura:
                checkAlgorithmicSingularity(tasks_[task_idx]->getActiveJacobian(), projector,
                                            tasks_[task_idx]->getTaskIdx(), task_types_[task_idx]);
                return projector * W_sqrt_inv_ *
                       pseudoInverse(tasks_[task_idx]->getActiveJacobian() * projector * W_sqrt_inv_) *
                       (tasks_[task_idx]->getActiveDesiredTaskVelocity() -
                        tasks_[task_idx]->getActiveJacobian() * velocity);  // rechecked
            case AlgorithmType::singularity_robust:
                return projector * W_sqrt_inv_ * pseudoInverse(tasks_[task_idx]->getActiveJacobian() * W_sqrt_inv_) *
                       tasks_[task_idx]->getActiveDesiredTaskVelocity();

            case AlgorithmType::singularity_robust_augmented:
                return projector * W_sqrt_inv_ * pseudoInverse(tasks_[task_idx]->getActiveJacobian() * W_sqrt_inv_) *
                       tasks_[task_idx]->getActiveDesiredTaskVelocity();
            case AlgorithmType::continuous: {
                checkAlgorithmicSingularity(tasks_[task_idx]->getActiveJacobian(), projector,
                                            tasks_[task_idx]->getTaskIdx(), task_types_[task_idx]);
                UVMSStateMatrix weighting;
                if (nullspace_weighting_) {
                    // weighting included as projector initialized with weighting matrix, don't apply further weighting
                    weighting.setIdentity();
                } else {
                    weighting = W_sqrt_inv_;
                }
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W =
                        tasks_[task_idx]->getFullJacobian() * projector *
                        weighting * tasks_[task_idx]->continuityOperator(
                                tasks_[task_idx]->getFullJacobian() *
                                projector, projector,
                                weighting, eta_);
                return projector *
                       weighting * tasks_[task_idx]->continuityOperator(tasks_[task_idx]->getFullJacobian() * projector,
                                                                        UVMSStateMatrix::Identity(),
                                                                        weighting, eta_) * W *
                       (tasks_[task_idx]->getFullDesiredTaskVelocity() -
                        tasks_[task_idx]->getFullJacobian() * velocity);
            }
            default:
                return projector * tasks_[task_idx]->getDesiredVelocities();

        }
    }

}


