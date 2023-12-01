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

#include "bluerov_ctrl/twist_model_based.hpp"

namespace bluerov_ctrl {

    Eigen::Matrix3d skew(const Eigen::Ref<const Eigen::Vector3d> &x) {
        Eigen::Matrix3d x_tilde;
        x_tilde << 0, -x(2), x(1),
                x(2), 0, -x(0),
                -x(1), x(0), 0;
        return x_tilde;
    }


    TwistModelBasedControl::TwistModelBasedControl(Matrix6d M, Vector6d damping_linear, Vector6d damping_nonlinear,
                                                   Eigen::Vector3d cog, Eigen::Vector3d cob, double mass,
                                                   double buoyancy, bool body_fixed):
                                                   M_(M), damping_linear_(damping_linear), damping_nonlinear_(damping_nonlinear),
                                                   mass_(mass), buoyancy_(buoyancy), cog_(cog), cob_(cob){
        body_fixed_ = body_fixed;
    }

    //! expects velocity target in body frame
    void TwistModelBasedControl::setVelocityTarget(const double &lin_x, const double &lin_y, const double &lin_z, const double &ang_x,
                           const double &ang_y, const double &ang_z) {
        nu_des_(0) = lin_x;
        nu_des_(1) = lin_y;
        nu_des_(2) = lin_z;
        nu_des_(3) = ang_x;
        nu_des_(4) = ang_y;
        nu_des_(5) = ang_z;
        set_velocity_target_ = true;
    }

    //! expects absolute acceleration target in body frame
    void TwistModelBasedControl::setAccelerationTarget(const double &lin_x, const double &lin_y, const double &lin_z, const double &ang_x,
                               const double &ang_y, const double &ang_z) {
        if (!set_velocity_target_){
            std::cerr << "Velocity target must be set first, as it is required for the calculation of the derivative" << std::endl;
        }
        dnu_des_(0) = lin_x;
        dnu_des_(1) = lin_y;
        dnu_des_(2) = lin_z;
        dnu_des_(3) = ang_x;
        dnu_des_(4) = ang_y;
        dnu_des_(5) = ang_z;

        // as we need derivative of linear velocity in body frame and not in inertial frame:
        dnu_des_.segment<3>(0) = dnu_des_.segment<3>(0) - nu_des_.segment<3>(3).cross(nu_des_.segment<3>(0));
    }

    void TwistModelBasedControl::update(const double &dt, const Eigen::Quaterniond &att, const Eigen::Vector3d &vel,
                                        const Eigen::Vector3d &acc,
                                        const Eigen::Vector3d &ang_vel,
                                        const Eigen::Vector3d &ang_acc, Vector6d &out) {
        Eigen::Matrix<double, 6, 1> nu, dnu;
        nu.segment<3>(0) = vel;
        nu.segment<3>(3) = ang_vel;
        dnu.segment<3>(0) =
                acc - ang_vel.cross(vel);// calculation of linear velocity time derivative in body coordinate system
        dnu.segment<3>(3) = ang_acc;

        // calculate gravity force vector and buoyancy
        const Eigen::Vector3d gravity_force =
                mass_ * att.toRotationMatrix().inverse() * Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
        const Eigen::Vector3d buoyancy_force =
                - buoyancy_ * att.toRotationMatrix().inverse() * Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
        g_.segment<3>(0) = -(gravity_force + buoyancy_force); // as included on the left side of the equations, the entities must be negative
        g_.segment<3>(3) = -(cog_.cross(gravity_force) + cob_.cross(buoyancy_force));// as included on the left side of the equations, the entities must be negative

        // update Coriolis matrix
        C_.block<3, 3>(0, 0).setZero();
        C_.block<3, 3>(0, 3) = -skew(M_.block<3, 3>(0, 0) * vel + M_.block<3, 3>(0, 3) * ang_vel);
        C_.block<3, 3>(3, 0) = -skew(M_.block<3, 3>(0, 0) * vel + M_.block<3, 3>(0, 3) * ang_vel);
        C_.block<3, 3>(3, 3) = -skew(M_.block<3, 3>(3, 0) * vel + M_.block<3, 3>(3, 3) * ang_vel);

        // update damping matrix
        D_.setZero();
        D_.diagonal().segment<3>(0) =
                damping_linear_.segment<3>(0) + (damping_nonlinear_.segment<3>(0).array() * vel.array().abs()).matrix();
        D_.diagonal().segment<3>(3) =
                damping_linear_.segment<3>(3) + (damping_nonlinear_.segment<3>(3).array() * vel.array().abs()).matrix();


        // body fixed controller
        if (body_fixed_) {
            error_integral_ += dt * (nu_des_ - nu);
        } else {
            error_integral_.segment<3>(0) += dt * att.toRotationMatrix() * (nu_des_.segment<3>(0) - nu.segment<3>(0));
            error_integral_.segment<3>(3) += dt * att.toRotationMatrix() * (nu_des_.segment<3>(3) - nu.segment<3>(3));
        }
        for (int i = 0; i < error_integral_.cols(); i++){ // reset errors if gain is set to zero
            if (i_gains_.diagonal()(i) == 0){
                error_integral_(i) = 0.0;
            }
        }

        p_component_ = M_ * p_gains_ * (nu_des_ - nu);
        d_component_ = M_ * d_gains_ * (int(d_use_accel_feedforward_) * dnu_des_ - int(d_use_accel_estimation_) * dnu);
        if (body_fixed_) {
            i_component_ = i_gains_ * error_integral_;
        } else {
            Vector6d error_integral_body;
            error_integral_body.segment<3>(0) = att.toRotationMatrix().inverse()*error_integral_.segment<3>(0);
            error_integral_body.segment<3>(3) = att.toRotationMatrix().inverse()*error_integral_.segment<3>(3);
            i_component_ = i_gains_ * error_integral_body;
        }
        model_component_ = int(use_accel_feedforward_) * M_ * dnu_des_ +  C_ * nu + D_ * nu + g_;
        out = p_component_ + i_component_ + d_component_ + model_component_;
        set_velocity_target_ = false;  // set again to false to test it when settin acceleration target
    }

}
