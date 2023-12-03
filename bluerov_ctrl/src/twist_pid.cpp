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

#include "bluerov_ctrl/twist_pid.hpp"

namespace bluerov_ctrl {
TwistPIDControl::TwistPIDControl(bool body_fixed) { body_fixed_ = body_fixed; }

//! expects velocity target in body frame
void TwistPIDControl::setVelocityTarget(
    const double &lin_x, const double &lin_y, const double &lin_z,
    const double &ang_x, const double &ang_y, const double &ang_z) {
  nu_des_(0) = lin_x;
  nu_des_(1) = lin_y;
  nu_des_(2) = lin_z;
  nu_des_(3) = ang_x;
  nu_des_(4) = ang_y;
  nu_des_(5) = ang_z;
  set_velocity_target_ = true;
}

//! expects absolute acceleration target in body frame
void TwistPIDControl::setAccelerationTarget(
    const double &lin_x, const double &lin_y, const double &lin_z,
    const double &ang_x, const double &ang_y, const double &ang_z) {
  if (!set_velocity_target_) {
    std::cerr << "Velocity target must be set first, as it is required for the "
                 "calculation of the derivative"
              << std::endl;
  }
  dnu_des_(0) = lin_x;
  dnu_des_(1) = lin_y;
  dnu_des_(2) = lin_z;
  dnu_des_(3) = ang_x;
  dnu_des_(4) = ang_y;
  dnu_des_(5) = ang_z;

  // as we need derivative of linear velocity in body frame and not in inertial
  // frame:
  dnu_des_.segment<3>(0) = dnu_des_.segment<3>(0) -
                           nu_des_.segment<3>(3).cross(nu_des_.segment<3>(0));
}

void TwistPIDControl::update(const double &dt, const Eigen::Quaterniond &att,
                             const Eigen::Vector3d &vel,
                             const Eigen::Vector3d &acc,
                             const Eigen::Vector3d &ang_vel,
                             const Eigen::Vector3d &ang_acc, Vector6d &out) {
  Eigen::Matrix<double, 6, 1> nu, dnu;
  nu.segment<3>(0) = vel;
  nu.segment<3>(3) = ang_vel;
  dnu.segment<3>(0) =
      acc - ang_vel.cross(vel);  // calculation of linear velocity time
                                 // derivative in body coordinate system
  dnu.segment<3>(3) = ang_acc;

  // body fixed controller
  if (body_fixed_) {
    error_integral_ += dt * (nu_des_ - nu);
  } else {
    error_integral_.segment<3>(0) += dt * att.toRotationMatrix() *
                                     (nu_des_.segment<3>(0) - nu.segment<3>(0));
    error_integral_.segment<3>(3) += dt * att.toRotationMatrix() *
                                     (nu_des_.segment<3>(3) - nu.segment<3>(3));
  }
  for (int i = 0; i < error_integral_.cols();
       i++) {  // reset errors if gain is set to zero
    if (i_gains_.diagonal()(i) == 0) {
      error_integral_(i) = 0.0;
    }
  }

  p_component_ = p_gains_ * (nu_des_ - nu);
  d_component_ = d_gains_ * (int(d_use_accel_feedforward_) * dnu_des_ -
                             int(d_use_accel_estimation_) * dnu);
  if (body_fixed_) {
    i_component_ = i_gains_ * error_integral_;
  } else {
    Vector6d error_integral_body;
    error_integral_body.segment<3>(0) =
        att.toRotationMatrix().inverse() * error_integral_.segment<3>(0);
    error_integral_body.segment<3>(3) =
        att.toRotationMatrix().inverse() * error_integral_.segment<3>(3);
    i_component_ = i_gains_ * error_integral_body;
  }
  out = p_component_ + i_component_ + d_component_;
  set_velocity_target_ =
      false;  // set again to false to test it when settin acceleration target
}

}  // namespace bluerov_ctrl
