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

#include "bluerov_ctrl/attitude_skew_symmetric_p_module.hpp"

#include <cmath>
#include <iostream>

namespace bluerov_ctrl {
    void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde) {
        x_tilde << 0, -x(2), x(1),
                x(2), 0, -x(0),
                -x(1), x(0), 0;
    }

    void AttSkewSymmetricPControlModule::update(
        const Eigen::Quaterniond &_orientation, Eigen::Vector3d &out_vel) {


      Eigen::Matrix3d R = _orientation.toRotationMatrix();
      Eigen::Matrix3d R_desired = orientation_target_.toRotationMatrix();
      Eigen::Matrix3d R_error =
          0.5 * (R_desired.transpose() * R - R.transpose() * R_desired); // calculates error in local body fraem


      // use array instead of vector to simplify coefficient-wise operations
      Eigen::Array3d R_error_vector{R_error(1, 2), R_error(2, 0), R_error(0, 1)}; // not the vee map operator that extracts the
      // vector out of a skew-symmetric matrix, takes negative entries instead!

      out_vel = att_mask_ * (p_gains_ * R_error_vector).matrix() + R.inverse() * v_angular_target_;


    }

    void AttSkewSymmetricPControlModule::setOrientationTarget(const double &_roll,
                                                              const double &_pitch,
                                                              const double &_yaw) {
      orientation_target_ = Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX());
    }
}
