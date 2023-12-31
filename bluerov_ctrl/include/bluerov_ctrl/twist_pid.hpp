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

#ifndef BLUEROV_CTRL_TWIST_PID_HPP
#define BLUEROV_CTRL_TWIST_PID_HPP

#include <eigen3/Eigen/Dense>

#include "uvms_common/param_utils.hpp"

namespace bluerov_ctrl {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class TwistPIDControl {
 public:
  TwistPIDControl(bool body_fixed);

  //! expects velocity target in body frame
  void setVelocityTarget(const double &lin_x, const double &lin_y,
                         const double &lin_z, const double &ang_x,
                         const double &ang_y, const double &ang_z);

  //! expects absolute acceleration target in body frame
  void setAccelerationTarget(const double &lin_x, const double &lin_y,
                             const double &lin_z, const double &ang_x,
                             const double &ang_y, const double &ang_z);

  void resetIntegral() { error_integral_.setZero(); }

  void setPgain(const double &gain, int idx) {
    p_gains_.diagonal()(idx) = gain;
  }

  void setIgain(const double &gain, int idx) {
    i_gains_.diagonal()(idx) = gain;
    if (gain == 0.0) {
      error_integral_(idx) = 0.0;
    }
  }

  void setDgain(const double &gain, int idx) {
    d_gains_.diagonal()(idx) = gain;
  }

  void setDUseAccelFeedforward(bool d_use_accel_feedforward) {
    d_use_accel_feedforward_ = d_use_accel_feedforward;
  }
  void setDUseAccelEstimation(bool d_use_accel_estimation) {
    d_use_accel_estimation_ = d_use_accel_estimation;
  }

  const Vector6d &getPComponent() { return p_component_; }

  const Vector6d &getIComponent() { return i_component_; }

  const Vector6d &getDComponent() { return d_component_; }

  //! Returns desired thrust forces in body-fixed coordinate system
  //! \param vel velocity in body frame
  //! \param acc acceleration in body frame
  //! \param out desired thrust forces in body frame
  void update(const double &dt, const Eigen::Quaterniond &att,
              const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
              const Eigen::Vector3d &ang_vel, const Eigen::Vector3d &ang_acc,
              Vector6d &out);

 private:
  Eigen::DiagonalMatrix<double, 6> p_gains_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  Eigen::DiagonalMatrix<double, 6> i_gains_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Eigen::DiagonalMatrix<double, 6> d_gains_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Vector6d nu_des_{0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0};  //!< velocity target in body frame
  Vector6d dnu_des_{0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0};  //!< velocity derivative target in body frame
  Vector6d error_integral_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // components for PID parts and model based part for debugging
  Vector6d p_component_;    //!< p component of the controller
  Vector6d i_component_;    //!< i component of the controller
  Vector6d d_component_;    //!< d component of the controller
  bool body_fixed_ = true;  //!< decides if the error integration is executed in
                            //!< inertial or body frame
  bool set_velocity_target_ = false;
  bool d_use_accel_feedforward_ = false;
  bool d_use_accel_estimation_ = false;
};

}  // namespace bluerov_ctrl
#endif  // BLUEROV_CTRL_TWIST_PID_HPP
