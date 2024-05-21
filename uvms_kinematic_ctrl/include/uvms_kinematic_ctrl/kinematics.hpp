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

#ifndef UVMS_CTRL_KINEMATICS_HPP
#define UVMS_CTRL_KINEMATICS_HPP

#include "alpha_model/kinematics.hpp"
#include "uvms_common/param_utils.hpp"
#include "uvms_common/rotation_utils.hpp"

// forward declaration of friend classes in their respective namespace
namespace uvms_kin_ctrl {
class Task;

class JointLimitTask;

class JointCenteringTask;

class AUVAttitudeLimitTask;

class EndeffectorTrackingTask;

class ManipulabilityTask;

class ManipulabilityLimitTask;
}  // namespace uvms_kin_ctrl

namespace uvms_kinematics {
using param_utils::n_active_joints;
using param_utils::n_states_uvms;
using param_utils::StateVector;

void skew(const Eigen::Ref<const Eigen::Vector3d> &x,
          Eigen::Ref<Eigen::Matrix3d> x_tilde);

//! Calculates the quaternion error as given in Siciliano, Robotics: Modelling,
//! planning and control
//! => represents rotation error expressed in inertial system (angle-axis
//! representation returns rotation axes in inertial system and angle to be
//! rotated to achieve desired pose \param des desired attitude R_I_B_des \param
//! state current attitude R_I_B \param out error representation as vector in
//! inertial system
void quaternionError(const Eigen::Quaterniond &des,
                     const Eigen::Quaterniond &state,
                     Eigen::Ref<Eigen::Vector3d> out);

void getTransformationEulerBodyRates(const double &r, const double &p,
                                     const double &y,
                                     Eigen::Ref<Eigen::Matrix3d> J);

void getTransformationBodyRatesEuler(const double &r, const double &p,
                                     const double &y,
                                     Eigen::Ref<Eigen::Matrix3d> J);

class Kinematics {
  friend class uvms_kin_ctrl::Task;

  friend class uvms_kin_ctrl::JointLimitTask;

  friend class uvms_kin_ctrl::JointCenteringTask;

  friend class uvms_kin_ctrl::AUVAttitudeLimitTask;

  friend class uvms_kin_ctrl::EndeffectorTrackingTask;

  friend class uvms_kin_ctrl::ManipulabilityTask;

  friend class uvms_kin_ctrl::ManipulabilityLimitTask;

 public:
  Kinematics()
      : auv_pos_(states_.segment<3>(0)),
        auv_att_(states_.segment<3>(3)),
        q_(states_.segment<n_active_joints>(6)) {

        };

  Kinematics(const std::vector<param_utils::TFParam> &tf_param);

  Kinematics(Eigen::Matrix<double, n_active_joints + 1, 4> &DH_table,
             Eigen::Vector3d r_b_0, Eigen::Matrix3d &R_B_0);

  void update(const StateVector &q, const Eigen::Vector3d &pos,
              const Eigen::Quaterniond &att);

  void getJacobian(const StateVector &q, const Eigen::Vector3d &pos,
                   const Eigen::Quaterniond &att,
                   param_utils::UVMSStateMatrix &J);

  void getEefPosition(double &x, double &y, double &z);

  const Eigen::Vector3d &getEefPosition() { return p_eef_; }

  const Eigen::Ref<const Eigen::Vector3d> &getAUVPosition() { return auv_pos_; }

  const Eigen::Matrix3d &getAUVAttitudeRotMat() { return R_I_B_; }

  void getEefAttitude(double &w, double &x, double &y, double &z);

  const Eigen::Quaterniond &getEefAttitude();

  void getEef(Eigen::Vector3d &pos_eef, Eigen::Quaterniond &att_eef);

  //! Get the position of a specific link of the manipulator w.r.t. the base
  //! link \param idx index of the link \return position in base link COS
  const Eigen::Ref<const Eigen::Vector3d> getLinkBaseRefPosition(
      const int &idx);

  void getLinkBaseRefPositionJacobian(
      const int &idx, Eigen::Matrix<double, 3, n_states_uvms> &J);

  void getEefJacobian(Eigen::Matrix<double, 6, n_states_uvms> &J);

 private:
  alpha_kinematics::Kinematics *manipulator_kin_;
  param_utils::UVMSStateVector states_;
  const Eigen::Ref<const Eigen::Vector3d>
      auv_pos_;  //!< reference to position entries in state vector
  const Eigen::Ref<const Eigen::Vector3d>
      auv_att_;  //!< reference to attitude roll pitch yaw entries in state
                 //!< vector
  const Eigen::Ref<const StateVector>
      q_;                  //!< reference to joint angles in state vector
  Eigen::Matrix3d R_I_B_;  //!< Rotation matrix between body- and world-COS
  Eigen::Quaterniond att_0_eef_;  //!< end-effector orientation in base link COS
  Eigen::Vector3d
      r_0_eef_;  //!< end-effector position in manipulator base link COS
  Eigen::Vector3d p_eef_;       //!< end-effector position in inertial COS
  Eigen::Quaterniond att_eef_;  //!< end-effector attitude in inertial COS
  Eigen::Matrix<double, 3, n_active_joints>
      J_manipulator_pos_;  //!< manipulator Jacobian matrix
  Eigen::Matrix<double, 3, n_active_joints>
      J_manipulator_rot_;        //!< manipulator Jacobian matrix
  const Eigen::Vector3d r_B_0_;  //!< vector from auv base link to manipulator
                                 //!< base origin in base link COS
  const Eigen::Matrix3d R_B_0_;  //!< rotation matrix between auv base link and
                                 //!< manipulator base link
};

}  // namespace uvms_kinematics

#endif  // UVMS_CTRL_KINEMATICS_HPP
