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

#ifndef BLUEROV_CTRL_KF_HPP
#define BLUEROV_CTRL_KF_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "uvms_common/param_utils.hpp"

namespace bluerov_estimation {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

//! linear Kalman filter for estimating acceleration from velocity measurements,
//! linear entities should be expressed in inertial frame to achieve decoupling
//! of attitude change, angular entities should be expressed in local frame
class KFLinear {
 public:
  struct vNoise {
    Eigen::Vector3d vel;  //!< process noise on velocity
    Eigen::Vector3d acc;  //!< process noise on acceleration
    vNoise() {
      vel.setZero();
      acc.setZero();
    }
  };

  struct wNoise {
    Eigen::Vector3d vel;  //!< measurement noise on velocity
    wNoise() { vel.setZero(); }
  };

  static constexpr int n_est_states = 2 * 3;
  static constexpr int n_meas_states = 1 * 3;
  static constexpr int n_v_noise = 2 * 3;
  static constexpr int n_w_noise = 1 * 3;
  typedef Eigen::Matrix<double, n_est_states, 1> KFStateVector;
  typedef Eigen::Matrix<double, n_meas_states, 1> KFMeasVector;
  typedef Eigen::Matrix<double, n_est_states, n_est_states> KFStateMatrix;

  KFLinear();

  void initialize(KFStateVector x0, KFStateMatrix P0, vNoise v_noise,
                  wNoise w_noise);

  void predict(const double &del_t);

  void update(const KFMeasVector &z);

  const KFStateVector &getEstimate() { return x_est_; }

  //! Returns velocity estimate, for linear case in the inertial coordinate
  //! system, for angular case in the local coordinate system \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstVel() { return x_vel_; }

  //! Returns acceleration estimate, for linear case in the inertial coordinate
  //! system, for angular case in the local coordinate system \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstAcc() { return x_acc_; }

  void setVNoise(const double &noise, int idx);

  void setWNoise(const double &noise, int idx);

 private:
  void updateA(const double &del_t);

  // todo: delete
  KFStateVector x_predicted_;

  KFStateVector x_est_;  //!< estimated states
  const Eigen::Ref<const Eigen::Vector3d>
      x_vel_;  //!< reference to velocity part of x_est, global linear velocity,
               //!< local angular velocity
  const Eigen::Ref<const Eigen::Vector3d>
      x_acc_;  //!< reference to acceleration part of x_est, global
               //!< acceleration, local angular acceleration

  KFStateMatrix P_;                                   //!< estimated covariance
  Eigen::Matrix<double, n_v_noise, n_v_noise> Sigv_;  //!< process noise matrix
  Eigen::Matrix<double, n_w_noise, n_w_noise>
      Sigw_;  //!< measurement noise matrix

  KFStateMatrix A_;  //!< dynamic model matrix
  Eigen::Matrix<double, n_meas_states, n_est_states>
      H_;  //!< measurement model matrix
};

Eigen::Matrix3d skew(const Eigen::Ref<const Eigen::Vector3d> &x);

template <typename T>
int sgn(T val);
//! nonlinear Kalman filter for estimating acceleration from velocity
//! measurements, linear and angular entities should be expressed in local frame
class EKF {
 public:
  struct vNoise {
    Eigen::Vector<double, 6>
        vel;  //!< process noise on velocity, linear and angular
    Eigen::Vector<double, 6>
        acc;  //!< process noise on acceleration, linear and angular
    vNoise() {
      vel.setZero();
      acc.setZero();
    }
  };

  struct wNoise {
    Eigen::Vector<double, 6> vel;  //!< measurement noise on velocity
    wNoise() { vel.setZero(); }
  };

  static constexpr int n_est_states = 2 * 6;
  static constexpr int n_meas_states = 1 * 6;
  static constexpr int n_v_noise = 2 * 6;
  static constexpr int n_w_noise = 1 * 6;
  typedef Eigen::Matrix<double, n_est_states, 1> KFStateVector;
  typedef Eigen::Matrix<double, n_meas_states, 1> KFMeasVector;
  typedef Eigen::Matrix<double, n_est_states, n_est_states> KFStateMatrix;

  EKF(Matrix6d M, Vector6d damping_linear, Vector6d damping_nonlinear,
      Eigen::Vector3d cog, Eigen::Vector3d cob, double mass, double buoyancy);

  void initialize(KFStateVector x0, KFStateMatrix P0, vNoise v_noise,
                  wNoise w_noise);

  void predict(const double &del_t, const Eigen::Quaterniond &att,
               const Vector6d &thrust_wrench,
               const Vector6d &manipulator_wrench);

  void update(const KFMeasVector &z);

  const KFStateVector &getEstimate() { return x_est_; }

  //! Returns velocity estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstLinVel() { return x_vel_; }

  //! Returns acceleration estimate in the local coordinate system
  //! \return estimate
  const Eigen::Vector3d &getEstLinAcc() { return x_acc_; }

  //! Returns angular velocity estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstAngVel() { return x_ang_vel_; }

  //! Returns angular acceleration estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstAngAcc() { return x_ang_acc_; }

  void setVNoise(const double &noise, int idx);

  void setWNoise(const double &noise, int idx);

 private:
  void updateModel(const double &del_t, const Eigen::Quaterniond &att);

  KFStateVector x_est_;  //!< estimated states
  const Eigen::Ref<const Eigen::Vector3d>
      x_vel_;  //!< reference to local velocity part of x_est
  const Eigen::Ref<const Eigen::Vector3d>
      x_ang_vel_;  //!< reference to local angular velocity part of x_est
  const Eigen::Ref<const Eigen::Vector3d>
      x_dvel_;  //!< reference to local velocity derivative part of x_est => not
                //!< absolute acceleration!
  const Eigen::Ref<const Eigen::Vector3d>
      x_ang_acc_;  //!< reference to local angular acceleration part of x_est
  Eigen::Vector3d x_acc_;  //!< absolute acceleration in local coordinate system

  KFStateMatrix P_;                                   //!< estimated covariance
  Eigen::Matrix<double, n_v_noise, n_v_noise> Sigv_;  //!< process noise matrix
  Eigen::Matrix<double, n_w_noise, n_w_noise>
      Sigw_;  //!< measurement noise matrix

  KFStateMatrix A_;  //!< dynamic model state derivative
  Eigen::Matrix<double, n_est_states, n_v_noise>
      L_;  //!< dynamic model noise derivative
  Eigen::Matrix<double, n_meas_states, n_est_states>
      H_;  //!< measurement model state derivative
  Eigen::Matrix<double, n_meas_states, n_est_states>
      M_meas_;  //!< measurement model noise derivative

  // Model parameters:
  const Matrix6d
      M_;       //!< mass matrix consisting of added mass and rigid body masss
  Matrix6d C_;  //!< Coriolis matrix with rigid body and added mass
  Matrix6d D_;  //!< damping matrix
  const Vector6d
      damping_linear_;  //!< vector containing of linear damping parameters
  const Vector6d damping_nonlinear_;  //!< vector containing of nonlinear
                                      //!< damping parameters
  const double mass_;
  const double buoyancy_;
  const Eigen::Vector3d cog_;  //!< center of gravity in body coordinate system
  const Eigen::Vector3d cob_;  //!< center of buoyancy in body coordinate system
  Vector6d g_;  //!< vector of gravity and buoyancy forces and torques
};

//! linear Kalman filter for estimating acceleration from velocity measurements,
//! linear and angular entities should be expressed in local frame
class KFFeedforward {
 public:
  struct vNoise {
    Eigen::Vector<double, 6>
        vel;  //!< process noise on velocity, linear and angular
    Eigen::Vector<double, 6>
        acc;  //!< process noise on acceleration, linear and angular
    vNoise() {
      vel.setZero();
      acc.setZero();
    }
  };

  struct wNoise {
    Eigen::Vector<double, 6> vel;  //!< measurement noise on velocity
    wNoise() { vel.setZero(); }
  };

  static constexpr int n_est_states = 2 * 6;
  static constexpr int n_meas_states = 1 * 6;
  static constexpr int n_v_noise = 2 * 6;
  static constexpr int n_w_noise = 1 * 6;
  typedef Eigen::Matrix<double, n_est_states, 1> KFStateVector;
  typedef Eigen::Matrix<double, n_meas_states, 1> KFMeasVector;
  typedef Eigen::Matrix<double, n_est_states, n_est_states> KFStateMatrix;

  KFFeedforward(Matrix6d M, Vector6d damping_linear, Vector6d damping_nonlinear,
                Eigen::Vector3d cog, Eigen::Vector3d cob, double mass,
                double buoyancy, double buoyancy_comp,
                Eigen::Vector3d buoyancy_comp_origin);

  void initialize(KFStateVector x0, KFStateMatrix P0, vNoise v_noise,
                  wNoise w_noise);

  void updateKFInput(const Eigen::Quaterniond &att, const Vector6d &nu,
                     const Vector6d &thrust_wrench,
                     const Vector6d &manipulator_wrench);

  void predict(const double &del_t);

  void update(const KFMeasVector &z);

  const KFStateVector &getEstimate() { return x_est_; }

  //! Returns velocity estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstLinVel() { return x_vel_; }

  //! Returns acceleration estimate in the local coordinate system
  //! \return estimate
  const Eigen::Vector3d &getEstLinAcc() { return x_acc_; }

  //! Returns angular velocity estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstAngVel() { return x_ang_vel_; }

  //! Returns angular acceleration estimate in the local coordinate system
  //! \return estimate
  const Eigen::Ref<const Eigen::Vector3d> &getEstAngAcc() { return x_ang_acc_; }

  void setVNoise(const double &noise, int idx);

  void setWNoise(const double &noise, int idx);

  void setMovingAverage(const double &moving_average) {
    moving_average_constant_ = std::clamp(moving_average, 0.01, 1.0);
  }

 private:
  void updateA(const double &del_t);

  KFStateVector x_est_;  //!< estimated states
  const Eigen::Ref<const Eigen::Vector3d>
      x_vel_;  //!< reference to local velocity part of x_est
  const Eigen::Ref<const Eigen::Vector3d>
      x_ang_vel_;  //!< reference to local angular velocity part of x_est
  const Eigen::Ref<const Eigen::Vector3d>
      x_dvel_;  //!< reference to local velocity derivative part of x_est => not
                //!< absolute acceleration!
  const Eigen::Ref<const Eigen::Vector3d>
      x_ang_acc_;  //!< reference to local angular acceleration part of x_est
  Eigen::Vector3d x_acc_;  //!< absolute acceleration in local coordinate system

  // todo: delete
  KFStateVector x_predicted_;
  KFStateMatrix P_;                                   //!< estimated covariance
  Eigen::Matrix<double, n_v_noise, n_v_noise> Sigv_;  //!< process noise matrix
  Eigen::Matrix<double, n_w_noise, n_w_noise>
      Sigw_;  //!< measurement noise matrix

  KFStateMatrix A_;  //!< dynamic model state derivative
  Eigen::Matrix<double, n_est_states, n_v_noise>
      L_;  //!< dynamic model noise derivative
  Eigen::Matrix<double, n_meas_states, n_est_states>
      H_;  //!< measurement model state derivative
  Eigen::Matrix<double, n_meas_states, n_meas_states>
      M_meas_;  //!< measurement model noise derivative

  // Model parameters:
  const Matrix6d
      M_;       //!< mass matrix consisting of added mass and rigid body masss
  Matrix6d C_;  //!< Coriolis matrix with rigid body and added mass
  Matrix6d D_;  //!< damping matrix
  const Vector6d
      damping_linear_;  //!< vector containing of linear damping parameters
  const Vector6d damping_nonlinear_;  //!< vector containing of nonlinear
                                      //!< damping parameters
  const double mass_;
  const double buoyancy_;
  const Eigen::Vector3d cog_;  //!< center of gravity in body coordinate system
  const Eigen::Vector3d cob_;  //!< center of buoyancy in body coordinate system
  Vector6d g_;  //!< vector of gravity and buoyancy forces and torques

  double added_buoyancy_mass_;  //!< added buoyancy due to removed weights
  Eigen::Vector3d
      added_buoyancy_origin_;  //!< added buoyancy due to removed weights origin

  double moving_average_constant_{
      1.0};  //!< constant to calculate thruster_wrench moving average with
             //!< thruster_wrench_new = thruster_wrench_new * constant +
             //!< thruster_wrench_old * (1 - constant)
  Vector6d model_input_;  //!< new feed-forward term
  Vector6d
      last_model_input_;  //!< calculated feed-forward term from last update
};

}  // namespace bluerov_estimation
#endif  // BLUEROV_CTRL_KF_HPP
