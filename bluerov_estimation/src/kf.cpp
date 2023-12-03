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

#include "bluerov_estimation/kf.hpp"

namespace bluerov_estimation {

KFLinear::KFLinear()
    : x_vel_(x_est_.segment<3>(0)), x_acc_(x_est_.segment<3>(3)) {}

void KFLinear::initialize(KFLinear::KFStateVector x0,
                          KFLinear::KFStateMatrix P0, vNoise v_noise,
                          wNoise w_noise) {
  Sigv_.setZero();
  Sigv_.block<3, 3>(0, 0).diagonal() = v_noise.vel;
  Sigv_.block<3, 3>(3, 3).diagonal() = v_noise.acc;
  Sigw_.setZero();
  Sigw_.block<3, 3>(0, 0).diagonal() = w_noise.vel;
  x_est_ = x0;
  P_ = P0;
  A_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  A_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  H_.setZero();
  int n = std::min(H_.cols(), H_.rows());
  for (int i = 0; i < n; i++) {
    H_(i, i) = 1.0;
  }
}

void KFLinear::predict(const double &del_t) {
  updateA(del_t);
  x_est_ = A_ * x_est_;
  P_ = A_ * P_ * A_.transpose() + Sigv_;
  x_predicted_ = x_est_;
}

void KFLinear::update(const KFLinear::KFMeasVector &z) {
  Eigen::Matrix<double, n_est_states, n_meas_states> K =
      P_ * H_.transpose() * (H_ * P_ * H_.transpose() + Sigw_).inverse();
  x_est_ = x_est_ + K * (z - H_ * x_est_);
  P_ = (KFStateMatrix::Identity() - K * H_) * P_ *
           (KFStateMatrix::Identity() - K * H_).transpose() +
       K * Sigw_ * K.transpose();
}

void KFLinear::updateA(const double &del_t) {
  A_.block<3, 3>(0, 3) = del_t * Eigen::Matrix3d::Identity();
}

void KFLinear::setVNoise(const double &noise, int idx) {
  if (idx > n_v_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_v_noise - 1 << std::endl;
    return;
  }
  Sigv_.diagonal()(idx) = noise;
}

void KFLinear::setWNoise(const double &noise, int idx) {
  if (idx > n_w_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_w_noise - 1 << std::endl;
    return;
  }
  Sigw_.diagonal()(idx) = noise;
}

Eigen::Matrix3d skew(const Eigen::Ref<const Eigen::Vector3d> &x) {
  Eigen::Matrix3d x_tilde;
  x_tilde << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  return x_tilde;
}

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

EKF::EKF(Matrix6d M, Vector6d damping_linear, Vector6d damping_nonlinear,
         Eigen::Vector3d cog, Eigen::Vector3d cob, double mass, double buoyancy)
    : x_vel_(x_est_.segment<3>(0)),
      x_ang_vel_(x_est_.segment<3>(3)),
      x_dvel_(x_est_.segment<3>(6)),
      x_ang_acc_(x_est_.segment<3>(9)),
      M_(M),
      damping_linear_(damping_linear),
      damping_nonlinear_(damping_nonlinear),
      mass_(mass),
      buoyancy_(buoyancy),
      cog_(cog),
      cob_(cob) {}

void EKF::initialize(EKF::KFStateVector x0, EKF::KFStateMatrix P0,
                     EKF::vNoise v_noise, EKF::wNoise w_noise) {
  Sigv_.setZero();
  Sigv_.block<6, 6>(0, 0).diagonal() = v_noise.vel;
  Sigv_.block<6, 6>(6, 6).diagonal() = v_noise.acc;
  Sigw_.setZero();
  Sigw_.block<6, 6>(0, 0).diagonal() = w_noise.vel;
  x_est_ = x0;
  P_ = P0;
  A_.setZero();
  A_.block<6, 6>(0, 0) = Matrix6d::Identity();
  L_.setZero();
  L_.block<6, 6>(0, 0) = Matrix6d::Identity();
  L_.block<6, 6>(6, 6) = M_.inverse();
  H_.setZero();
  for (int i = 0; i < n_meas_states; i++) {
    H_(i, i) = 1.0;
  }
  M_meas_.setIdentity();
}

void EKF::predict(const double &del_t, const Eigen::Quaterniond &att,
                  const Vector6d &thrust_wrench,
                  const Vector6d &manipulator_wrench) {
  updateModel(del_t, att);
  x_est_.segment<6>(0) = x_est_.segment<6>(0) + del_t * x_est_.segment<6>(6);
  x_est_.segment<6>(6) = M_.inverse() * (thrust_wrench + manipulator_wrench -
                                         C_ * x_est_.segment<6>(0) -
                                         D_ * x_est_.segment<6>(0) - g_);
  P_ = A_ * P_ * A_.transpose() + L_ * Sigv_ * L_.transpose();
}

void EKF::updateModel(const double &del_t, const Eigen::Quaterniond &att) {
  // calculate gravity force vector and buoyancy
  const Eigen::Vector3d gravity_force =
      mass_ * att.toRotationMatrix().inverse() *
      Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
  const Eigen::Vector3d buoyancy_force =
      -buoyancy_ * att.toRotationMatrix().inverse() *
      Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
  g_.segment<3>(0) =
      -(gravity_force +
        buoyancy_force);  // as included on the left side of the equations, the
                          // entities must be negative
  g_.segment<3>(3) = -(
      cog_.cross(gravity_force) +
      cob_.cross(buoyancy_force));  // as included on the left side of the
                                    // equations, the entities must be negative

  // update Coriolis matrix
  C_.block<3, 3>(0, 0).setZero();
  C_.block<3, 3>(0, 3) =
      -skew(M_.block<3, 3>(0, 0) * x_vel_ + M_.block<3, 3>(0, 3) * x_ang_vel_);
  C_.block<3, 3>(3, 0) =
      -skew(M_.block<3, 3>(0, 0) * x_vel_ + M_.block<3, 3>(0, 3) * x_ang_vel_);
  C_.block<3, 3>(3, 3) =
      -skew(M_.block<3, 3>(3, 0) * x_vel_ + M_.block<3, 3>(3, 3) * x_ang_vel_);

  // update damping matrix
  D_.setZero();
  D_.diagonal().segment<3>(0) =
      damping_linear_.segment<3>(0) +
      (damping_nonlinear_.segment<3>(0).array() * x_vel_.array().abs())
          .matrix();
  D_.diagonal().segment<3>(3) =
      damping_linear_.segment<3>(3) +
      (damping_nonlinear_.segment<3>(3).array() * x_vel_.array().abs())
          .matrix();

  A_.block<6, 6>(0, 6) = del_t * Matrix6d::Identity();

  // the derivatives are calculate symbolically and assume purely diagonal added
  // mass and rigid body inertia tensor for the mass matrix and purely diagonal
  // damping
  Matrix6d dCdnu;
  dCdnu << 0, -x_ang_vel_(2) * M_(1, 1), x_ang_vel_(1) * M_(2, 2), 0,
      x_vel_(2) * M_(2, 2), -x_vel_(1) * M_(1, 1), x_ang_vel_(2) * M_(0, 0), 0,
      -x_ang_vel_(0) * M_(2, 2), -x_vel_(2) * M_(2, 2), 0, x_vel_(0) * M_(0, 0),
      -x_ang_vel_(1) * M_(0, 0), x_ang_vel_(0) * M_(1, 1), 0,
      x_vel_(1) * M_(1, 1), -x_vel_(0) * M_(0, 0), 0, 0,
      -x_vel_(2) * M_(1, 1) + x_vel_(2) * M_(2, 2),
      -x_vel_(1) * M_(1, 1) + x_vel_(1) * M_(2, 2), 0,
      -x_ang_vel_(2) * M_(4, 4) + x_ang_vel_(2) * M_(5, 5),
      -x_ang_vel_(1) * M_(4, 4) + x_ang_vel_(1) * M_(5, 5),
      x_vel_(2) * M_(0, 0) - x_vel_(2) * M_(2, 2), 0,
      x_vel_(0) * M_(0, 0) - x_vel_(0) * M_(2, 2),
      x_ang_vel_(2) * M_(3, 3) - x_ang_vel_(2) * M_(5, 5), 0,
      x_ang_vel_(0) * M_(3, 3) - x_ang_vel_(0) * M_(5, 5),
      -x_vel_(1) * M_(0, 0) + x_vel_(1) * M_(1, 1),
      -x_vel_(0) * M_(0, 0) + x_vel_(0) * M_(1, 1), 0,
      -x_ang_vel_(1) * M_(3, 3) + x_ang_vel_(1) * M_(4, 4),
      -x_ang_vel_(0) * M_(3, 3) + x_ang_vel_(0) * M_(4, 4), 0;
  Matrix6d dDdnu;
  dDdnu << damping_linear_(0) +
               damping_nonlinear_(0) * x_vel_(0) * sgn(x_vel_(0)) +
               damping_nonlinear_(0) * std::abs(x_vel_(0)),
      0, 0, 0, 0, 0, 0,
      damping_linear_(1) + damping_nonlinear_(1) * x_vel_(1) * sgn(x_vel_(1)) +
          damping_nonlinear_(1) * std::abs(x_vel_(1)),
      0, 0, 0, 0, 0, 0,
      damping_linear_(2) + damping_nonlinear_(2) * x_vel_(2) * sgn(x_vel_(2)) +
          damping_nonlinear_(2) * std::abs(x_vel_(2)),
      0, 0, 0, 0, 0, 0,
      damping_linear_(3) +
          damping_nonlinear_(3) * x_ang_vel_(0) * sgn(x_ang_vel_(0)) +
          damping_nonlinear_(3) * std::abs(x_ang_vel_(0)),
      0, 0, 0, 0, 0, 0,
      damping_linear_(4) +
          damping_nonlinear_(4) * x_ang_vel_(1) * sgn(x_ang_vel_(1)) +
          damping_nonlinear_(4) * std::abs(x_ang_vel_(1)),
      0, 0, 0, 0, 0, 0,
      damping_linear_(5) +
          damping_nonlinear_(5) * x_ang_vel_(2) * sgn(x_ang_vel_(2)) +
          damping_nonlinear_(5) * std::abs(x_ang_vel_(2));

  A_.block<6, 6>(6, 0) = -M_.inverse() * (dCdnu + dDdnu);
}

void EKF::update(const EKF::KFMeasVector &z) {
  Eigen::Matrix<double, n_est_states, n_meas_states> K =
      P_ * H_.transpose() *
      (H_ * P_ * H_.transpose() + M_ * Sigw_ * M_).inverse();
  x_est_ = x_est_ + K * (z - H_ * x_est_);
  P_ = (KFStateMatrix::Identity() - K * H_) * P_ *
           (KFStateMatrix::Identity() - K * H_).transpose() +
       K * Sigw_ * K.transpose();
  x_acc_ = x_dvel_ + x_ang_vel_.cross(x_vel_);
}

void EKF::setVNoise(const double &noise, int idx) {
  if (idx > n_v_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_v_noise - 1 << std::endl;
    return;
  }
  Sigv_.diagonal()(idx) = noise;
}

void EKF::setWNoise(const double &noise, int idx) {
  if (idx > n_w_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_w_noise - 1 << std::endl;
    return;
  }
  Sigw_.diagonal()(idx) = noise;
}

KFFeedforward::KFFeedforward(Matrix6d M, Vector6d damping_linear,
                             Vector6d damping_nonlinear, Eigen::Vector3d cog,
                             Eigen::Vector3d cob, double mass, double buoyancy,
                             double buoyancy_comp,
                             Eigen::Vector3d buoyancy_comp_origin)
    : x_vel_(x_est_.segment<3>(0)),
      x_ang_vel_(x_est_.segment<3>(3)),
      x_dvel_(x_est_.segment<3>(6)),
      x_ang_acc_(x_est_.segment<3>(9)),
      M_(M),
      damping_linear_(damping_linear),
      damping_nonlinear_(damping_nonlinear),
      mass_(mass),
      buoyancy_(buoyancy),
      cog_(cog),
      cob_(cob),
      added_buoyancy_mass_(buoyancy_comp),
      added_buoyancy_origin_(buoyancy_comp_origin) {}

void KFFeedforward::initialize(KFFeedforward::KFStateVector x0,
                               KFFeedforward::KFStateMatrix P0,
                               KFFeedforward::vNoise v_noise,
                               KFFeedforward::wNoise w_noise) {
  Sigv_.setZero();
  Sigv_.block<6, 6>(0, 0).diagonal() = v_noise.vel;
  Sigv_.block<6, 6>(6, 6).diagonal() = v_noise.acc;
  Sigw_.setZero();
  Sigw_.block<6, 6>(0, 0).diagonal() = w_noise.vel;
  x_est_ = x0;
  P_ = P0;
  A_.setZero();
  A_.setIdentity();
  L_.setZero();
  L_.setIdentity();
  H_.setZero();
  for (int i = 0; i < n_meas_states; i++) {
    H_(i, i) = 1.0;
  }
  M_meas_.setIdentity();
  last_model_input_.setZero();
  model_input_.setZero();
}

void KFFeedforward::predict(const double &del_t) {
  updateA(del_t);
  x_est_ = A_ * x_est_;
  x_est_.segment<6>(6) +=
      model_input_ - last_model_input_;  // apply model feedforward input
  P_ = A_ * P_ * A_.transpose() + L_ * Sigv_ * L_.transpose();
  x_predicted_ = x_est_;
}

void KFFeedforward::updateA(const double &del_t) {
  A_.block<6, 6>(0, 6) = del_t * Matrix6d::Identity();
}

void KFFeedforward::updateKFInput(const Eigen::Quaterniond &att,
                                  const Vector6d &nu,
                                  const Vector6d &thrust_wrench,
                                  const Vector6d &manipulator_wrench) {
  // Vector6d nu_test = x_est_.segment<6>(0);
  Vector6d nu_test = nu;
  last_model_input_ = model_input_;
  // calculate gravity force vector and buoyancy
  const Eigen::Vector3d gravity_force =
      mass_ * att.toRotationMatrix().inverse() *
      Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
  const Eigen::Vector3d buoyancy_force =
      -buoyancy_ * att.toRotationMatrix().inverse() *
      Eigen::Vector3d(0.0, 0.0, -param_utils::GRAVITY);
  g_.segment<3>(0) =
      -(gravity_force +
        buoyancy_force);  // as included on the left side of the equations, the
                          // entities must be negative
  g_.segment<3>(3) = -(
      cog_.cross(gravity_force) +
      cob_.cross(buoyancy_force));  // as included on the left side of the
                                    // equations, the entities must be negative

  // update Coriolis matrix
  C_.block<3, 3>(0, 0).setZero();
  C_.block<3, 3>(0, 3) = -skew(M_.block<3, 3>(0, 0) * nu_test.segment<3>(0) +
                               M_.block<3, 3>(0, 3) * nu_test.segment<3>(3));
  C_.block<3, 3>(3, 0) = -skew(M_.block<3, 3>(0, 0) * nu_test.segment<3>(0) +
                               M_.block<3, 3>(0, 3) * nu_test.segment<3>(3));
  C_.block<3, 3>(3, 3) = -skew(M_.block<3, 3>(3, 0) * nu_test.segment<3>(0) +
                               M_.block<3, 3>(3, 3) * nu_test.segment<3>(3));

  // update damping matrix
  D_.setZero();
  D_.diagonal().segment<3>(0) = damping_linear_.segment<3>(0) +
                                (damping_nonlinear_.segment<3>(0).array() *
                                 nu_test.segment<3>(0).array().abs())
                                    .matrix();
  D_.diagonal().segment<3>(3) = damping_linear_.segment<3>(3) +
                                (damping_nonlinear_.segment<3>(3).array() *
                                 nu_test.segment<3>(0).array().abs())
                                    .matrix();

  // calculate forces to applied by buoyancy through removed weights
  Vector6d wrench_buoyancy_added;
  wrench_buoyancy_added.segment<3>(0) =
      att.toRotationMatrix().inverse() * added_buoyancy_mass_ *
      Eigen::Vector3d::UnitZ() * (param_utils::GRAVITY);
  wrench_buoyancy_added.segment<3>(3) = added_buoyancy_origin_.cross(
      att.toRotationMatrix().inverse() * added_buoyancy_mass_ *
      Eigen::Vector3d::UnitZ() * (param_utils::GRAVITY));

  model_input_ = moving_average_constant_ *
                     (M_.inverse() *
                      (thrust_wrench + wrench_buoyancy_added +
                       manipulator_wrench - C_ * nu_test - D_ * nu_test - g_)) +
                 (1 - moving_average_constant_) * last_model_input_;
}

void KFFeedforward::update(const KFFeedforward::KFMeasVector &z) {
  Eigen::Matrix<double, n_est_states, n_meas_states> K =
      P_ * H_.transpose() *
      (H_ * P_ * H_.transpose() + M_meas_ * Sigw_ * M_meas_).inverse();
  x_est_ = x_est_ + K * (z - H_ * x_est_);
  P_ = (KFStateMatrix::Identity() - K * H_) * P_ *
           (KFStateMatrix::Identity() - K * H_).transpose() +
       K * Sigw_ * K.transpose();
  x_acc_ = x_dvel_ + x_ang_vel_.cross(x_vel_);
}

void KFFeedforward::setVNoise(const double &noise, int idx) {
  if (idx > n_v_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_v_noise - 1 << std::endl;
    return;
  }
  Sigv_.diagonal()(idx) = noise;
}

void KFFeedforward::setWNoise(const double &noise, int idx) {
  if (idx > n_w_noise - 1) {
    std::cerr << "Specified index " << idx << " out of range of valid indexes "
              << n_w_noise - 1 << std::endl;
    return;
  }
  Sigw_.diagonal()(idx) = noise;
}
}  // namespace bluerov_estimation