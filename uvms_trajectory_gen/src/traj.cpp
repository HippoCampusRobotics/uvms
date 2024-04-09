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

#include "uvms_trajectory_gen/traj.hpp"

namespace uvms_traj_gen {

void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde) {
  x_tilde << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
}

void Traj::publishStatus(int status) {
  std_msgs::msg::Int64 msg;
  msg.data = status;
  status_pub_ptr_->publish(msg);
}

void Traj::getStartPosition(Eigen::Vector3d &pos) {
  EefTrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  pos = setpoint.pos;
}

void Traj::getStartOrientation(Eigen::Quaterniond &att) {
  EefTrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  att = setpoint.att;
}

void Traj::getStartPose(Eigen::Vector3d &pos, Eigen::Quaterniond &att) {
  EefTrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  pos = setpoint.pos;
  att = setpoint.att;
}

void Traj::declareStandardParams(bool output) {
  ros_param_utils::getParam(node_ptr_, att_dofs_tracked_, "att_dofs_tracked", 2,
                            output);
  ros_param_utils::getParam(node_ptr_, offset_(0),
                            parameter_prefixes.at(trajType()) + "startpoint.x",
                            2.0, output);
  ros_param_utils::getParam(node_ptr_, offset_(1),
                            parameter_prefixes.at(trajType()) + "startpoint.y",
                            1.0, output);
  ros_param_utils::getParam(node_ptr_, offset_(2),
                            parameter_prefixes.at(trajType()) + "startpoint.z",
                            -0.5, output);

  double rot_x, rot_y, rot_z;
  ros_param_utils::getParam(node_ptr_, rot_x,
                            parameter_prefixes.at(trajType()) + "rotation.x",
                            0.0, output);
  ros_param_utils::getParam(node_ptr_, rot_y,
                            parameter_prefixes.at(trajType()) + "rotation.y",
                            0.0, output);
  ros_param_utils::getParam(node_ptr_, rot_z,
                            parameter_prefixes.at(trajType()) + "rotation.z",
                            0.0, output);
  rot_ = (Eigen::AngleAxisd(rot_z * M_PI, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rot_y * M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rot_x * M_PI, Eigen::Vector3d::UnitX()))
             .toRotationMatrix();
  ros_param_utils::getParam(
      node_ptr_, rot_tangential_,
      parameter_prefixes.at(trajType()) + "rot_tangential", 0.0, output);
  rot_tangential_ = rot_tangential_ * M_PI;
}

void Traj::getSetpoint(const double &t, EefTrajSetpoint &out) {
  getTranslationalMotion(t, out.pos, out.vel);
  out.acc.setZero();

  getAttitude(t, out.vel, out.att, false);
  Eigen::Vector3d pos_backward, vel_backward, acc_backward;
  double t_backward = t - eps_fd_;
  getTranslationalMotion(t_backward, pos_backward, vel_backward);
  Eigen::Quaterniond att_backward;
  getAttitude(t_backward, vel_backward, att_backward, false);

  Eigen::Vector3d pos_forward, vel_forward;
  double t_forward = t + eps_fd_;
  getTranslationalMotion(t_forward, pos_forward, vel_forward);
  Eigen::Quaterniond att_forward;
  getAttitude(t_forward, vel_forward, att_forward, false);
  Eigen::AngleAxisd datt_central =
      static_cast<Eigen::AngleAxisd>(att_backward.inverse() * att_forward);
  Eigen::Vector3d w_central = att_backward.toRotationMatrix() *
                              datt_central.angle() * datt_central.axis() /
                              (2 * eps_fd_);
  out.ang_vel = w_central;
}

void Traj::getSetpointMsg(const double &t,
                          hippo_control_msgs::msg::ControlTarget &out) {
  if (isFinished(t)) {
    *status_ptr_ = TrajStatus::finished_motion;
    publishStatus(TrajStatus::finished_motion);
    return;
  }
  EefTrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::kInertialName;
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
  switch (att_dofs_tracked_) {
    case 3:
      out.mask = 0;
      break;
    case 2:
      out.mask = hippo_control_msgs::msg::ControlTarget::IGNORE_ATTITUDE_Y;
      break;
    case 1:
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "No valid number of attitude dofs specified to be tracked. "
                   "Track only position");
      out.mask = hippo_control_msgs::msg::ControlTarget::IGNORE_ATTITUDE;
      break;
    case 0:
      out.mask = hippo_control_msgs::msg::ControlTarget::IGNORE_ATTITUDE;
      break;
    default:
      out.mask = hippo_control_msgs::msg::ControlTarget::IGNORE_ATTITUDE_Y;
      break;
  }
}

void Traj::getAttitude(const double &t, const Eigen::Vector3d &vel,
                       Eigen::Quaterniond &att, bool upright_z) {
  // construct attitude that matches the direction of the velocity vector with
  // the y-axis of the endeffector
  Eigen::Vector3d tangential;
  getTangential(t, vel, tangential);

  double angle =
      acos(double(tangential.transpose() * Eigen::Vector3d::UnitY()));
  Eigen::Vector3d axis = Eigen::Vector3d::UnitY().cross(tangential);
  axis = axis / std::max(axis.norm(), eps_zero_div_);
  att = Eigen::AngleAxisd(angle, axis);
  if (upright_z) {
    Eigen::Vector3d y_axis = att.toRotationMatrix() * Eigen::Vector3d::UnitY();
    Eigen::Vector3d z_axis_des = y_axis.cross(Eigen::Vector3d::UnitZ().cross(
        y_axis));  // desired z-axis in world frame
    if (z_axis_des[2] < 0) {
      z_axis_des *= -1;
    }
    z_axis_des = att.toRotationMatrix().inverse() *
                 z_axis_des;  // transform in local trajectory frame
    // Eigen::Vector3d z_axis = att.toRotationMatrix() *
    // Eigen::Vector3d::UnitZ();
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    double angle_z = acos(double(z_axis_des.transpose() * z_axis));

    Eigen::Vector3d axis_z = z_axis.cross(z_axis_des);
    if (axis_z.norm() > 1e-6) {
      axis_z = axis_z / std::max(axis_z.norm(), eps_zero_div_);
      Eigen::Quaterniond att_z_upright =
          Eigen::Quaterniond(Eigen::AngleAxisd(angle_z, axis_z));
      att = att * att_z_upright;
    }
  }
  att = att * Eigen::AngleAxisd(rot_tangential_, Eigen::Vector3d::UnitY());
}

void Traj::getTangential(const double &t, const Eigen::Vector3d &vel,
                         Eigen::Vector3d &tangential) {
  double norm_vel = vel.norm();
  if (norm_vel > eps_zero_div_) {
    tangential = vel / vel.norm();
  } else {
    Eigen::Vector3d tmp_pos, tmp_vel;
    getTranslationalMotion(t + eps_fd_, tmp_pos, tmp_vel);
    int counter = 0;
    getTangentialRecursive(t + eps_fd_, tmp_vel, norm_vel, counter, tangential);
  }
  if (!got_last_tangential_) {
    got_last_tangential_ = true;
  } else if (last_tangential_.transpose() * tangential < 0) {
    tangential *= -1;
  }
  last_tangential_ = tangential;
}

void Traj::getTangentialRecursive(const double &t, const Eigen::Vector3d &vel,
                                  const double &initial_vel_norm, int &counter,
                                  Eigen::Vector3d &tangential) {
  double norm_vel = vel.norm();
  if (norm_vel > eps_zero_div_) {
    tangential = vel / vel.norm();
  } else if (counter > kmaxCounterRecursive) {
    if (!got_last_tangential_) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "No feasible tangent can be generated for trajectory!");
      tangential = Eigen::Vector3d::UnitY();
      return;
    }
    tangential = last_tangential_;
  } else {
    Eigen::Vector3d tmp_pos, tmp_vel;
    getTranslationalMotion(
        t + std::min(eps_fd_, 0.1 / double(kmaxCounterRecursive)), tmp_pos,
        tmp_vel);
    counter++;
    getTangentialRecursive(
        t + std::min(eps_fd_, 0.1 / double(kmaxCounterRecursive)), tmp_vel,
        initial_vel_norm, counter, tangential);
  }
}

void StraightLineTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  double motion_distance;
  ros_param_utils::getParam(node_ptr_, unidirectional_,
                            "straight_line.unidirectional", true, output);
  ros_param_utils::getParam(node_ptr_, motion_distance,
                            "straight_line.distance", 1.0, output);
  ros_param_utils::getParam(node_ptr_, forward_velocity_,
                            "straight_line.forward_velocity", 40.0, output);
  duration_ = motion_distance / forward_velocity_;
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "straight_line.startup_duration", 0.5, output);

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getStraightLineSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getStraightLineSetpoint(duration_, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void StraightLineTraj::getTranslationalMotion(const double &t,
                                              Eigen::Vector3d &pos,
                                              Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);
  } else if (t < start_trajectory_.getDuration() + duration_) {
    getStraightLineSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  } else {
    Eigen::Vector3d dummy_acc;
    end_trajectory_.getSetpoint(t - duration_ - start_trajectory_.getDuration(),
                                pos, vel, dummy_acc);
  }
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void StraightLineTraj::getStraightLineSetpoint(const double &t,
                                               Eigen::Vector3d &pos,
                                               Eigen::Vector3d &vel) {
  pos << 0.0, forward_velocity_ * t, 0.0;
  vel << 0.0, forward_velocity_, 0.0;
}

void StraightLineTraj::getStraightLineSetpoint(const double &t,
                                               Eigen::Vector3d &pos,
                                               Eigen::Vector3d &vel,
                                               Eigen::Vector3d &acc) {
  pos << 0.0, forward_velocity_ * t, 0.0;
  vel << 0.0, forward_velocity_, 0.0;
  acc << 0.0, 0.0, 0.0;
}

void Sinusoidal2DTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  double motion_w, motion_distance, motion_amp_oscillations;
  int motion_n_oscillations;
  ros_param_utils::getParam(node_ptr_, unidirectional_,
                            "sinusoidal2D.unidirectional", true, output);
  ros_param_utils::getParam(node_ptr_, motion_distance, "sinusoidal2D.distance",
                            1.0, output);
  ros_param_utils::getParam(node_ptr_, period_, "sinusoidal2D.period", 40.0,
                            output);
  motion_w = 2 * M_PI / period_;
  ros_param_utils::getParam(node_ptr_, motion_n_oscillations,
                            "sinusoidal2D.n_oscillations", 1, output);
  ros_param_utils::getParam(node_ptr_, motion_amp_oscillations,
                            "sinusoidal2D.amp_oscillations", 0.1, output);
  w_ = motion_w;
  amp_ = motion_distance / 2;
  w_oscillations_ = motion_n_oscillations * motion_w;
  amp_oscillations_ = motion_amp_oscillations;
}

void Sinusoidal2DTraj::getTranslationalMotion(const double &t,
                                              Eigen::Vector3d &pos,
                                              Eigen::Vector3d &vel) {
  // double test_w = w_ * 7;
  pos << 0.0, (-amp_ * cos(w_ * t) + amp_),
      amp_oscillations_ * cos(w_oscillations_ * t);
  vel << 0.0, amp_ * w_ * sin(w_ * t),
      -amp_oscillations_ * w_oscillations_ * sin(w_oscillations_ * t);
  // acc << 0.0, amp_ * std::pow(w_, 2) * cos(w_ * t),
  //         -amp_oscillations_ * std::pow(w_oscillations_, 2) *
  //         cos(w_oscillations_ * t);
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void SiTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  Eigen::Vector3d motion_offset;
  double motion_distance, motion_amp_oscillations;
  int motion_n_oscillations;
  ros_param_utils::getParam(node_ptr_, unidirectional_, "sinc.unidirectional",
                            true, output);
  ros_param_utils::getParam(node_ptr_, motion_distance, "sinc.distance", 1.0,
                            output);
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "sinc.startup_duration", 0.5, output);
  ros_param_utils::getParam(node_ptr_, T_, "sinc.period", 40.0, output);
  ros_param_utils::getParam(node_ptr_, motion_n_oscillations,
                            "sinc.n_oscillations", 1, output);
  ros_param_utils::getParam(node_ptr_, motion_amp_oscillations,
                            "sinc.amp_oscillations", 0.1, output);
  w_ = 2 * M_PI / T_;
  amp_ = motion_distance / 2;
  w_oscillations_ = motion_n_oscillations * w_;
  amp_oscillations_ = motion_amp_oscillations;

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getSiSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getSiSetpoint(T_ / 2, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void SiTraj::getSiFunc(const double &t, const double &delta_t, double &x,
                       double &dx) {
  double t_func = t + delta_t;
  x = sin(w_oscillations_ * t_func) /
      (std::max(abs(t_func), eps_zero_div_) * t_func /
       std::max(std::abs(t_func), eps_zero_div_));
  dx = (w_oscillations_ * t_func * cos(w_oscillations_ * t_func) -
        sin(w_oscillations_ * t_func)) /
       std::max(std::pow(t_func, 2), eps_zero_div_);
}

void SiTraj::getSiSetpoint(const double &t, Eigen::Vector3d &pos,
                           Eigen::Vector3d &vel) {
  double x_1, dx_1;  //, x_2, dx_2;
  getSiFunc(t, -T_ / 4.0, x_1, dx_1);
  // getSiFunc(t, -3 * T_ / 4.0, x_2, dx_2);

  pos << 0.0, -amp_ * cos(w_ * t) + amp_, amp_oscillations_ * (x_1);
  vel << 0.0, amp_ * w_ * sin(w_ * t), amp_oscillations_ * (dx_1);
}

void SiTraj::getSiFunc(const double &t, const double &delta_t, double &x,
                       double &dx, double &ddx) {
  double t_func = t + delta_t;
  x = sin(w_oscillations_ * t_func) /
      (std::max(abs(t_func), eps_zero_div_) * t_func /
       std::max(std::abs(t_func), eps_zero_div_));
  dx = (w_oscillations_ * t_func * cos(w_oscillations_ * t_func) -
        sin(w_oscillations_ * t_func)) /
       std::max(std::pow(t_func, 2), eps_zero_div_);
  ddx = -std::pow(w_oscillations_, 2) * sin(t_func * w_oscillations_) /
            (std::max(std::abs(t_func), eps_zero_div_) * t_func /
             std::max(std::abs(t_func), eps_zero_div_)) -
        2 * w_oscillations_ * cos(t_func * w_oscillations_) /
            std::max(std::pow(t_func, 2), eps_zero_div_) +
        2 * sin(t_func * w_oscillations_) /
            (std::max(std::abs(std::pow(t_func, 3)), eps_zero_div_) * t_func /
             std::max(std::abs(t_func), eps_zero_div_));
}

void SiTraj::getSiSetpoint(const double &t, Eigen::Vector3d &pos,
                           Eigen::Vector3d &vel, Eigen::Vector3d &acc) {
  double x_1, dx_1, ddx_1;  //, x_2, dx_2;
  getSiFunc(t, -T_ / 4.0, x_1, dx_1, ddx_1);
  // getSiFunc(t, -3 * T_ / 4.0, x_2, dx_2);

  pos << 0.0, -amp_ * cos(w_ * t) + amp_, amp_oscillations_ * (x_1);
  vel << 0.0, amp_ * w_ * sin(w_ * t), amp_oscillations_ * (dx_1);
  acc << 0.0, amp_ * std::pow(w_, 2) * cos(w_ * t), amp_oscillations_ * (ddx_1);
}

void SiTraj::getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                    Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);

  } else if (t < start_trajectory_.getDuration() + T_ / 2) {
    getSiSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  } else {
    Eigen::Vector3d dummy_acc;
    end_trajectory_.getSetpoint(t - T_ / 2 - start_trajectory_.getDuration(),
                                pos, vel, dummy_acc);
  }
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void FlowerTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  int motion_n_oscillations;
  ros_param_utils::getParam(node_ptr_, r_, "flower.radius", 1.0, output);
  ros_param_utils::getParam(node_ptr_, T_, "flower.period", 40.0, output);
  ros_param_utils::getParam(node_ptr_, motion_n_oscillations,
                            "flower.n_oscillations", 1, output);
  ros_param_utils::getParam(node_ptr_, unidirectional_, "flower.unidirectional",
                            false, output);
  ros_param_utils::getParam(node_ptr_, amp_oscillations_,
                            "flower.amp_oscillations", 0.1, output);
  ros_param_utils::getParam(node_ptr_, oscillations_radial_,
                            "flower.oscillations_radial", true, output);
  ros_param_utils::getParam(node_ptr_, n_turns_, "flower.n_turns", 1);
  w_ = 2 * M_PI / T_;
  w_oscillations_ = motion_n_oscillations * w_;
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "flower.startup_duration", 0.5, output);

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getFlowerSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getFlowerSetpoint(T_, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void FlowerTraj::getSetpoint(const double &t, EefTrajSetpoint &out) {
  getTranslationalMotion(t, out.pos, out.vel);
  out.acc.setZero();

  getAttitude(t, out.vel, out.att, true);
  Eigen::Vector3d pos_backward, vel_backward, acc_backward;
  double t_backward = t - eps_fd_;
  getTranslationalMotion(t_backward, pos_backward, vel_backward);
  Eigen::Quaterniond att_backward;
  getAttitude(t_backward, vel_backward, att_backward, true);

  Eigen::Vector3d pos_forward, vel_forward;
  double t_forward = t + eps_fd_;
  getTranslationalMotion(t_forward, pos_forward, vel_forward);
  Eigen::Quaterniond att_forward;
  getAttitude(t_forward, vel_forward, att_forward, true);
  Eigen::AngleAxisd datt_central =
      static_cast<Eigen::AngleAxisd>(att_backward.inverse() * att_forward);
  Eigen::Vector3d w_central = att_backward.toRotationMatrix() *
                              datt_central.angle() * datt_central.axis() /
                              (2 * eps_fd_);
  out.ang_vel = w_central;
}

void FlowerTraj::getFlowerSetpoint(const double &t, Eigen::Vector3d &pos,
                                   Eigen::Vector3d &vel) {
  if (oscillations_radial_) {
    double r_tmp = r_ + amp_oscillations_ * sin(w_oscillations_ * t);
    double dr_tmp =
        amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);

    pos << r_tmp * cos(w_ * t), r_tmp * sin(w_ * t), 0.0;
    vel << -r_tmp * w_ * sin(w_ * t) + dr_tmp * cos(w_ * t),
        r_tmp * w_ * cos(w_ * t) + dr_tmp * sin(w_ * t), 0.0;
  } else {
    pos << r_ * cos(w_ * t), r_ * sin(w_ * t),
        amp_oscillations_ * cos(w_oscillations_ * t);
    vel << -r_ * w_ * sin(w_ * t), r_ * w_ * cos(w_ * t),
        -amp_oscillations_ * w_oscillations_ * sin(w_oscillations_ * t);
  }
}

void FlowerTraj::getFlowerSetpoint(const double &t, Eigen::Vector3d &pos,
                                   Eigen::Vector3d &vel, Eigen::Vector3d &acc) {
  if (oscillations_radial_) {
    double r_tmp = r_ + amp_oscillations_ * sin(w_oscillations_ * t);
    double dr_tmp =
        amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);

    pos << r_tmp * cos(w_ * t), r_tmp * sin(w_ * t), 0.0;
    vel << -r_tmp * w_ * sin(w_ * t) + dr_tmp * cos(w_ * t),
        r_tmp * w_ * cos(w_ * t) + dr_tmp * sin(w_ * t), 0.0;
    acc << -2 * amp_oscillations_ * w_ * w_oscillations_ * sin(t * w_) *
                   cos(t * w_oscillations_) -
               amp_oscillations_ * std::pow(w_oscillations_, 2) *
                   sin(t * w_oscillations_) * cos(t * w_) -
               std::pow(w_, 2) *
                   (amp_oscillations_ * sin(t * w_oscillations_) + r_) *
                   cos(t * w_),
        2 * amp_oscillations_ * w_ * w_oscillations_ * cos(t * w_) *
                cos(t * w_oscillations_) -
            amp_oscillations_ * std::pow(w_oscillations_, 2) * sin(t * w_) *
                sin(t * w_oscillations_) -
            std::pow(w_, 2) *
                (amp_oscillations_ * sin(t * w_oscillations_) + r_) *
                sin(t * w_),
        0.0;

  } else {
    pos << r_ * cos(w_ * t), r_ * sin(w_ * t),
        amp_oscillations_ * cos(w_oscillations_ * t);
    vel << -r_ * w_ * sin(w_ * t), r_ * w_ * cos(w_ * t),
        -amp_oscillations_ * w_oscillations_ * sin(w_oscillations_ * t);
    acc << -r_ * std::pow(w_, 2) * cos(w_ * t),
        -r_ * std::pow(w_, 2) * sin(w_ * t),
        -amp_oscillations_ * std::pow(w_oscillations_, 2) *
            cos(w_oscillations_ * t);
  }
}

void FlowerTraj::getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                        Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);
  } else if (unidirectional_) {
    if (t < start_trajectory_.getDuration() + T_) {
      getFlowerSetpoint(t - start_trajectory_.getDuration(), pos, vel);
    } else {
      Eigen::Vector3d dummy_acc;
      end_trajectory_.getSetpoint(t - T_ - start_trajectory_.getDuration(), pos,
                                  vel, dummy_acc);
    }
  } else {
    getFlowerSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  }

  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void SinusTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  double motion_distance, motion_amp_oscillations;
  int motion_n_oscillations;
  ros_param_utils::getParam(node_ptr_, unidirectional_, "sinus.unidirectional",
                            true, output);
  ros_param_utils::getParam(node_ptr_, motion_distance, "sinus.distance", 1.0,
                            output);
  ros_param_utils::getParam(node_ptr_, forward_velocity_,
                            "sinus.forward_velocity", 40.0, output);
  duration_ = motion_distance / forward_velocity_;
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "sinus.startup_duration", 0.5, output);
  ros_param_utils::getParam(node_ptr_, motion_n_oscillations,
                            "sinus.n_oscillations", 1, output);
  ros_param_utils::getParam(node_ptr_, motion_amp_oscillations,
                            "sinus.amp_oscillations", 0.1, output);
  w_oscillations_ = motion_n_oscillations * 2 * M_PI / duration_;
  amp_oscillations_ = motion_amp_oscillations;

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getSinusSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getSinusSetpoint(duration_, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void SinusTraj::getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                       Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);
  } else if (t < start_trajectory_.getDuration() + duration_) {
    getSinusSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  } else {
    Eigen::Vector3d dummy_acc;
    end_trajectory_.getSetpoint(t - duration_ - start_trajectory_.getDuration(),
                                pos, vel, dummy_acc);
  }
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void SinusTraj::getSinusSetpoint(const double &t, Eigen::Vector3d &pos,
                                 Eigen::Vector3d &vel) {
  pos << 0.0, forward_velocity_ * t,
      amp_oscillations_ * sin(w_oscillations_ * t);
  vel << 0.0, forward_velocity_,
      amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);
}

void SinusTraj::getSinusSetpoint(const double &t, Eigen::Vector3d &pos,
                                 Eigen::Vector3d &vel, Eigen::Vector3d &acc) {
  pos << 0.0, forward_velocity_ * t,
      amp_oscillations_ * sin(w_oscillations_ * t);
  vel << 0.0, forward_velocity_,
      amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);
  acc << 0.0, 0.0,
      -amp_oscillations_ * std::pow(w_oscillations_, 2) *
          sin(w_oscillations_ * t);
}

void SpiralTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  double motion_distance, motion_amp_oscillations;
  int motion_n_oscillations;
  ros_param_utils::getParam(node_ptr_, unidirectional_, "spiral.unidirectional",
                            true, output);
  ros_param_utils::getParam(node_ptr_, motion_distance, "spiral.distance", 1.0,
                            output);
  ros_param_utils::getParam(node_ptr_, forward_velocity_,
                            "spiral.forward_velocity", 40.0, output);
  duration_ = motion_distance / forward_velocity_;
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "spiral.startup_duration", 0.5, output);
  ros_param_utils::getParam(node_ptr_, motion_n_oscillations,
                            "spiral.n_oscillations", 1, output);
  ros_param_utils::getParam(node_ptr_, motion_amp_oscillations,
                            "spiral.amp_oscillations", 0.1, output);
  w_oscillations_ = motion_n_oscillations * 2 * M_PI / duration_;
  amp_oscillations_ = motion_amp_oscillations;

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getSpiralSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getSpiralSetpoint(duration_, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void SpiralTraj::getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                        Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);
  } else if (unidirectional_) {
    if (t < start_trajectory_.getDuration() + duration_) {
      getSpiralSetpoint(t - start_trajectory_.getDuration(), pos, vel);
    } else {
      Eigen::Vector3d dummy_acc;
      end_trajectory_.getSetpoint(
          t - duration_ - start_trajectory_.getDuration(), pos, vel, dummy_acc);
    }
  } else {
    Eigen::Vector3d dummy_acc;
    getSpiralSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  }
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void SpiralTraj::getSpiralSetpoint(const double &t, Eigen::Vector3d &pos,
                                   Eigen::Vector3d &vel) {
  pos << amp_oscillations_ * cos(w_oscillations_ * t), forward_velocity_ * t,
      amp_oscillations_ * sin(w_oscillations_ * t);
  vel << -amp_oscillations_ * w_oscillations_ * sin(w_oscillations_ * t),
      forward_velocity_,
      amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);
}

void SpiralTraj::getSpiralSetpoint(const double &t, Eigen::Vector3d &pos,
                                   Eigen::Vector3d &vel, Eigen::Vector3d &acc) {
  pos << amp_oscillations_ * cos(w_oscillations_ * t), forward_velocity_ * t,
      amp_oscillations_ * sin(w_oscillations_ * t);
  vel << -amp_oscillations_ * w_oscillations_ * sin(w_oscillations_ * t),
      forward_velocity_,
      amp_oscillations_ * w_oscillations_ * cos(w_oscillations_ * t);
  acc << -amp_oscillations_ * std::pow(w_oscillations_, 2) *
             cos(w_oscillations_ * t),
      0.0,
      -amp_oscillations_ * std::pow(w_oscillations_, 2) *
          sin(w_oscillations_ * t);
}

void EightTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  declareStandardParams(output);
  ros_param_utils::getParam(node_ptr_, unidirectional_, "eight.unidirectional",
                            true, output);
  ros_param_utils::getParam(node_ptr_, amp_, "eight.amp", 0.3, output);
  ros_param_utils::getParam(node_ptr_, T_, "eight.period", 80.0, output);
  ros_param_utils::getParam(node_ptr_, startup_duration_,
                            "eight.startup_duration", 0.5, output);
  double t_offset_rel;
  ros_param_utils::getParam(node_ptr_, t_offset_rel, "eight.t_offset_rel", 0.25,
                            output);
  w_ = 2 * M_PI / T_;
  t_fix_ = t_offset_rel * 2 * M_PI / w_;

  // initialize start up trajectory:
  Eigen::Vector3d p_start, p_end, v_start, v_end, a_start, a_end;
  getEightSetpoint(0.0, p_end, v_end, a_end);
  v_start.setZero();
  a_start.setZero();
  start_trajectory_.initialize(p_end, v_start, v_end, a_start, a_end,
                               startup_duration_);

  // initialize end trajectory:
  getEightSetpoint(T_, p_start, v_start, a_start);
  v_end.setZero();
  a_end.setZero();
  end_trajectory_.initialize(p_start, v_start, v_end, a_start, a_end,
                             startup_duration_);
}

void EightTraj::getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                       Eigen::Vector3d &vel) {
  if (t < start_trajectory_.getDuration()) {
    Eigen::Vector3d dummy_acc;
    start_trajectory_.getSetpoint(t, pos, vel, dummy_acc);
  } else if (unidirectional_) {
    if (t < start_trajectory_.getDuration() + T_) {
      getEightSetpoint(t - start_trajectory_.getDuration(), pos, vel);
    } else {
      Eigen::Vector3d dummy_acc;
      end_trajectory_.getSetpoint(t - T_ - start_trajectory_.getDuration(), pos,
                                  vel, dummy_acc);
    }
  } else {
    Eigen::Vector3d dummy_acc;
    getEightSetpoint(t - start_trajectory_.getDuration(), pos, vel);
  }
  pos = rot_ * pos;
  vel = rot_ * vel;
  pos += offset_;
}

void EightTraj::getEightSetpoint(const double &t, Eigen::Vector3d &pos,
                                 Eigen::Vector3d &vel) {
  double t_eval = t + t_fix_;
  double denum = 1 + std::pow(sin(w_ * t_eval), 2);
  pos(0) = amp_ * sin(w_ * t_eval) * cos(w_ * t_eval) / denum;
  pos(1) = amp_ * cos(w_ * t_eval) / denum;
  pos(2) = 0.0;
  vel(0) = -amp_ * w_ * std::pow(sin(t_eval * w_), 2) /
               (std::pow(sin(t_eval * w_), 2) + 1) +
           amp_ * w_ * std::pow(cos(t_eval * w_), 2) /
               (std::pow(sin(t_eval * w_), 2) + 1) -
           2 * amp_ * w_ * std::pow(sin(t_eval * w_), 2) *
               std::pow(cos(t_eval * w_), 2) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 2);
  vel(1) = -amp_ * w_ * sin(t_eval * w_) / (std::pow(sin(t_eval * w_), 2) + 1) -
           2 * amp_ * w_ * sin(t_eval * w_) * std::pow(cos(t_eval * w_), 2) /
               std::pow((std::pow(sin(t_eval * w_), 2) + 1), 2);
  vel(2) = 0.0;
}

void EightTraj::getEightSetpoint(const double &t, Eigen::Vector3d &pos,
                                 Eigen::Vector3d &vel, Eigen::Vector3d &acc) {
  double t_eval = t + t_fix_;
  double denum = 1 + std::pow(sin(w_ * t_eval), 2);
  pos(0) = amp_ * sin(w_ * t_eval) * cos(w_ * t_eval) / denum;
  pos(1) = amp_ * cos(w_ * t_eval) / denum;
  pos(2) = 0.0;
  vel(0) = -amp_ * w_ * std::pow(sin(t_eval * w_), 2) /
               (std::pow(sin(t_eval * w_), 2) + 1) +
           amp_ * w_ * std::pow(cos(t_eval * w_), 2) /
               (std::pow(sin(t_eval * w_), 2) + 1) -
           2 * amp_ * w_ * std::pow(sin(t_eval * w_), 2) *
               std::pow(cos(t_eval * w_), 2) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 2);
  vel(1) = -amp_ * w_ * sin(t_eval * w_) / (std::pow(sin(t_eval * w_), 2) + 1) -
           2 * amp_ * w_ * sin(t_eval * w_) * std::pow(cos(t_eval * w_), 2) /
               std::pow((std::pow(sin(t_eval * w_), 2) + 1), 2);
  vel(2) = 0.0;
  acc(0) = -4 * amp_ * std::pow(w_, 2) * sin(t_eval * w_) * cos(t_eval * w_) /
               (std::pow(sin(t_eval * w_), 2) + 1) +
           6 * amp_ * std::pow(w_, 2) * std::pow(sin(t_eval * w_), 3) *
               cos(t_eval * w_) /
               std::pow((std::pow(sin(t_eval * w_), 2) + 1), 2) -
           6 * amp_ * std::pow(w_, 2) * sin(t_eval * w_) *
               std::pow(cos(t_eval * w_), 3) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 2) +
           8 * amp_ * std::pow(w_, 2) * std::pow(sin(t_eval * w_), 3) *
               std::pow(cos(t_eval * w_), 3) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 3);
  acc(1) = -amp_ * std::pow(w_, 2) * cos(t_eval * w_) /
               (std::pow(sin(t_eval * w_), 2) + 1) +
           6 * amp_ * std::pow(w_, 2) * std::pow(sin(t_eval * w_), 2) *
               cos(t_eval * w_) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 2) -
           2 * amp_ * std::pow(w_, 2) * std::pow(cos(t_eval * w_), 3) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 2) +
           8 * amp_ * std::pow(w_, 2) * std::pow(sin(t_eval * w_), 2) *
               std::pow(cos(t_eval * w_), 3) /
               std::pow(std::pow(sin(t_eval * w_), 2) + 1, 3);
  acc(2) = 0.0;
}

}  // namespace uvms_traj_gen
