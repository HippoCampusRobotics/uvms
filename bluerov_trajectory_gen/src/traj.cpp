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

#include "bluerov_trajectory_gen/traj.hpp"

namespace bluerov_traj_gen {

void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde) {
  x_tilde << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
}

void Traj::publishStatus(int status) {
  std_msgs::msg::Int64 msg;
  msg.data = status;
  status_pub_ptr_->publish(msg);
}

void Traj::getStartPosition(Eigen::Vector3d &pos) {
  TrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  pos = setpoint.pos;
}

void Traj::getStartOrientation(Eigen::Quaterniond &att) {
  TrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  att = setpoint.att;
}

void Traj::getStartPose(Eigen::Vector3d &pos, Eigen::Quaterniond &att) {
  TrajSetpoint setpoint;
  getSetpoint(0.0, setpoint);
  pos = setpoint.pos;
  att = setpoint.att;
}

void SingleDOFVelTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  std::string name_prefix = "single_dof.";
  if (!ros_param_utils::getParam(node_ptr_, idx_, name_prefix + "idx", 1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param eight width not set!");
    return;
  }
  std::string prefix;
  switch (idx_) {
    case 0:
      prefix = "pos_x.";
      break;
    case 1:
      prefix = "pos_y.";
      break;
    case 2:
      prefix = "pos_z.";
      break;
    case 3:
      prefix = "att_x.";
      break;
    case 4:
      prefix = "att_y.";
      break;
    case 5:
      prefix = "att_z.";
      break;
  }
  std::vector<double> start_pos;
  if (!ros_param_utils::getParamArray(node_ptr_, start_pos,
                                      name_prefix + prefix + "start_pos",
                                      {1.0, 1.5, -0.5})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_pos not set!");
    return;
  }
  if (start_pos.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_pos parameter has size " +
                  std::to_string(start_pos.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  pos_des_ = Eigen::Vector3d(start_pos.data());

  std::vector<double> start_att;
  if (!ros_param_utils::getParamArray(node_ptr_, start_att,
                                      name_prefix + prefix + "start_att",
                                      {0.0, 0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_att not set!");
    return;
  }
  if (start_att.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_att parameter has size " +
                  std::to_string(start_att.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  att_des_ = Eigen::AngleAxisd(start_att[2] * M_PI, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(start_att[1] * M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(start_att[0] * M_PI, Eigen::Vector3d::UnitX());

  std::vector<double> limits;
  if (!ros_param_utils::getParamArray(
          node_ptr_, limits, name_prefix + prefix + "limits", {0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param limits not set!");
    return;
  }
  std::copy(limits.begin(), limits.end(), limits_.begin());
  if (idx_ > 2) {
    limits_[0] = M_PI * limits_[0];
    limits_[1] = M_PI * limits_[1];
  }

  if (!ros_param_utils::getParam(node_ptr_, vel_, name_prefix + prefix + "vel",
                                 4.0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param vel not set!");
    return;
  }
}

void SingleDOFVelTraj::getSetpoint(const double &t, TrajSetpoint &out) {
  out.pos = pos_des_;
  out.att = att_des_;
  out.vel.setZero();
  out.ang_vel.setZero();
  out.acc.setZero();
  out.ang_acc.setZero();
  if (t == 0.0) {  // start pose is desired, return initial point
    return;
  }

  if (idx_ <
      2) {  // x- and y-direction of BlueROV are both tested along global y-axis
    if (limits_[0] < (*pos_ptr_)(1) && (*pos_ptr_)(1) < limits_[1]) {
      out.vel(1) = vel_;
      return;
    }
    if ((*pos_ptr_)(1) >= limits_[1]) {
      out.pos(1) = limits_[1];
      out.vel(1) = 0.0;
    } else if ((*pos_ptr_)(1) <= limits_[0]) {
      out.pos(1) = limits_[0];
      out.vel(1) = 0.0;
    }
  } else if (idx_ == 2) {
    if (limits_[0] < (*pos_ptr_)(idx_) && (*pos_ptr_)(idx_) < limits_[1]) {
      out.vel(idx_) = vel_;
      return;
    }
    if ((*pos_ptr_)(idx_) >= limits_[1]) {
      out.pos(idx_) = limits_[1];
      out.vel(idx_) = 0.0;
    } else if ((*pos_ptr_)(idx_) <= limits_[0]) {
      out.pos(idx_) = limits_[0];
      out.vel(idx_) = 0.0;
    }
  } else if (idx_ < 6) {
    Eigen::Vector3d rpy;
    rot_utils::quat_to_rpy(*att_ptr_, rpy);

    if (limits_[0] < rpy(idx_ - 3) && rpy(idx_ - 3) < limits_[1]) {
      out.ang_vel(idx_ - 3) = vel_;
      return;
    }
    if (rpy(idx_ - 3) >= limits_[1]) {
      Eigen::Vector3d rpy_des;
      rot_utils::quat_to_rpy(*att_ptr_, rpy_des);
      out.att.setIdentity();
      for (int i = 2; i >= 0; i--) {
        out.att =
            out.att * Eigen::AngleAxis(int(i == idx_ - 3) * limits_[1] +
                                           int(i != idx_ - 3) * rpy_des(i),
                                       Eigen::Vector3d::Unit(i));
      }
      out.ang_vel(idx_ - 3) = 0.0;
    } else if (rpy(idx_ - 3) <= limits_[0]) {
      Eigen::Vector3d rpy_des;
      rot_utils::quat_to_rpy(*att_ptr_, rpy_des);
      out.att.setIdentity();
      for (int i = 2; i >= 0; i--) {
        out.att =
            out.att * Eigen::AngleAxis(int(i == idx_ - 3) * limits_[0] +
                                           int(i != idx_ - 3) * rpy_des(i),
                                       Eigen::Vector3d::Unit(i));
      }
      out.ang_vel(idx_ - 3) = 0.0;
    }
  } else {
    std::cerr << "Velocity index " << idx_ << " of trajectory out of range!"
              << std::endl;
    return;
  }
  // reached the end of the trajectory, reset and move back to start
  *started_loop_ptr_ = false;
  *gen_start_traj_ptr_ = true;
}

void SingleDOFVelTraj::getSetpointMsg(const double &t,
                                      hippo_msgs::msg::ControlTarget &out) {
  TrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
  if (!*started_loop_ptr_) {
    out.mask = 0;
    return;
  }
  switch (idx_) {
    case 0:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_POSITION_Y;
      break;
    case 1:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_POSITION_Y;
      break;
    case 2:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_POSITION_Z;
      break;
    case 3:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_ATTITUDE_X;
      break;
    case 4:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_ATTITUDE_Y;
      break;
    case 5:
      out.mask = hippo_msgs::msg::ControlTarget::IGNORE_ATTITUDE_Z;
      break;
    default:
      std::cerr << "Index " << idx_ << " is out of range for ignoring state"
                << std::endl;
  }
}

void SingleDOFSetpoints::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  idx_setpoint_ = 0;
  std::string name_prefix = "single_dof_setpoints.";
  if (!ros_param_utils::getParam(node_ptr_, idx_, name_prefix + "idx", 0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param idx not set!");
    return;
  }
  std::string prefix;
  switch (idx_) {
    case 0:
      prefix = "att_x.";
      break;
    case 1:
      prefix = "att_y.";
      break;
  }
  std::vector<double> start_pos;
  if (!ros_param_utils::getParamArray(node_ptr_, start_pos,
                                      name_prefix + prefix + "start_pos",
                                      {1.0, 1.5, -0.5})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_pos not set!");
    return;
  }
  if (start_pos.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_pos parameter has size " +
                  std::to_string(start_pos.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  pos_des_ = Eigen::Vector3d(start_pos.data());

  std::vector<double> limits;
  if (!ros_param_utils::getParamArray(
          node_ptr_, limits, name_prefix + prefix + "limits", {0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param limits not set!");
    return;
  }
  std::copy(limits.begin(), limits.end(), limits_.begin());
  limits_[0] = M_PI * limits_[0];
  limits_[1] = M_PI * limits_[1];

  att_des_ = Eigen::AngleAxisd(limits_[0], Eigen::Vector3d::Unit(idx_));

  if (!ros_param_utils::getParam(node_ptr_, n_setpoints_,
                                 name_prefix + "n_setpoints", 5)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param n_setpoints not set!");
    return;
  }

  setpoints_.resize(n_setpoints_);
  double interval = (limits_[1] - limits_[0]) / (n_setpoints_ - 1);
  double min = limits_[0];
  std::generate(
      setpoints_.begin(), setpoints_.end(),
      [n = 0, &interval, &min]() mutable { return n++ * interval + min; });

  if (!ros_param_utils::getParam(node_ptr_, t_setpoint_,
                                 name_prefix + "t_setpoint", 5.0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param n_setpoints not set!");
    return;
  }
  if (!ros_param_utils::getParam(node_ptr_, accuracy_, name_prefix + "accuracy",
                                 0.01)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param accuracy not set!");
    return;
  }
}

void SingleDOFSetpoints::getSetpoint(const double &t, TrajSetpoint &out) {
  out.pos = pos_des_;
  out.att = att_des_;
  out.vel.setZero();
  out.ang_vel.setZero();
  out.acc.setZero();
  out.ang_acc.setZero();
  if (t == 0.0) {  // start pose is desired, return initial point
    return;
  }

  Eigen::Vector3d rpy;
  rot_utils::quat_to_rpy(*att_ptr_, rpy);
  if (!keep_pose_ &&
      std::abs(setpoints_[idx_setpoint_] - rpy(idx_)) < accuracy_) {
    t_reached_ = t;
    keep_pose_ = true;
    publishStatus(idx_setpoint_);
  }

  if (keep_pose_ && t - t_reached_ >= t_setpoint_) {
    if (idx_setpoint_ < n_setpoints_ - 1) {
      keep_pose_ = false;
      idx_setpoint_++;
    }
  }
  out.att = Eigen::Quaterniond(Eigen::AngleAxisd(setpoints_[idx_setpoint_],
                                                 Eigen::Vector3d::Unit(idx_)));
  // out.ang_vel(idx_) = setpoints_[idx_setpoint_] - rpy(idx_);
}

void SingleDOFSetpoints::getSetpointMsg(const double &t,
                                        hippo_msgs::msg::ControlTarget &out) {
  TrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
}

void SingleDOFSinusoidal::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  std::string name_prefix = "single_dof_sinusoidal.";
  if (!ros_param_utils::getParam(node_ptr_, idx_, name_prefix + "idx", 1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param eight width not set!");
    return;
  }
  std::string prefix;
  switch (idx_) {
    case 0:
      prefix = "pos_x.";
      break;
    case 1:
      prefix = "pos_y.";
      break;
    case 2:
      prefix = "pos_z.";
      break;
    case 3:
      prefix = "att_x.";
      break;
    case 4:
      prefix = "att_y.";
      break;
    case 5:
      prefix = "att_z.";
      break;
  }
  std::vector<double> start_pos;
  if (!ros_param_utils::getParamArray(node_ptr_, start_pos,
                                      name_prefix + prefix + "start_pos",
                                      {1.0, 1.5, -0.5})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_pos not set!");
    return;
  }
  if (start_pos.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_pos parameter has size " +
                  std::to_string(start_pos.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  pos_des_ = Eigen::Vector3d(start_pos.data());

  std::vector<double> start_att;
  if (!ros_param_utils::getParamArray(node_ptr_, start_att,
                                      name_prefix + prefix + "start_att",
                                      {0.0, 0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_att not set!");
    return;
  }
  if (start_att.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_att parameter has size " +
                  std::to_string(start_att.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  att_des_ = Eigen::AngleAxisd(start_att[2] * M_PI, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(start_att[1] * M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(start_att[0] * M_PI, Eigen::Vector3d::UnitX());

  if (!ros_param_utils::getParam(node_ptr_, amplitude_,
                                 name_prefix + prefix + "amplitude", 0.1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param amplitude not set!");
    return;
  }

  if (idx_ > 2) {
    amplitude_ = M_PI * amplitude_;
  }

  if (!ros_param_utils::getParam(node_ptr_, omega_,
                                 name_prefix + prefix + "omega", 0.1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param omega not set!");
    return;
  }
}

void SingleDOFSinusoidal::getSetpoint(const double &t, TrajSetpoint &out) {
  out.pos = pos_des_;
  out.att = att_des_;
  out.vel.setZero();
  out.ang_vel.setZero();
  out.acc.setZero();
  out.ang_acc.setZero();
  if (t == 0.0) {  // start pose is desired, return initial point
    return;
  }

  if (idx_ <
      2) {  // x- and y-direction of BlueROV are both tested along global y-axis
    out.pos(1) += amplitude_ * sin(omega_ * t);
    out.vel(1) = amplitude_ * omega_ * cos(omega_ * t);
    out.acc(1) = amplitude_ * std::pow(omega_, 3) * sin(omega_ * t);
    return;
  } else if (idx_ == 2) {
    out.pos(idx_) += amplitude_ * sin(omega_ * t);
    out.vel(idx_) = amplitude_ * omega_ * cos(omega_ * t);
    out.acc(idx_) = -amplitude_ * std::pow(omega_, 2) * sin(omega_ * t);
    return;
  } else if (idx_ < 6) {
    Eigen::Vector3d rpy;
    rot_utils::quat_to_rpy(*att_ptr_, rpy);
    out.att = out.att * Eigen::AngleAxis(amplitude_ * sin(omega_ * t),
                                         Eigen::Vector3d::Unit(idx_ - 3));
    out.ang_vel(idx_ - 3) = amplitude_ * omega_ * cos(omega_ * t);
    out.ang_acc(idx_ - 3) = -amplitude_ * std::pow(omega_, 2) * sin(omega_ * t);
  } else {
    std::cerr << "Index " << idx_ << " of trajectory out of range!"
              << std::endl;
    return;
  }
}

void SingleDOFSinusoidal::getSetpointMsg(const double &t,
                                         hippo_msgs::msg::ControlTarget &out) {
  TrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
}

void StationKeeping::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  std::string name_prefix = "station_keeping.";
  std::vector<double> start_pos;
  if (!ros_param_utils::getParamArray(
          node_ptr_, start_pos, name_prefix + "start_pos", {1.0, 1.5, -0.5})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_pos not set!");
    return;
  }
  if (start_pos.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_pos parameter has size " +
                  std::to_string(start_pos.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  pos_des_ = Eigen::Vector3d(start_pos.data());
  std::vector<double> start_att;
  if (!ros_param_utils::getParamArray(
          node_ptr_, start_att, name_prefix + "start_att", {0.0, 0.0, 0.0})) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param start_att not set!");
    return;
  }
  if (start_att.size() != 3) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s",
                 ("start_att parameter has size " +
                  std::to_string(start_att.size()) + " but must be equal to 3")
                     .c_str());
    return;
  }
  att_des_ = Eigen::AngleAxisd(start_att[2] * M_PI, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(start_att[1] * M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(start_att[0] * M_PI, Eigen::Vector3d::UnitX());
}

void StationKeeping::getSetpoint(const double &t, TrajSetpoint &out) {
  out.pos = pos_des_;
  out.att = att_des_;
  out.vel.setZero();
  out.ang_vel.setZero();
  out.acc.setZero();
  out.ang_acc.setZero();
  if (t == 0.0) {  // start pose is desired, return initial point
    publishStatus(0);
    return;
  }
  publishStatus(1);
}

void StationKeeping::getSetpointMsg(const double &t,
                                    hippo_msgs::msg::ControlTarget &out) {
  TrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
}

void EightTraj::initialize(rclcpp::Node *node_ptr, bool output) {
  node_ptr_ = node_ptr;
  std::string name_prefix = "eight.";
  if (!ros_param_utils::getParam(node_ptr_, a_, name_prefix + "a", 0.1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param eight width not set!");
    return;
  }
  double f;
  if (!ros_param_utils::getParam(node_ptr_, f, name_prefix + "f", 0.1)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param eight frequency not set!");
    return;
  }

  if (!ros_param_utils::getParam(node_ptr_, x_width_, name_prefix + "x_width",
                                 2.0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param tank x-width not set!");
    return;
  }

  if (!ros_param_utils::getParam(node_ptr_, y_width_, name_prefix + "y_width",
                                 4.0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param tank y-width not set!");
    return;
  }

  if (!ros_param_utils::getParam(node_ptr_, height_, name_prefix + "height",
                                 -0.5)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Param desired height not set!");
    return;
  }

  if (!ros_param_utils::getParam(node_ptr_, max_roll_, name_prefix + "max_roll",
                                 0.0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "Param accuracy for positioning not set!");
    return;
  }

  omega_des_ = 2 * M_PI * f;
}

void EightTraj::getSetpoint(const double &t, TrajSetpoint &out) {
  omega_ = omega_des_ * std::min(std::max(0.01, t), 1.0);
  t_fix_ = 0.25 * 2 * M_PI / omega_;
  const double t_input = t + t_fix_;
  getEightPos(out.pos, t_input);
  getEightVel(out.vel, t_input);
  getEightAtt(out.att, t_input);
  getEightRates(out.ang_vel, t_input);
}

void EightTraj::getSetpointMsg(const double &t,
                               hippo_msgs::msg::ControlTarget &out) {
  TrajSetpoint setpoint;
  getSetpoint(t, setpoint);
  out.header.stamp = node_ptr_->now();
  out.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(setpoint.pos, out.position);
  hippo_common::convert::EigenToRos(setpoint.vel, out.velocity);
  hippo_common::convert::EigenToRos(setpoint.acc, out.acceleration);
  hippo_common::convert::EigenToRos(setpoint.att, out.attitude);
  hippo_common::convert::EigenToRos(setpoint.ang_vel, out.angular_velocity);
  hippo_common::convert::EigenToRos(setpoint.ang_acc, out.angular_acceleration);
  out.mask = 0;
}

void EightTraj::getEightPos(Eigen::Vector3d &pos, const double &t) {
  double denum = 1 + std::pow(sin(omega_ * t), 2);
  pos(0) = x_width_ * 0.5 + a_ * sin(omega_ * t) * cos(omega_ * t) / denum;
  pos(1) = y_width_ * 0.5 + a_ * cos(omega_ * t) / denum;
  pos(2) = height_;
}

void EightTraj::getEightVel(Eigen::Vector3d &vel, const double &t) {
  double denum2 = std::pow(1 + std::pow(sin(omega_ * t), 2), 2);
  vel(0) = a_ * omega_ * (1 - 3 * std::pow(sin(omega_ * t), 2)) / denum2;
  vel(1) = a_ * omega_ * sin(omega_ * t) * (std::pow(sin(omega_ * t), 2) - 3) /
           denum2;
  vel(2) = 0.0;
}

void EightTraj::getEightAtt(Eigen::Quaterniond &att, const double &t) {
  double roll, pitch, yaw;
  getEightRPY(roll, pitch, yaw, t);
  att = hippo_common::tf2_utils::EulerToQuaternion(roll, pitch, yaw);
}

void EightTraj::getEightRPY(double &roll, double &pitch, double &yaw,
                            const double &t) {
  getEightYaw(yaw, t);
  roll = max_roll_ * cos(omega_ * t);
  pitch = 0;
}

void EightTraj::getEightYaw(double &yaw, const double &t) {
  Eigen::Vector3d vel;
  getEightVel(vel, t);
  yaw = atan2(vel.y(), vel.x());
}

void EightTraj::getEightRates(Eigen::Vector3d &rates, const double &t) {
  double eps = 1e-8;
  double t_in, yaw_next, yaw_prev;
  t_in = t + eps;
  getEightYaw(yaw_next, t_in);
  t_in = t - eps;
  getEightYaw(yaw_prev, t_in);
  Eigen::Vector3d deuler_param;
  deuler_param << -omega_ * max_roll_ * sin(omega_ * t), 0.0,
      (yaw_next - yaw_prev) / (2 * eps);

  double r, p, y;
  getEightRPY(r, p, y, t);

  Eigen::Matrix3d J_rot;
  J_rot << 1, 0, -sin(p), 0, cos(r), cos(p) * sin(r), 0, -sin(r),
      cos(p) * cos(r);
  rates = J_rot * deuler_param;
}

}  // namespace bluerov_traj_gen
