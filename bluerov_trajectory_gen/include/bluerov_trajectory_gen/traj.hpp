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

#ifndef UVMS_TRAJECTORY_GEN_TRAJ_HPP
#define UVMS_TRAJECTORY_GEN_TRAJ_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "uvms_common/param_utils.hpp"
#include "uvms_common/ros_param_utils.hpp"
#include "uvms_common/rotation_utils.hpp"

namespace bluerov_traj_gen {
struct TrajSetpoint {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Quaterniond att;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d ang_acc;
};

enum TrajectoryType {
  eight = 1,
  single_DOF = 2,
  single_DOF_setpoints = 3,
  single_DOF_sinusoidal = 4,
  station_keeping = 5,
  station_keeping_coupled = 6,
};

void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde);

class Traj {
 public:
  Traj() = default;

  virtual ~Traj() = default;

  virtual void initialize(rclcpp::Node *node_ptr, bool output) = 0;

  void addStatusPublisher(
      rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_ptr) {
    status_pub_ptr_ = status_pub_ptr;
  }

  void getStartPosition(Eigen::Vector3d &pos);

  void getStartOrientation(Eigen::Quaterniond &att);

  void getStartPose(Eigen::Vector3d &pos, Eigen::Quaterniond &att);

  virtual void getSetpoint(const double &t, TrajSetpoint &out) = 0;

  virtual void getSetpointMsg(const double &t,
                              hippo_msgs::msg::ControlTarget &out) = 0;

 protected:
  void publishStatus(int status);
  Eigen::Vector3d offset_;  //!< initial offset of the whole trajectory
  rclcpp::Node *node_ptr_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_ptr_;
};

class SingleDOFVelTraj : public Traj {
 public:
  SingleDOFVelTraj() = default;

  void initialize(rclcpp::Node *node_ptr, bool output) override;
  void getSetpoint(const double &t, TrajSetpoint &out) override;
  void getSetpointMsg(const double &t,
                      hippo_msgs::msg::ControlTarget &out) override;
  void setPositionPtr(Eigen::Vector3d *pos_ptr) { pos_ptr_ = pos_ptr; }
  void setAttitudePtr(Eigen::Quaterniond *att_ptr) { att_ptr_ = att_ptr; }
  void setStartedLoopFlagPtr(bool *started_loop) {
    started_loop_ptr_ = started_loop;
  }
  void setGenStartTrajFlagPtr(bool *gen_start_traj) {
    gen_start_traj_ptr_ = gen_start_traj;
  }

 private:
  double vel_;              //!< velocity target for actuated DOF
  int idx_;                 //!< idx of actuated DOF
  bool *started_loop_ptr_;  //!< pointer to the flag of the trajectory node, can
                            //!< be set to falsed when limit is reached to drive
                            //!< back to start
  bool *gen_start_traj_ptr_;  //!< pointer to the flag of the trajectory node,
                              //!< can be set to falsed when limit is reached to
                              //!< drive back to start
  std::array<double, 2> limits_;
  Eigen::Vector3d pos_des_;
  Eigen::Quaterniond att_des_;
  Eigen::Vector3d *pos_ptr_;     //!< pointer to current position of vehicle
  Eigen::Quaterniond *att_ptr_;  //!< pointer to current attitude of vehicle
};

class SingleDOFSetpoints : public Traj {
 public:
  SingleDOFSetpoints() = default;

  void initialize(rclcpp::Node *node_ptr, bool output) override;
  void getSetpoint(const double &t, TrajSetpoint &out) override;
  void getSetpointMsg(const double &t,
                      hippo_msgs::msg::ControlTarget &out) override;
  void setPositionPtr(Eigen::Vector3d *pos_ptr) { pos_ptr_ = pos_ptr; }
  void setAttitudePtr(Eigen::Quaterniond *att_ptr) { att_ptr_ = att_ptr; }

 private:
  bool keep_pose_ = false;  //!< decides if pose should currently be held
  double t_setpoint_;       //!< time how long a setpoint is kept constant
  double t_reached_;        //!< time step on which the setpoint is reached
  int idx_setpoint_;
  double accuracy_;                //!< accuracy for reaching a setpoint
  int idx_;                        //!< idx of actuated DOF
  int n_setpoints_;                //!< number of points used
  std::vector<double> setpoints_;  //!< setpoints to be used
  Eigen::Vector3d pos_des_;
  std::array<double, 2> limits_;
  Eigen::Quaterniond att_des_;
  Eigen::Vector3d *pos_ptr_;     //!< pointer to current position of vehicle
  Eigen::Quaterniond *att_ptr_;  //!< pointer to current attitude of vehicle
};

class SingleDOFSinusoidal : public Traj {
 public:
  SingleDOFSinusoidal() = default;

  void initialize(rclcpp::Node *node_ptr, bool output) override;
  void getSetpoint(const double &t, TrajSetpoint &out) override;
  void getSetpointMsg(const double &t,
                      hippo_msgs::msg::ControlTarget &out) override;
  void setPositionPtr(Eigen::Vector3d *pos_ptr) { pos_ptr_ = pos_ptr; }
  void setAttitudePtr(Eigen::Quaterniond *att_ptr) { att_ptr_ = att_ptr; }

 private:
  int idx_;  //!< idx of actuated DOF
  double amplitude_;
  double omega_;
  std::vector<double> setpoints_;  //!< setpoints to be used
  Eigen::Vector3d pos_des_;
  Eigen::Quaterniond att_des_;
  Eigen::Vector3d *pos_ptr_;     //!< pointer to current position of vehicle
  Eigen::Quaterniond *att_ptr_;  //!< pointer to current attitude of vehicle
};

class StationKeeping : public Traj {
 public:
  StationKeeping() = default;

  void initialize(rclcpp::Node *node_ptr, bool output) override;
  void getSetpoint(const double &t, TrajSetpoint &out) override;
  void getSetpointMsg(const double &t,
                      hippo_msgs::msg::ControlTarget &out) override;
  void setPositionPtr(Eigen::Vector3d *pos_ptr) { pos_ptr_ = pos_ptr; }
  void setAttitudePtr(Eigen::Quaterniond *att_ptr) { att_ptr_ = att_ptr; }

 private:
  Eigen::Vector3d pos_des_;
  Eigen::Quaterniond att_des_;
  Eigen::Vector3d *pos_ptr_;     //!< pointer to current position of vehicle
  Eigen::Quaterniond *att_ptr_;  //!< pointer to current attitude of vehicle
};

class EightTraj : public Traj {
 public:
  EightTraj() = default;

  void initialize(rclcpp::Node *node_ptr, bool output) override;

  void getSetpoint(const double &t, TrajSetpoint &out) override;

  void getSetpointMsg(const double &t,
                      hippo_msgs::msg::ControlTarget &out) override;

 private:
  void getEightPos(Eigen::Vector3d &pos, const double &t);

  void getEightVel(Eigen::Vector3d &vel, const double &t);

  void getEightAtt(Eigen::Quaterniond &att, const double &t);

  void getEightRPY(double &roll, double &pitch, double &yaw, const double &t);

  void getEightYaw(double &yaw, const double &t);

  void getEightRates(Eigen::Vector3d &rates, const double &t);

  double a_;
  double x_width_;
  double y_width_;
  double height_;
  double omega_;
  double omega_des_;
  double max_roll_;
  double t_fix_;
};

}  // namespace bluerov_traj_gen

#endif  // ALPHA_TRAJECTORY_GEN_TRAJ_HPP
