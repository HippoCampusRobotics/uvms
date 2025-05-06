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
#include <eigen3/Eigen/Dense>

#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_control_msgs/msg/control_target.hpp"
#include "uvms_common/param_utils.hpp"
#include "uvms_common/pose_to_pose_trajectory.hpp"
#include "uvms_common/ros_param_utils.hpp"

namespace uvms_traj_gen {
enum TrajStatus {
  undeclared = -1,
  approaching_initial_auv_pose = 1,
  reached_initial_auv_pose = 2,
  approaching_initial_manipulator_pose = 3,
  reached_initial_pose = 4,
  approaching_initial_eef_pose = 5,
  reached_initial_eef_pose = 6,
  adapt_task_dimension = 7,
  start_motion = 8,
  finished_motion = 9,
  wait_for_restart = 10,
  finished_run = 11,
  waiting_for_goal = 12,
  approaching_goal = 13,
  reached_goal = 14,
  waiting_for_planner = 15,
};

enum TrajMode {
  undeclared_mode = -1,
  keep_eef_pose = 1,
  new_eff_trajectory = 2,
  go_to_starting_position = 3,
  stop = 4,
  received_status_reached_goal = 5,
};


// enum TrajStatusPaP {
//   undeclared = -1,
//   approaching_initial_auv_pose = 1,
//   reached_initial_auv_pose = 2,
//   approaching_initial_manipulator_pose = 3,
//   reached_initial_pose = 4,

//   waiting_for_goal = 5,
//   approaching_object = 6,
//   reched_object = 7,

//   finished_run = 11,
// };

struct EefTrajSetpoint {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Quaterniond att;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d ang_acc;
};

enum TrajectoryType {
  straight = 1,
  sinusoidal2D = 2,
  si = 3,
  flower = 4,
  sinus = 5,
  spiral = 6,
  eight = 7,
};

constexpr int kmaxCounterRecursive = 100;

const std::unordered_map<int, std::string> parameter_prefixes = {
    {TrajectoryType::straight, "straight_line."},
    {TrajectoryType::sinusoidal2D, "sinusoidal2D."},
    {TrajectoryType::si, "sinc."},
    {TrajectoryType::flower, "flower."},
    {TrajectoryType::sinus, "sinus."},
    {TrajectoryType::spiral, "spiral."},
    {TrajectoryType::eight, "eight."}};

void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde);

class Traj {
 public:
  Traj() = default;

  virtual ~Traj() = default;

  void addStatusPublisher(
      rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_ptr) {
    status_pub_ptr_ = status_pub_ptr;
  }

  void addStatusPtr(int *status_ptr) { status_ptr_ = status_ptr; }

  virtual bool isFinished(const double &t) = 0;

  virtual int trajType() = 0;

  void publishStatus(int status);

  virtual void initialize(rclcpp::Node *node_ptr, bool output) = 0;

  void getStartPosition(Eigen::Vector3d &pos);

  void getStartOrientation(Eigen::Quaterniond &att);

  void getStartPose(Eigen::Vector3d &pos, Eigen::Quaterniond &att);

  virtual void getSetpoint(const double &t, EefTrajSetpoint &out);

  void getSetpointMsg(const double &t,
                      hippo_control_msgs::msg::ControlTarget &out);

 protected:
  void declareStandardParams(bool output);
  void getAttitude(const double &t, const Eigen::Vector3d &vel,
                   Eigen::Quaterniond &att, bool upright_z);
  virtual void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                                      Eigen::Vector3d &vel) = 0;
  virtual void getTangential(const double &t, const Eigen::Vector3d &vel,
                             Eigen::Vector3d &tangential);
  virtual void getTangentialRecursive(const double &t,
                                      const Eigen::Vector3d &vel,
                                      const double &initial_vel_norm,
                                      int &counter,
                                      Eigen::Vector3d &tangential);
  Eigen::Vector3d offset_;  //!< initial offset of the whole trajectory
  Eigen::Matrix3d rot_;  //!< rotation matrix for rotating the whole trajectory
  double rot_tangential_;  //!< rotation around tangential axis
  rclcpp::Node *node_ptr_;
  const double eps_fd_ = 1e-4;  //!< step size for finite differences
  const double eps_zero_div_ = 1e-8;
  int *status_ptr_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_ptr_;
  int att_dofs_tracked_ = 1;
  Eigen::Vector3d last_tangential_;
  bool got_last_tangential_;
};

class StraightLineTraj : public Traj {
 public:
  StraightLineTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= start_trajectory_.getDuration() + duration_ +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }

  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;
  void getStraightLineSetpoint(const double &t, Eigen::Vector3d &pos,
                               Eigen::Vector3d &vel);
  void getStraightLineSetpoint(const double &t, Eigen::Vector3d &pos,
                               Eigen::Vector3d &vel, Eigen::Vector3d &acc);

  int type_ = TrajectoryType::straight;
  double duration_;
  double forward_velocity_;
  double startup_duration_;
  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
  bool unidirectional_ = true;
};

class Sinusoidal2DTraj : public Traj {
 public:
  Sinusoidal2DTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= period_ / 2) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }
  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;

  int type_ = TrajectoryType::sinusoidal2D;
  double w_;  //!< frequency of slow motion along y-axis
  double period_;
  double amp_;
  double w_oscillations_;  //!< frequency of motions perpendicular to movement
                           //!< direction
  double amp_oscillations_;
  bool unidirectional_ = true;
};

class SiTraj : public Traj {
 public:
  SiTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= T_ / 2 + start_trajectory_.getDuration() +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }

  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getSiFunc(const double &t, const double &delta_t, double &x, double &dx);
  void getSiSetpoint(const double &t, Eigen::Vector3d &pos,
                     Eigen::Vector3d &vel);
  void getSiFunc(const double &t, const double &delta_t, double &x, double &dx,
                 double &ddx);
  void getSiSetpoint(const double &t, Eigen::Vector3d &pos,
                     Eigen::Vector3d &vel, Eigen::Vector3d &acc);
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;

  int type_ = TrajectoryType::si;
  double w_;  //!< frequency of slow motion along y-axis
  double T_;
  double amp_;
  double w_oscillations_;  //!< frequency of motions perpendicular to movement
                           //!< direction
  double amp_oscillations_;
  double startup_duration_;
  bool unidirectional_ = true;

  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
};

class FlowerTraj : public Traj {
 public:
  FlowerTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= T_ + start_trajectory_.getDuration() +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    } else if (t / (T_ + start_trajectory_.getDuration() +
                    end_trajectory_.getDuration()) >=
               double(n_turns_)) {
      got_last_tangential_ = false;
      return true;
    }

    return false;
  }
  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }
  void getSetpoint(const double &t, EefTrajSetpoint &out) override;

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;
  void getFlowerSetpoint(const double &t, Eigen::Vector3d &pos,
                         Eigen::Vector3d &vel);
  void getFlowerSetpoint(const double &t, Eigen::Vector3d &pos,
                         Eigen::Vector3d &vel, Eigen::Vector3d &acc);
  int type_ = TrajectoryType::flower;
  int n_turns_;
  double r_;
  double w_;
  double T_;
  double w_oscillations_;  //!< frequency of motions perpendicular to movement
                           //!< direction
  double amp_oscillations_;
  bool oscillations_radial_ = true;
  double startup_duration_;
  bool unidirectional_ = false;
  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
};

class SinusTraj : public Traj {
 public:
  SinusTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= start_trajectory_.getDuration() + duration_ +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }
  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;
  void getSinusSetpoint(const double &t, Eigen::Vector3d &pos,
                        Eigen::Vector3d &vel);
  void getSinusSetpoint(const double &t, Eigen::Vector3d &pos,
                        Eigen::Vector3d &vel, Eigen::Vector3d &acc);
  int type_ = TrajectoryType::sinus;
  double duration_;
  double forward_velocity_;
  double startup_duration_;
  double w_oscillations_;  //!< frequency of motions perpendicular to movement
                           //!< direction
  double amp_oscillations_;
  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
  bool unidirectional_ = true;
};

class SpiralTraj : public Traj {
 public:
  SpiralTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= start_trajectory_.getDuration() + duration_ +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }
  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;
  void getSpiralSetpoint(const double &t, Eigen::Vector3d &pos,
                         Eigen::Vector3d &vel);
  void getSpiralSetpoint(const double &t, Eigen::Vector3d &pos,
                         Eigen::Vector3d &vel, Eigen::Vector3d &acc);
  int type_ = TrajectoryType::spiral;
  double duration_;
  double forward_velocity_;
  double startup_duration_;
  double w_oscillations_;  //!< frequency of motions perpendicular to movement
                           //!< direction
  double amp_oscillations_;
  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
  bool unidirectional_ = true;
};

class EightTraj : public Traj {
 public:
  EightTraj() = default;

  bool isFinished(const double &t) override {
    if (unidirectional_ && t >= start_trajectory_.getDuration() + T_ +
                                    end_trajectory_.getDuration()) {
      got_last_tangential_ = false;
      return true;
    }
    return false;
  }
  void initialize(rclcpp::Node *node_ptr, bool output) override;

  int trajType() override { return type_; }

 private:
  //!< Returns position, velocity and acceleration all specified in inertial
  //!< coordinate system
  void getTranslationalMotion(const double &t, Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel) override;
  void getEightSetpoint(const double &t, Eigen::Vector3d &pos,
                        Eigen::Vector3d &vel);
  void getEightSetpoint(const double &t, Eigen::Vector3d &pos,
                        Eigen::Vector3d &vel, Eigen::Vector3d &acc);
  int type_ = TrajectoryType::eight;
  double T_;
  double w_;
  double amp_;
  double startup_duration_;
  double t_fix_;

  uvms_common::s2s_trajectory::ReversePoint2PointTrajectory start_trajectory_;
  uvms_common::s2s_trajectory::Point2PointTrajectory end_trajectory_;
  bool unidirectional_ = true;
};
}  // namespace uvms_traj_gen

#endif  // ALPHA_TRAJECTORY_GEN_TRAJ_HPP
