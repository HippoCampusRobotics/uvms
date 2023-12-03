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

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "bluerov_trajectory_gen/traj.hpp"
#include "hippo_common/convert.hpp"
#include "hippo_common/tf2_utils.hpp"
#include "hippo_msgs/msg/control_target.hpp"
#include "uvms_common/pose_to_pose_trajectory.hpp"
#include "uvms_common/ros_param_utils.hpp"

using std::placeholders::_1;

namespace bluerov_traj_gen {
class MotionPlanner : public rclcpp::Node {
 public:
  MotionPlanner() : Node("trajectory_gen") { initialize(); };

 private:
  void initialize() {
    bool print_load_motion_output = true;
    int traj_type;
    ros_param_utils::getParam(this, traj_type, "trajectory_type", 1,
                              print_load_motion_output);
    initializeParameters(print_load_motion_output);
    switch (traj_type) {
      case (TrajectoryType::eight):
        traj_gen_ = new bluerov_traj_gen::EightTraj();
        traj_gen_->initialize(this, print_load_motion_output);
        break;
      case (TrajectoryType::single_DOF):
        traj_gen_ = new bluerov_traj_gen::SingleDOFVelTraj();
        traj_gen_->initialize(this, print_load_motion_output);
        dynamic_cast<SingleDOFVelTraj*>(traj_gen_)->setPositionPtr(&pos_);
        dynamic_cast<SingleDOFVelTraj*>(traj_gen_)->setAttitudePtr(&att_);
        dynamic_cast<SingleDOFVelTraj*>(traj_gen_)->setStartedLoopFlagPtr(
            &start_loop_);
        dynamic_cast<SingleDOFVelTraj*>(traj_gen_)->setGenStartTrajFlagPtr(
            &gen_start_traj_);
        break;
      case (TrajectoryType::single_DOF_setpoints):
        traj_gen_ = new bluerov_traj_gen::SingleDOFSetpoints();
        traj_gen_->initialize(this, print_load_motion_output);
        dynamic_cast<SingleDOFSetpoints*>(traj_gen_)->setPositionPtr(&pos_);
        dynamic_cast<SingleDOFSetpoints*>(traj_gen_)->setAttitudePtr(&att_);
        break;
      case (TrajectoryType::single_DOF_sinusoidal):
        traj_gen_ = new bluerov_traj_gen::SingleDOFSinusoidal();
        traj_gen_->initialize(this, print_load_motion_output);
        dynamic_cast<SingleDOFSinusoidal*>(traj_gen_)->setPositionPtr(&pos_);
        dynamic_cast<SingleDOFSinusoidal*>(traj_gen_)->setAttitudePtr(&att_);
        break;
      case (TrajectoryType::station_keeping):
        traj_gen_ = new bluerov_traj_gen::StationKeeping();
        traj_gen_->initialize(this, print_load_motion_output);
        dynamic_cast<StationKeeping*>(traj_gen_)->setPositionPtr(&pos_);
        dynamic_cast<StationKeeping*>(traj_gen_)->setAttitudePtr(&att_);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "No valid trajectory type specified!");
        break;
    }

    setpoint_pub_ = this->create_publisher<hippo_msgs::msg::ControlTarget>(
        "traj_setpoint", rclcpp::SystemDefaultsQoS());
    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&MotionPlanner::update_state, this, _1));
    wait_for_start_ = false;
    start_loop_ = false;
    gen_start_traj_ = true;
    double freq = 50;
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        static_cast<std::chrono::milliseconds>(int(1000 * 1.0 / freq)),
        std::bind(&MotionPlanner::sendSetpoint, this));
    status_pub_ = this->create_publisher<std_msgs::msg::Int64>(
        "traj_status", rclcpp::SystemDefaultsQoS());
    traj_gen_->addStatusPublisher(status_pub_);
  }
  void initializeParameters(bool output) {
    ros_param_utils::getParam(this, v_max_init_, "v_max_init", 0.1, output);
    ros_param_utils::getParam(this, w_max_init_, "w_max_init", 0.1, output);
    ros_param_utils::getParam(this, start_accuracy_, "start_accuracy", 0.005,
                              output);
  }

  //! send trajectory setpoint in world-COS
  void sendSetpoint() {
    if (!first_msg_) {
      return;
    }
    if (gen_start_traj_) {
      Eigen::Vector3d pos_start;
      Eigen::Quaterniond att_start;
      traj_gen_->getStartPose(pos_start, att_start);
      start_traj_.initializeFromVelocityLimits(pos_, att_, pos_start, att_start,
                                               v_max_init_, w_max_init_);
      start_time_ = this->now();
      gen_start_traj_ = false;
    }

    if (!start_loop_) {
      TrajSetpoint setpoint;
      Eigen::Vector3d pos_start;
      traj_gen_->getStartPosition(pos_start);
      Eigen::Quaterniond att_start;
      traj_gen_->getStartOrientation(att_start);
      // calculate quaternion error:
      Eigen::Matrix3d att_start_tilde;
      skew(att_start.vec(), att_start_tilde);

      Eigen::Vector3d att_error = att_.w() * att_start.vec() -
                                  att_start.w() * att_.vec() -
                                  att_start_tilde * att_.vec();

      Eigen::Vector3d pos_des, vel_des, acc_des, ang_vel_des, ang_acc_des;
      Eigen::Quaterniond att_des;
      const double dt = (this->now() - start_time_).seconds();
      start_traj_.getPositionSetpoint(dt, pos_des, vel_des, acc_des);
      start_traj_.getOrientationSetpoint(dt, att_des, ang_vel_des, ang_acc_des);
      if ((pos_start - pos_).norm() < start_accuracy_ &&
          att_error.norm() < start_accuracy_) {
        start_loop_ = true;
        wait_for_start_ = true;
        start_time_ = this->now();
      }
      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::InertialFrame();
      hippo_common::convert::EigenToRos(pos_des, out_msg_.position);
      hippo_common::convert::EigenToRos(vel_des, out_msg_.velocity);
      hippo_common::convert::EigenToRos(acc_des, out_msg_.acceleration);
      hippo_common::convert::EigenToRos(att_des, out_msg_.attitude);
      hippo_common::convert::EigenToRos(ang_vel_des, out_msg_.angular_velocity);
      hippo_common::convert::EigenToRos(ang_acc_des,
                                        out_msg_.angular_acceleration);
      out_msg_.mask = 0;
      setpoint_pub_->publish(out_msg_);
      return;
    } else if (wait_for_start_) {
      if ((this->now() - start_time_).seconds() >= 1.0) {
        start_time_ = this->now();
        wait_for_start_ = false;
      }
      setpoint_pub_->publish(out_msg_);
      return;
    } else {
      t_ = (this->now() - start_time_).seconds();
      traj_gen_->getSetpointMsg(t_, out_msg_);
    }
    setpoint_pub_->publish(out_msg_);
  };

  void update_state(const nav_msgs::msg::Odometry::SharedPtr msg) {
    stamp_ = rclcpp::Time(msg->header.stamp);

    hippo_common::convert::RosToEigen(msg->pose.pose.position, pos_);
    hippo_common::convert::RosToEigen(msg->pose.pose.orientation, att_);
    if (!first_msg_) {
      first_msg_ = true;
      start_time_ = this->now();
    }
  }

  Eigen::Vector3d pos_;
  Eigen::Quaterniond att_;
  rclcpp::Time stamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hippo_msgs::msg::ControlTarget>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_;

  uvms_common::p2p_trajectory::Pose2PoseTrajectory start_traj_;
  Traj* traj_gen_;
  rclcpp::Time start_time_;
  hippo_msgs::msg::ControlTarget out_msg_;
  double v_max_init_;
  double w_max_init_;
  double t_;
  double start_accuracy_;
  bool first_msg_;  // first state update received?
  bool start_loop_ = false;
  bool gen_start_traj_ = true;
  bool wait_for_start_;
};

}  // namespace bluerov_traj_gen
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bluerov_traj_gen::MotionPlanner>());
  rclcpp::shutdown();

  return 0;
}
