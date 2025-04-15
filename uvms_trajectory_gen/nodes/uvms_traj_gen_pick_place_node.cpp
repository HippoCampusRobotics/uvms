// Copyright (C) 2024  Vincent Lenz

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

#include "uvms_traj_gen_pick_place_node.hpp"

namespace uvms_traj_gen {
UVMSTrajGenPickPlace::UVMSTrajGenPickPlace() : Node("traj_gen_pick_place_node") {
  bool print_load_motion_output = true;

  initializeParameters(print_load_motion_output);

  first_state_ = false;
  received_goal_ = false;
  start_time_ = this->now();
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  eef_traj_pub_ = this->create_publisher<hippo_msgs::msg::ControlTarget>(
      "traj_setpoint", qos);

  pose_eef_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_endeffector", qos,
      std::bind(&UVMSTrajGenPickPlace::updateEef, this, std::placeholders::_1));

  goal_pose_eef_sub_ = this->create_subscription<hippo_msgs::msg::PoseStampedNumbered>(
      "goal_pose_endeffector", qos,
      std::bind(&UVMSTrajGenPickPlace::updateGoal, this, std::placeholders::_1));

  double freq = 50;
  timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      static_cast<std::chrono::milliseconds>(int(1000 * 1.0 / freq)),
      std::bind(&UVMSTrajGenPickPlace::sendSetpoint, this));
  status_pub_ =
      this->create_publisher<std_msgs::msg::Int64>("traj_status", qos);

  planner_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "planner_control_mode", qos,
      std::bind(&UVMSTrajGenPickPlace::updatePlannerMode, this, std::placeholders::_1));
  
  planner_status_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "planner_mode", qos,
      std::bind(&UVMSTrajGenPickPlace::updatePlannerStatus, this, std::placeholders::_1));

  initial_startup_.initialize(this);
  initial_startup_.setTrajStatusPtr(&traj_status_);
  initial_startup_.setTrajStatusPublisherPtr(status_pub_);
}

void UVMSTrajGenPickPlace::initializeParameters(bool output) {
  ros_param_utils::getParam(this, v_max_init_, "v_max_init_eef", 0.1, output);
  ros_param_utils::getParam(this, w_max_init_, "w_max_init_eef", 0.1, output);
  ros_param_utils::getParam(this, start_accuracy_, "start_accuracy", 0.005,
                            output);
  ros_param_utils::getParam(this, run_accuracy_, "run_accuracy", 0.005,
                            output);
  ros_param_utils::getParam(this, v_max_short_init_, "v_max_short_init_eef", 0.1, output);
  ros_param_utils::getParam(this, a_max_init_, "a_max_init_eef", 0.1, output);
  ros_param_utils::getParam(this, dw_max_init_, "dw_max_init_eef", 0.1, output);
  zero_vec_.setZero();
}


//!< returns setpoint in world coordinate system, meaning that position,
//!< velocity, angular velocity are specified in inertial coordinate system,
// attitude represents world-body
void UVMSTrajGenPickPlace::sendSetpoint() {
  if (!first_state_ || planner_mode_==TrajMode::undeclared_mode) {
    return;
  }

  if (planner_mode_==TrajMode::stop) {
    traj_status_=TrajStatus::waiting_for_planner;
  }
 
  switch (traj_status_) {
    case TrajStatus::undeclared:
    case TrajStatus::approaching_initial_auv_pose:
    case TrajStatus::reached_initial_auv_pose:
    case TrajStatus::approaching_initial_manipulator_pose:
      initial_startup_.sendSetpoint();

      // as long as uvms control is active (all states above) a dummy eef trajectory equal 
      // to eef's current pose is send such that estimation_drift_watchdog will not force
      // shutdown
      // this will not affect uvms_switching_kin_ctrl_node since the controller will
      // neglect the eef-trajectory as long as uvms trajectories are published
      
      if (first_state_) {
        out_msg_.header.stamp = this->now();
        out_msg_.header.frame_id =
            hippo_common::tf2_utils::frame_id::kInertialName;

        hippo_common::convert::EigenToRos(pos_, out_msg_.position);
        hippo_common::convert::EigenToRos(zero_vec_, out_msg_.velocity);
        hippo_common::convert::EigenToRos(zero_vec_, out_msg_.acceleration);
        hippo_common::convert::EigenToRos(att_, out_msg_.attitude);
        hippo_common::convert::EigenToRos(zero_vec_, out_msg_.angular_velocity);
        hippo_common::convert::EigenToRos(zero_vec_, out_msg_.angular_acceleration);
        break;
      }
      return;
    case TrajStatus::reached_initial_pose: {
      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;

      hippo_common::convert::EigenToRos(pos_, out_msg_.position);
      hippo_common::convert::EigenToRos(zero_vec_, out_msg_.velocity);
      hippo_common::convert::EigenToRos(zero_vec_, out_msg_.acceleration);
      hippo_common::convert::EigenToRos(att_, out_msg_.attitude);
      hippo_common::convert::EigenToRos(zero_vec_, out_msg_.angular_velocity);
      hippo_common::convert::EigenToRos(zero_vec_, out_msg_.angular_acceleration);

      hippo_common::convert::EigenToEigen(pos_, start_pos_);
      hippo_common::convert::EigenToEigen(att_, start_att_);

      traj_status_ = TrajStatus::waiting_for_planner;
      // initial_startup_.resetConnections();
      std_msgs::msg::Int64 msg;
      msg.data = traj_status_;
      status_pub_->publish(msg);
      break;
    }
    case TrajStatus::waiting_for_goal: {
      if (received_goal_){
        traj_gen_.initializeFromVelocityAccelerationLimits(
          pos_, att_, goal_pos_, goal_att_, v_max_init_, w_max_init_, a_max_init_, dw_max_init_);
        start_time_ = this->now();
        traj_status_ = TrajStatus::approaching_goal;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;  
        status_pub_->publish(msg);
      }
      out_msg_.header.stamp = this->now();  // publish last msg again
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      break;
    }
    case TrajStatus::approaching_goal: {
      EefTrajSetpoint setpoint;

      // calculate quaternion error:
      Eigen::Matrix3d goal_att_tilde;
      skew(goal_att_.vec(), goal_att_tilde); 
      Eigen::Vector3d att_error = att_.w() * goal_att_.vec() - 
                                  goal_att_.w() * att_.vec() -
                                  goal_att_tilde * att_.vec(); 

      double dt = (this->now() - start_time_).seconds();
      traj_gen_.getPositionSetpoint(dt, setpoint.pos, setpoint.vel,
                                      setpoint.acc);
      traj_gen_.getOrientationSetpoint(dt, setpoint.att, setpoint.ang_vel,
                                         setpoint.ang_acc);
      if ((goal_pos_ - pos_).norm() < start_accuracy_ && //check, if eef reached initial start position and orientation/attitude of the trajectory
          att_error.norm() < start_accuracy_) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::reached_goal;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;
        status_pub_->publish(msg);
      }

      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      hippo_common::convert::EigenToRos(setpoint.pos, out_msg_.position);
      hippo_common::convert::EigenToRos(setpoint.vel, out_msg_.velocity);
      hippo_common::convert::EigenToRos(setpoint.acc, out_msg_.acceleration);
      hippo_common::convert::EigenToRos(setpoint.att, out_msg_.attitude);
      hippo_common::convert::EigenToRos(setpoint.ang_vel,
                                        out_msg_.angular_velocity);
      hippo_common::convert::EigenToRos(setpoint.ang_acc,
                                        out_msg_.angular_acceleration);
      out_msg_.mask = 0;
      break;
    }
    case TrajStatus::reached_goal: {
      if ((this->now() - start_time_).seconds() >= 1.0 &&
          planner_mode_ == TrajMode::received_status_reached_goal) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::waiting_for_planner;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;
        status_pub_->publish(msg);
      }

      out_msg_.header.stamp = this->now();  // publish last msg again
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      break;
    }
    case TrajStatus::waiting_for_planner: {
      switch (planner_mode_) {
        case TrajMode::stop:
        case TrajMode::received_status_reached_goal:
        case TrajMode::keep_eef_pose: {
          out_msg_.header.stamp = this->now();  // publish last msg again
          out_msg_.header.frame_id =
            hippo_common::tf2_utils::frame_id::kInertialName;
          break;    
        }
        case TrajMode::new_eff_trajectory: {
          received_goal_ = false;
          traj_status_ = TrajStatus::waiting_for_goal;
          std_msgs::msg::Int64 msg;
          msg.data = traj_status_;
          status_pub_->publish(msg);
          break;
        }
        case TrajMode::go_to_starting_position: {
            traj_status_ = TrajStatus::undeclared;
            initial_startup_.resetAUVState();
            break;
        }
        default:
            RCLCPP_ERROR(this->get_logger(),
                   "Unknown trajectory status, no published trajectory.");
            break;
      }
    }
  }
  eef_traj_pub_->publish(out_msg_);
}

void UVMSTrajGenPickPlace::updateEef(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hippo_common::convert::RosToEigen(msg->pose.position, pos_);
  hippo_common::convert::RosToEigen(msg->pose.orientation, att_);
  if (!first_state_) {
    first_state_ = true;
  }
}

void UVMSTrajGenPickPlace::updateGoal(
    const hippo_msgs::msg::PoseStampedNumbered::SharedPtr msg) {
  if (!received_goal_) {
    // check that the goal is actually new
    if (!first_goal_) {
      old_goal_number_ = msg->number - 1;
      first_goal_ = true;
    }
    if (msg->number == old_goal_number_ + 1) {
      old_goal_number_++;
      received_goal_ = true;
    } else {
      return;
    }
  } else {
    return; //does not update goal pose as long as received_goal_ is not reset
  }      
  hippo_common::convert::RosToEigen(msg->position, goal_pos_);
  hippo_common::convert::RosToEigen(msg->orientation, goal_att_);
}

void UVMSTrajGenPickPlace::updatePlannerMode(
    const std_msgs::msg::Int64::SharedPtr msg) {
  planner_mode_ = msg->data;
}

void UVMSTrajGenPickPlace::updatePlannerStatus(
    const std_msgs::msg::Int64::SharedPtr msg) {
  planner_status_ = msg->data;
}

}  // namespace uvms_traj_gen

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_traj_gen::UVMSTrajGenPickPlace>());
  rclcpp::shutdown();
  return 0;
}
