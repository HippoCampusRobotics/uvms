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

#include "uvms_traj_gen_node.hpp"
namespace uvms_traj_gen {
UVMSTrajGen::UVMSTrajGen() : Node("traj_gen_node") {
  bool print_load_motion_output = true;
  ros_param_utils::getParam(this, n_runs_, "n_runs", 5,
                            print_load_motion_output);
  int traj_type;
  ros_param_utils::getParam(this, traj_type, "trajectory_type", 0,
                            print_load_motion_output);
  initializeParameters(print_load_motion_output);
  switch (traj_type) {
    case (TrajectoryType::straight):
      traj_gen_ = new uvms_traj_gen::StraightLineTraj();
      break;
    case (TrajectoryType::sinusoidal2D):
      traj_gen_ = new uvms_traj_gen::Sinusoidal2DTraj();
      break;
    case (TrajectoryType::si):
      traj_gen_ = new uvms_traj_gen::SiTraj();
      break;
    case (TrajectoryType::flower):
      traj_gen_ = new uvms_traj_gen::FlowerTraj();
      break;
    case (TrajectoryType::sinus):
      traj_gen_ = new uvms_traj_gen::SinusTraj();
      break;
    case (TrajectoryType::spiral):
      traj_gen_ = new uvms_traj_gen::SpiralTraj();
      break;
    case (TrajectoryType::eight):
      traj_gen_ = new uvms_traj_gen::EightTraj();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "No valid trajectory type specified!");
      break;
  }
  traj_gen_->initialize(this, print_load_motion_output);

  first_state_ = false;
  start_time_ = this->now();
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  eef_traj_pub_ =
      this->create_publisher<hippo_control_msgs::msg::ControlTarget>(
          "traj_setpoint", qos);
  if (publish_prediction_) {
    eef_traj_pub_prediction_ =
        this->create_publisher<uvms_msgs::msg::ControlTargetPrediction>(
            "traj_setpoint_prediction", qos);
  }
  pose_eef_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_endeffector", qos,
      std::bind(&UVMSTrajGen::updateEef, this, std::placeholders::_1));
  angular_vel_debug_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "~/angular_vel_naive", qos);

  double freq = 50;
  timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      static_cast<std::chrono::milliseconds>(int(1000 * 1.0 / freq)),
      std::bind(&UVMSTrajGen::sendSetpoint, this));
  status_pub_ =
      this->create_publisher<std_msgs::msg::Int64>("traj_status", qos);
  traj_gen_->addStatusPublisher(status_pub_);
  traj_gen_->addStatusPtr(&traj_status_);

  initial_startup_.initialize(this);
  initial_startup_.setTrajStatusPtr(&traj_status_);
  initial_startup_.setTrajStatusPublisherPtr(status_pub_);
}

void UVMSTrajGen::initializeParameters(bool output) {
  ros_param_utils::getParam(this, v_max_init_, "v_max_init_eef", 0.1, output); //in traj_gen.launch.py are no limits defined as parameters, hence those defaults are used
  ros_param_utils::getParam(this, w_max_init_, "w_max_init_eef", 0.1, output);
  ros_param_utils::getParam(this, start_accuracy_, "start_accuracy", 0.005,
                            output);
}

//!< returns setpoint in world coordinate system, meaning that position,
//!< velocity, angular velocity are specified in inertial coordinate system,
// attitude represents world-body
void UVMSTrajGen::sendSetpoint() {
  if (!first_state_) {
    return;
  }

  switch (traj_status_) {
    case TrajStatus::undeclared:
    case TrajStatus::approaching_initial_auv_pose:
    case TrajStatus::reached_initial_auv_pose:
    case TrajStatus::approaching_initial_manipulator_pose:
      initial_startup_.sendSetpoint();
      return;
    case TrajStatus::reached_initial_pose: {
      EefTrajSetpoint setpoint;
      traj_gen_->getSetpoint(0.0, setpoint); //every traj class generates a time dependant trajectory for the eef, 
      // getsetpoint(t=0, setpoint reference) then writes the initial point of that time dependant trajectory in the setpoint reference
      start_traj_.initializeFromVelocityLimits(
          pos_, att_, setpoint.pos, setpoint.att, v_max_init_, w_max_init_);
      start_time_ = this->now();
      traj_status_ = TrajStatus::approaching_initial_eef_pose;
      initial_startup_.resetConnections();
      std_msgs::msg::Int64 msg;
      msg.data = traj_status_;
      status_pub_->publish(msg);
      return;
    }
    case TrajStatus::approaching_initial_eef_pose: {
      EefTrajSetpoint setpoint;
      Eigen::Vector3d pos_start;
      traj_gen_->getStartPosition(pos_start); //intern wird in der traj.cpp class obige getSetpoint(0.0, setpoint) ausgeführt und dann nur der position Wert zurückgegeben, statt der gesamten pose
      Eigen::Quaterniond att_start;
      traj_gen_->getStartOrientation(att_start); ///intern wird in der traj.cpp class obige getSetpoint(0.0, setpoint) ausgeführt und dann nur der orientation Wert zurückgegeben, statt der gesamten pose

      // calculate quaternion error:
      Eigen::Matrix3d att_start_tilde;
      skew(att_start.vec(), att_start_tilde); // gibt die skew natrix auf att_start_tilde zurück, welche aus den Werten vom att_start.vec() aufgebaut wird

      //dies ist eine bekannte Weise, wie man auf Basis von quaternions den attitude error berechnet. Weiter unten sieht man, dass davon die Norm genommen wird
      //.w() ist der lineare Anteil, .vec() ist der vektorielle Anteil
      //att_ ist der aktuelle orientierungswert des End-Effektors, der vom pose_endeffector topic ausgelesen wird 
      Eigen::Vector3d att_error = att_.w() * att_start.vec() - 
                                  att_start.w() * att_.vec() -
                                  att_start_tilde * att_.vec(); // mithilfe der skew Matrix, kann so das Kreuzprodukt durch Matrix Vektro Multiplikation von att_start und att_ berechnet werden

      // RCLCPP_INFO(this->get_logger(), "%s", ("Distance to start point: " +
      // std::to_string((pos_start-pos_).norm())).c_str());
      // RCLCPP_INFO(this->get_logger(), "%s", ("Attitude error to start point:
      // " + std::to_string(att_error.norm())).c_str());
      double dt = (this->now() - start_time_).seconds();
      start_traj_.getPositionSetpoint(dt, setpoint.pos, setpoint.vel,
                                      setpoint.acc);
      start_traj_.getOrientationSetpoint(dt, setpoint.att, setpoint.ang_vel,
                                         setpoint.ang_acc);
      if ((pos_start - pos_).norm() < start_accuracy_ && //check, if eef reached initial start position and orientation/attitude of the trajectory
          att_error.norm() < start_accuracy_) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::reached_initial_eef_pose;
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

      if (!publish_prediction_) {
        break;
      }
      // forward step:
      const double diff = double(1.0) / freq_;
      dt += diff;
      start_traj_.getPositionSetpoint(dt, setpoint.pos, setpoint.vel,
                                      setpoint.acc);
      start_traj_.getOrientationSetpoint(dt, setpoint.att, setpoint.ang_vel,
                                         setpoint.ang_acc);
      out_msg_prediction_.header.stamp = this->now();
      out_msg_prediction_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      out_msg_prediction_.target = out_msg_;
      hippo_common::convert::EigenToRos(
          setpoint.pos, out_msg_prediction_.target_forward.position);
      hippo_common::convert::EigenToRos(
          setpoint.vel, out_msg_prediction_.target_forward.velocity);
      hippo_common::convert::EigenToRos(
          setpoint.acc, out_msg_prediction_.target_forward.acceleration);
      hippo_common::convert::EigenToRos(
          setpoint.att, out_msg_prediction_.target_forward.attitude);
      hippo_common::convert::EigenToRos(
          setpoint.ang_vel,
          out_msg_prediction_.target_forward.angular_velocity);
      hippo_common::convert::EigenToRos(
          setpoint.ang_acc,
          out_msg_prediction_.target_forward.angular_acceleration);
      out_msg_prediction_.dt = diff;
      out_msg_prediction_.target_forward.mask = out_msg_.mask;

      break;
    }
    case TrajStatus::reached_initial_eef_pose:
      if ((this->now() - start_time_).seconds() >= 1.0) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::adapt_task_dimension;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;
        status_pub_->publish(msg);
      }

      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      traj_gen_->getSetpointMsg(0.0, out_msg_);
      out_msg_.mask = 0;
      break;  // publish desired current end effector pose
    case TrajStatus::adapt_task_dimension: {
      if ((this->now() - start_time_).seconds() >= 4.0) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::start_motion;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;
        status_pub_->publish(msg);
      }
      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      traj_gen_->getSetpointMsg(0.0, out_msg_);

      if (!publish_prediction_) {
        break;
      }

      // forward step:
      out_msg_prediction_.target = out_msg_;
      const double diff = double(1.0) / freq_;
      out_msg_prediction_.header.stamp = this->now();
      out_msg_prediction_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      traj_gen_->getSetpointMsg(0.0, out_msg_prediction_.target_forward);
      out_msg_prediction_.dt = diff;
      out_msg_prediction_.target_forward.mask = out_msg_.mask;
      break;
    }
    case TrajStatus::start_motion: {
      double t = (this->now() - start_time_).seconds();
      out_msg_.header.stamp = this->now();
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      traj_gen_->getSetpointMsg(t, out_msg_);

      if (!publish_prediction_) {
        break;
      }

      // forward step:
      out_msg_prediction_.target = out_msg_;
      const double diff = double(1.0) / freq_;
      t += diff;
      out_msg_prediction_.header.stamp = this->now();
      out_msg_prediction_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      traj_gen_->getSetpointMsg(t, out_msg_prediction_.target_forward);
      out_msg_prediction_.dt = diff;
      out_msg_prediction_.target_forward.mask = out_msg_.mask;
      break;
    }
    case TrajStatus::finished_motion: {  // drive back to initial pose

      run_counter_++;
      RCLCPP_INFO(this->get_logger(), "%s",
                  ("Finished" + std::to_string(run_counter_) + " of " +
                   std::to_string(n_runs_) + " runs.")
                      .c_str());
      if (run_counter_ >= n_runs_) {
        traj_status_ = TrajStatus::finished_run;
      } else {
        start_time_ = this->now();
        traj_status_ = TrajStatus::wait_for_restart;
      }
      std_msgs::msg::Int64 msg;
      msg.data = traj_status_;
      status_pub_->publish(msg);
      break;
    }
    case TrajStatus::wait_for_restart: {  // drive back to initial pose

      if ((this->now() - start_time_).seconds() >= 1.0) {
        start_time_ = this->now();
        traj_status_ = TrajStatus::reached_initial_pose;
        std_msgs::msg::Int64 msg;
        msg.data = traj_status_;
        status_pub_->publish(msg);
      }
      out_msg_.header.stamp = this->now();  // publish last msg again
      out_msg_.header.frame_id =
          hippo_common::tf2_utils::frame_id::kInertialName;
      break;
    }
    case TrajStatus::finished_run: {
      RCLCPP_INFO(this->get_logger(),
                  "Finished run, stopped publishing setpoints");
      return;
    }
    default:
      RCLCPP_ERROR(this->get_logger(),
                   "Unknown trajectory status, no published trajectory.");
      break;
  }
  eef_traj_pub_->publish(out_msg_);
  if (publish_prediction_) {
    eef_traj_pub_prediction_->publish(out_msg_prediction_);
  }
}

void UVMSTrajGen::updateEef(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hippo_common::convert::RosToEigen(msg->pose.position, pos_);
  hippo_common::convert::RosToEigen(msg->pose.orientation, att_);
  if (!first_state_) {
    first_state_ = true;
  }
}

}  // namespace uvms_traj_gen

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_traj_gen::UVMSTrajGen>());
  rclcpp::shutdown();
  return 0;
}
