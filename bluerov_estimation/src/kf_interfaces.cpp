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

#include "bluerov_estimation/kf_interfaces.hpp"

namespace bluerov_estimation {
void KFLinearInterface::initialize(rclcpp::Node *node_ptr,
                                   bool publish_debug_info) {
  node_ptr_ = node_ptr;
  publish_debug_info_ = publish_debug_info;
  debug_velocities_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/kf_linear/velocities", rclcpp::SystemDefaultsQoS());
  debug_accelerations_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/kf_linear/accelerations", rclcpp::SystemDefaultsQoS());
  loadNoiseParam();
}

void KFLinearInterface::initializeKF() {
  // initialize translational KF
  KFLinear::KFStateVector x0_lin;
  x0_lin.setZero();
  x0_lin.segment<KFLinear::n_meas_states>(0) = z_lin_;
  KFLinear::KFStateMatrix P0_lin;
  P0_lin.setZero();
  P0_lin.block<3, 3>(0, 0).diagonal() = w_noise_lin_.vel;
  P0_lin.block<3, 3>(3, 3).diagonal() = v_noise_lin_.acc;
  kf_lin_.initialize(x0_lin, P0_lin, v_noise_lin_, w_noise_lin_);

  // initialize rotational KF
  KFLinear::KFStateVector x0_ang;
  x0_ang.setZero();
  x0_ang.segment<KFLinear::n_meas_states>(0) = z_ang_;
  KFLinear::KFStateMatrix P0_ang;
  P0_ang.setZero();
  P0_ang.block<3, 3>(0, 0).diagonal() = w_noise_ang_.vel;
  P0_ang.block<3, 3>(3, 3).diagonal() = v_noise_ang_.acc;
  kf_ang_.initialize(x0_ang, P0_ang, v_noise_ang_, w_noise_ang_);

  initializeParamCallbacks();
  initialized_ = true;
}

void KFLinearInterface::initializeParamCallbacks() {
  noise_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&KFLinearInterface::onSetNoiseParamCallback, this,
                std::placeholders::_1));
}

void KFLinearInterface::update(const nav_msgs::msg::Odometry::SharedPtr &msg) {
  Eigen::Quaterniond pose = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  z_lin_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
  z_lin_ =
      pose.toRotationMatrix() * z_lin_;  // transform velocity measurement in
                                         // inertial coordinate system, as the
  // Kalman filter refers to the inertial coordinate system
  z_ang_ =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);
  if (!initialized_) {
    initializeKF();
    last_stamp_ = rclcpp::Time(msg->header.stamp);
    return;
  }
  const double dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
  last_stamp_ = msg->header.stamp;

  // update translational KF
  kf_lin_.predict(dt);
  kf_lin_.update(z_lin_);

  // update rotational KF
  kf_ang_.predict(dt);
  kf_ang_.update(z_ang_);
  Eigen::Vector3d local_acceleration_estimate =
      pose.toRotationMatrix().inverse() * kf_lin_.getEstAcc();
  hippo_common::convert::EigenToRos(local_acceleration_estimate,
                                    out_accelerations_.linear);
  hippo_common::convert::EigenToRos(kf_ang_.getEstAcc(),
                                    out_accelerations_.angular);

  // store debug velocities
  Eigen::Vector3d local_velocity_estimate =
      pose.toRotationMatrix().inverse() * kf_lin_.getEstVel();
  hippo_common::convert::EigenToRos(local_velocity_estimate,
                                    debug_velocities_.linear);
  hippo_common::convert::EigenToRos(kf_ang_.getEstVel(),
                                    debug_velocities_.angular);
}

void KFLinearInterface::publishDebugInfo() {
  if (!publish_debug_info_) {
    return;
  }
  if (!initialized_) {
    return;
  }
  geometry_msgs::msg::TwistStamped out_twist;
  out_twist.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  out_twist.header.stamp = node_ptr_->now();
  out_twist.twist = debug_velocities_;
  debug_velocities_pub_->publish(out_twist);
  out_twist.twist = out_accelerations_;
  debug_accelerations_pub_->publish(out_twist);
}

void KFLinearInterface::loadNoiseParam() {
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (int i = 0; i < int(suffixes.size()); i++) {
    ros_param_utils::getParam(node_ptr_, v_noise_lin_.vel(i),
                              "kf_lin.v_noise.linear.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_lin_.acc(i),
                              "kf_lin.v_noise.linear.acc." + suffixes[i], 10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_lin_.vel(i),
                              "kf_lin.w_noise.linear.vel." + suffixes[i], 0.5);
    ros_param_utils::getParam(node_ptr_, v_noise_ang_.vel(i),
                              "kf_lin.v_noise.angular.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_ang_.acc(i),
                              "kf_lin.v_noise.angular.acc." + suffixes[i],
                              10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_ang_.vel(i),
                              "kf_lin.w_noise.angular.vel." + suffixes[i], 0.5);
  }
}

rcl_interfaces::msg::SetParametersResult
KFLinearInterface::onSetNoiseParamCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < int(suffixes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.v_noise.linear.vel." + suffixes[i],
              v_noise_lin_.vel(i))) {
        kf_lin_.setVNoise(v_noise_lin_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_lin_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.v_noise.linear.acc." + suffixes[i],
              v_noise_lin_.acc(i))) {
        kf_lin_.setVNoise(v_noise_lin_.acc(i), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_lin_.acc(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.w_noise.linear.vel." + suffixes[i],
              w_noise_lin_.vel(i))) {
        kf_lin_.setWNoise(w_noise_lin_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_lin_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.v_noise.angular.vel." + suffixes[i],
              v_noise_ang_.vel(i))) {
        kf_ang_.setVNoise(v_noise_ang_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_ang_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.v_noise.angular.acc." + suffixes[i],
              v_noise_ang_.acc(i))) {
        kf_ang_.setVNoise(v_noise_ang_.acc(i), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_ang_.acc(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_lin.w_noise.angular.vel." + suffixes[i],
              w_noise_ang_.vel(i))) {
        kf_ang_.setWNoise(w_noise_ang_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_ang_.vel(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void EKFInterface::initialize(rclcpp::Node *node_ptr, bool publish_debug_info) {
  node_ptr_ = node_ptr;
  publish_debug_info_ = publish_debug_info;
  debug_velocities_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/ekf/velocities", rclcpp::SystemDefaultsQoS());
  debug_accelerations_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/ekf/accelerations", rclcpp::SystemDefaultsQoS());
  thrust_setpoint_sub_ =
      node_ptr_->create_subscription<hippo_control_msgs::msg::ActuatorSetpoint>(
          "thrust_setpoint", rclcpp::SystemDefaultsQoS(),
          std::bind(&EKFInterface::thrustSetpointCallback, this,
                    std::placeholders::_1));
  torque_setpoint_sub_ =
      node_ptr_->create_subscription<hippo_control_msgs::msg::ActuatorSetpoint>(
          "torque_setpoint", rclcpp::SystemDefaultsQoS(),
          std::bind(&EKFInterface::torqueSetpointCallback, this,
                    std::placeholders::_1));
  tau_manipulator_sub_ =
      node_ptr_->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "force_torque", rclcpp::SystemDefaultsQoS(),
          std::bind(&EKFInterface::manipulatorWrenchCallback, this,
                    std::placeholders::_1));
  loadNoiseParam();
  instantiateKF();
  tau_thrusters_.setZero();
  tau_manipulator_.setZero();
}

void EKFInterface::instantiateKF() {
  Matrix6d M;
  Vector6d damping_linear, damping_nonlinear;
  Eigen::Vector3d cog, cob;
  double mass, buoyancy;

  // load parameters
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "model.mass";
  descr_text = "Rigid body mass";
  descr = hippo_common::param_utils::Description(descr_text);
  mass = 11.0;
  mass = node_ptr_->declare_parameter(name, mass, descr);

  name = "model.inertia";
  descr_text = "Rigid body mass";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> inertia_params = {1.0, 1.0, 1.0};
  inertia_params = node_ptr_->declare_parameter(name, inertia_params, descr);
  if (int(inertia_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(inertia_params.size())))
            .c_str());
    return;
  }

  name = "model.cog";
  descr_text = "center of gravity position";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> cog_params = {0.0, 0.0, 0.0};
  cog_params = node_ptr_->declare_parameter(name, cog_params, descr);
  if (int(cog_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(cog_params.size())))
            .c_str());
    return;
  }
  cog = Eigen::Vector3d(cog_params.data());

  M.setZero();
  M.block<3, 3>(0, 0) = mass * Eigen::Matrix3d::Identity();
  M.block<3, 3>(0, 3) = -mass * skew(cog);
  M.block<3, 3>(3, 0) = mass * skew(cog);
  M.block<3, 3>(3, 3) =
      Eigen::DiagonalMatrix<double, 3>(inertia_params[0], inertia_params[1],
                                       inertia_params[2])
          .toDenseMatrix() -
      mass * skew(cog) * skew(cog);

  name = "model.added_mass";
  descr_text = "added mass parameters";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> added_mass_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  added_mass_params =
      node_ptr_->declare_parameter(name, added_mass_params, descr);
  if (int(added_mass_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(added_mass_params.size())))
            .c_str());
    return;
  }
  M.diagonal() += Vector6d(added_mass_params.data());

  name = "model.buoyancy";
  descr_text = "buoyancy";
  buoyancy = 11.0;
  descr = hippo_common::param_utils::Description(descr_text);
  buoyancy = node_ptr_->declare_parameter(name, buoyancy, descr);

  name = "model.cob";
  descr_text = "center of buoyancy position";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> cob_params = {0.0, 0.0, 0.0};
  cob_params = node_ptr_->declare_parameter(name, cob_params, descr);
  if (int(cob_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(cob_params.size())))
            .c_str());
    return;
  }
  cob = Eigen::Vector3d(cob_params.data());

  name = "model.damping_linear";
  descr_text = "linear damping coefficients";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> damping_linear_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  damping_linear_params =
      node_ptr_->declare_parameter(name, damping_linear_params, descr);
  if (int(damping_linear_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(damping_linear_params.size())))
            .c_str());
    return;
  }
  damping_linear = Vector6d(damping_linear_params.data());

  name = "model.damping_nonlinear";
  descr_text = "nonlinear damping coefficients";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> damping_nonlinear_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  damping_nonlinear_params =
      node_ptr_->declare_parameter(name, damping_nonlinear_params, descr);
  if (int(damping_nonlinear_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(damping_nonlinear_params.size())))
            .c_str());
    return;
  }
  damping_nonlinear = Vector6d(damping_nonlinear_params.data());
  kf_ = new EKF(M, damping_linear, damping_nonlinear, cog, cob, mass, buoyancy);
}

void EKFInterface::initializeKF() {
  EKF::KFStateVector x0;
  x0.setZero();
  x0.segment<EKF::n_meas_states>(0) = z_;
  EKF::KFStateMatrix P0;
  P0.setZero();
  P0.block<EKF::n_meas_states, EKF::n_meas_states>(0, 0).diagonal() =
      w_noise_.vel;
  P0.block<6, 6>(EKF::n_meas_states, EKF::n_meas_states).diagonal() =
      v_noise_.acc;
  kf_->initialize(x0, P0, v_noise_, w_noise_);
  initializeParamCallbacks();
  initialized_ = true;
}

void EKFInterface::initializeParamCallbacks() {
  noise_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &EKFInterface::onSetNoiseParamCallback, this, std::placeholders::_1));
}

void EKFInterface::update(const nav_msgs::msg::Odometry::SharedPtr &msg) {
  Eigen::Quaterniond att = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  z_.segment<3>(0) =
      Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);
  z_.segment<3>(3) =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);
  if (!initialized_) {
    initializeKF();
    last_stamp_ = rclcpp::Time(msg->header.stamp);
    return;
  }
  const double dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
  last_stamp_ = msg->header.stamp;

  // update translational KF
  kf_->predict(dt, att, tau_thrusters_, tau_manipulator_);
  kf_->update(z_);

  hippo_common::convert::EigenToRos(kf_->getEstLinAcc(),
                                    out_accelerations_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstAngAcc(),
                                    out_accelerations_.angular);
  // store debug velocities
  hippo_common::convert::EigenToRos(kf_->getEstLinVel(),
                                    debug_velocities_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstAngVel(),
                                    debug_velocities_.angular);
}

void EKFInterface::publishDebugInfo() {
  if (!publish_debug_info_) {
    return;
  }
  if (!initialized_) {
    return;
  }
  geometry_msgs::msg::TwistStamped out_twist;
  out_twist.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  out_twist.header.stamp = node_ptr_->now();
  out_twist.twist = debug_velocities_;
  debug_velocities_pub_->publish(out_twist);
  out_twist.twist = out_accelerations_;
  debug_accelerations_pub_->publish(out_twist);
}

void EKFInterface::loadNoiseParam() {
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (int i = 0; i < int(suffixes.size()); i++) {
    ros_param_utils::getParam(node_ptr_, v_noise_.vel(i),
                              "ekf.v_noise.linear.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_.acc(i),
                              "ekf.v_noise.linear.acc." + suffixes[i], 10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_.vel(i),
                              "ekf.w_noise.linear.vel." + suffixes[i], 0.5);
    ros_param_utils::getParam(node_ptr_, v_noise_.vel(i + 3),
                              "ekf.v_noise.angular.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_.acc(i + 3),
                              "ekf.v_noise.angular.acc." + suffixes[i], 10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_.vel(i + 3),
                              "ekf.w_noise.angular.vel." + suffixes[i], 0.5);
  }
}

rcl_interfaces::msg::SetParametersResult EKFInterface::onSetNoiseParamCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < int(suffixes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.v_noise.linear.vel." + suffixes[i],
              v_noise_.vel(i))) {
        kf_->setVNoise(v_noise_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.v_noise.linear.acc." + suffixes[i],
              v_noise_.acc(i))) {
        kf_->setVNoise(v_noise_.acc(i), i + 6);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.acc(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.w_noise.linear.vel." + suffixes[i],
              w_noise_.vel(i))) {
        kf_->setWNoise(w_noise_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.v_noise.angular.vel." + suffixes[i],
              v_noise_.vel(i + 3))) {
        kf_->setVNoise(v_noise_.vel(i + 3), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.vel(i + 3)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.v_noise.angular.acc." + suffixes[i],
              v_noise_.acc(i + 3))) {
        kf_->setVNoise(v_noise_.acc(i + 3), i + 9);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.acc(i + 3)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "ekf.w_noise.angular.vel." + suffixes[i],
              w_noise_.vel(i + 3))) {
        kf_->setWNoise(w_noise_.vel(i + 3), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_.vel(i + 3)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void EKFInterface::thrustSetpointCallback(
    const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr msg) {
  tau_thrusters_.segment<3>(0) = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void EKFInterface::torqueSetpointCallback(
    const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr msg) {
  tau_thrusters_.segment<3>(3) = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void EKFInterface::manipulatorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  tau_manipulator_ = Vector6d(msg->wrench.force.x, msg->wrench.force.y,
                              msg->wrench.force.z, msg->wrench.torque.x,
                              msg->wrench.torque.y, msg->wrench.torque.z);
}

void KFFeedforwardInterface::initialize(rclcpp::Node *node_ptr,
                                        bool publish_debug_info) {
  node_ptr_ = node_ptr;
  publish_debug_info_ = publish_debug_info;
  debug_velocities_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/kf_ff/velocities", rclcpp::SystemDefaultsQoS());
  debug_accelerations_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/kf_ff/accelerations", rclcpp::SystemDefaultsQoS());
  thrust_setpoint_sub_ =
      node_ptr_->create_subscription<hippo_control_msgs::msg::ActuatorSetpoint>(
          "thrust_setpoint", rclcpp::SystemDefaultsQoS(),
          std::bind(&KFFeedforwardInterface::thrustSetpointCallback, this,
                    std::placeholders::_1));
  torque_setpoint_sub_ =
      node_ptr_->create_subscription<hippo_control_msgs::msg::ActuatorSetpoint>(
          "torque_setpoint", rclcpp::SystemDefaultsQoS(),
          std::bind(&KFFeedforwardInterface::torqueSetpointCallback, this,
                    std::placeholders::_1));
  tau_manipulator_sub_ =
      node_ptr_->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "force_torque", rclcpp::SystemDefaultsQoS(),
          std::bind(&KFFeedforwardInterface::manipulatorWrenchCallback, this,
                    std::placeholders::_1));
  loadNoiseParam();
  instantiateKF();
  tau_thrusters_.setZero();
  tau_manipulator_.setZero();
}

void KFFeedforwardInterface::instantiateKF() {
  Matrix6d M;
  Vector6d damping_linear, damping_nonlinear;
  Eigen::Vector3d cog, cob;
  double mass, buoyancy;

  // load parameters
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "model.mass";
  descr_text = "Rigid body mass";
  descr = hippo_common::param_utils::Description(descr_text);
  mass = 11.0;
  mass = node_ptr_->declare_parameter(name, mass, descr);

  name = "model.inertia";
  descr_text = "Rigid body mass";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> inertia_params = {1.0, 1.0, 1.0};
  inertia_params = node_ptr_->declare_parameter(name, inertia_params, descr);
  if (int(inertia_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(inertia_params.size())))
            .c_str());
    return;
  }

  name = "model.cog";
  descr_text = "center of gravity position";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> cog_params = {0.0, 0.0, 0.0};
  cog_params = node_ptr_->declare_parameter(name, cog_params, descr);
  if (int(cog_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(cog_params.size())))
            .c_str());
    return;
  }
  cog = Eigen::Vector3d(cog_params.data());

  M.setZero();
  M.block<3, 3>(0, 0) = mass * Eigen::Matrix3d::Identity();
  M.block<3, 3>(0, 3) = -mass * skew(cog);
  M.block<3, 3>(3, 0) = mass * skew(cog);
  M.block<3, 3>(3, 3) =
      Eigen::DiagonalMatrix<double, 3>(inertia_params[0], inertia_params[1],
                                       inertia_params[2])
          .toDenseMatrix() -
      mass * skew(cog) * skew(cog);

  name = "model.added_mass";
  descr_text = "added mass parameters";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> added_mass_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  added_mass_params =
      node_ptr_->declare_parameter(name, added_mass_params, descr);
  if (int(added_mass_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(added_mass_params.size())))
            .c_str());
    return;
  }
  M.diagonal() += Vector6d(added_mass_params.data());

  name = "model.buoyancy";
  descr_text = "buoyancy";
  buoyancy = 11.0;
  descr = hippo_common::param_utils::Description(descr_text);
  buoyancy = node_ptr_->declare_parameter(name, buoyancy, descr);

  name = "model.cob";
  descr_text = "center of buoyancy position";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> cob_params = {0.0, 0.0, 0.0};
  cob_params = node_ptr_->declare_parameter(name, cob_params, descr);
  if (int(cob_params.size()) != 3) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 3, but has" +
         std::to_string(int(cob_params.size())))
            .c_str());
    return;
  }
  cob = Eigen::Vector3d(cob_params.data());

  name = "model.damping_linear";
  descr_text = "linear damping coefficients";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> damping_linear_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  damping_linear_params =
      node_ptr_->declare_parameter(name, damping_linear_params, descr);
  if (int(damping_linear_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(damping_linear_params.size())))
            .c_str());
    return;
  }
  damping_linear = Vector6d(damping_linear_params.data());

  name = "model.damping_nonlinear";
  descr_text = "nonlinear damping coefficients";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<double> damping_nonlinear_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  damping_nonlinear_params =
      node_ptr_->declare_parameter(name, damping_nonlinear_params, descr);
  if (int(damping_nonlinear_params.size()) != 6) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        ("Loaded parameter " + name + " is expected to have size 6, but has" +
         std::to_string(int(damping_nonlinear_params.size())))
            .c_str());
    return;
  }
  damping_nonlinear = Vector6d(damping_nonlinear_params.data());

  name = "manipulator_comp.buoyancy_mass";
  descr_text = "Buoyancy added to the AUV to compensate the manipulator weight";
  descr = hippo_common::param_utils::Description(descr_text);
  double added_buoyancy_mass = 0.0;
  added_buoyancy_mass =
      node_ptr_->declare_parameter(name, added_buoyancy_mass, descr);

  std::vector<std::string> suffixes = {"x", "y", "z"};
  Eigen::Vector3d added_buoyancy_origin;
  for (int i = 0; i < int(suffixes.size()); i++) {
    name = "manipulator_comp.origin." + suffixes[i];
    descr_text =
        "Center of buoyancy added to the AUV to compensate the manipulator "
        "weight";
    descr = hippo_common::param_utils::Description(descr_text);
    added_buoyancy_origin(i) = 0.0;
    added_buoyancy_origin(i) =
        node_ptr_->declare_parameter(name, added_buoyancy_origin(i), descr);
  }

  kf_ = new KFFeedforward(M, damping_linear, damping_nonlinear, cog, cob, mass,
                          buoyancy, added_buoyancy_mass, added_buoyancy_origin);

  name = "kf_ff.moving_average";
  descr_text = "constant for calculating moving average of model output";
  descr = hippo_common::param_utils::Description(descr_text);
  double moving_average = 1.0;
  moving_average = node_ptr_->declare_parameter(name, moving_average, descr);
  kf_->setMovingAverage(moving_average);
}

void KFFeedforwardInterface::initializeKF() {
  KFFeedforward::KFStateVector x0;
  x0.setZero();
  x0.segment<KFFeedforward::n_meas_states>(0) = z_;
  KFFeedforward::KFStateMatrix P0;
  P0.setZero();
  P0.block<KFFeedforward::n_meas_states, KFFeedforward::n_meas_states>(0, 0)
      .diagonal() = w_noise_.vel;
  P0.block<6, 6>(KFFeedforward::n_meas_states, KFFeedforward::n_meas_states)
      .diagonal() = v_noise_.acc;
  kf_->initialize(x0, P0, v_noise_, w_noise_);
  initializeParamCallbacks();
  initialized_ = true;
}

void KFFeedforwardInterface::initializeParamCallbacks() {
  noise_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&KFFeedforwardInterface::onSetNoiseParamCallback, this,
                std::placeholders::_1));
  moving_average_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&KFFeedforwardInterface::onSetMovingAverageParamCallback, this,
                std::placeholders::_1));
}

void KFFeedforwardInterface::update(
    const nav_msgs::msg::Odometry::SharedPtr &msg) {
  Eigen::Quaterniond att = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Vector6d nu;
  nu.segment<3>(0) =
      Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);
  nu.segment<3>(3) =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);
  // z_.segment<3>(0) = att.toRotationMatrix() * nu.segment<3>(0); // todo
  // transformed here to world coordinate system z_.segment<3>(3) =
  // nu.segment<3>(3);
  z_ = nu;

  if (!initialized_) {
    initializeKF();
    last_stamp_ = rclcpp::Time(msg->header.stamp);
    return;
  }
  const double dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
  last_stamp_ = msg->header.stamp;

  kf_->updateKFInput(att, nu, tau_thrusters_, tau_manipulator_);
  kf_->predict(dt);
  kf_->update(z_);

  // Eigen::Vector3d local_acceleration_estimate =
  // att.toRotationMatrix().inverse() * kf_->getEstLinAcc();  // todo
  // transformed here to body coordinate system
  // hippo_common::convert::EigenToRos(local_acceleration_estimate,
  // out_accelerations_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstLinAcc(),
                                    out_accelerations_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstAngAcc(),
                                    out_accelerations_.angular);
  // store debug velocities
  // Eigen::Vector3d local_velocity_estimate = att.toRotationMatrix().inverse()
  // * kf_->getEstLinVel(); // todo transformed here to body coordinate system
  // hippo_common::convert::EigenToRos(local_velocity_estimate,
  // debug_velocities_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstLinVel(),
                                    debug_velocities_.linear);
  hippo_common::convert::EigenToRos(kf_->getEstAngVel(),
                                    debug_velocities_.angular);
}

void KFFeedforwardInterface::publishDebugInfo() {
  if (!publish_debug_info_) {
    return;
  }
  if (!initialized_) {
    return;
  }
  geometry_msgs::msg::TwistStamped out_twist;
  out_twist.header.frame_id =
      hippo_common::tf2_utils::frame_id::BaseLink(node_ptr_);
  out_twist.header.stamp = node_ptr_->now();
  out_twist.twist = debug_velocities_;
  debug_velocities_pub_->publish(out_twist);
  out_twist.twist = out_accelerations_;
  debug_accelerations_pub_->publish(out_twist);
}

void KFFeedforwardInterface::loadNoiseParam() {
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (int i = 0; i < int(suffixes.size()); i++) {
    ros_param_utils::getParam(node_ptr_, v_noise_.vel(i),
                              "kf_ff.v_noise.linear.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_.acc(i),
                              "kf_ff.v_noise.linear.acc." + suffixes[i], 10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_.vel(i),
                              "kf_ff.w_noise.linear.vel." + suffixes[i], 0.5);
    ros_param_utils::getParam(node_ptr_, v_noise_.vel(i + 3),
                              "kf_ff.v_noise.angular.vel." + suffixes[i], 1.0);
    ros_param_utils::getParam(node_ptr_, v_noise_.acc(i + 3),
                              "kf_ff.v_noise.angular.acc." + suffixes[i], 10.0);
    ros_param_utils::getParam(node_ptr_, w_noise_.vel(i + 3),
                              "kf_ff.w_noise.angular.vel." + suffixes[i], 0.5);
  }
}

rcl_interfaces::msg::SetParametersResult
KFFeedforwardInterface::onSetNoiseParamCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> suffixes = {"x", "y", "z"};
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < int(suffixes.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.v_noise.linear.vel." + suffixes[i],
              v_noise_.vel(i))) {
        kf_->setVNoise(v_noise_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.v_noise.linear.acc." + suffixes[i],
              v_noise_.acc(i))) {
        kf_->setVNoise(v_noise_.acc(i), i + 6);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.acc(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.w_noise.linear.vel." + suffixes[i],
              w_noise_.vel(i))) {
        kf_->setWNoise(w_noise_.vel(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_.vel(i)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.v_noise.angular.vel." + suffixes[i],
              v_noise_.vel(i + 3))) {
        kf_->setVNoise(v_noise_.vel(i + 3), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.vel(i + 3)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.v_noise.angular.acc." + suffixes[i],
              v_noise_.acc(i + 3))) {
        kf_->setVNoise(v_noise_.acc(i + 3), i + 9);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(v_noise_.acc(i + 3)))
                        .c_str());
        break;
      }
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "kf_ff.w_noise.angular.vel." + suffixes[i],
              w_noise_.vel(i + 3))) {
        kf_->setWNoise(w_noise_.vel(i + 3), i + 3);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(w_noise_.vel(i + 3)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult
KFFeedforwardInterface::onSetMovingAverageParamCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double moving_average;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "kf_ff.moving_average", moving_average)) {
      kf_->setMovingAverage(moving_average);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(moving_average))
                      .c_str());
      break;
    }
  }
  return result;
}

void KFFeedforwardInterface::thrustSetpointCallback(
    const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr msg) {
  tau_thrusters_.segment<3>(0) = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void KFFeedforwardInterface::torqueSetpointCallback(
    const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr msg) {
  tau_thrusters_.segment<3>(3) = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void KFFeedforwardInterface::manipulatorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  tau_manipulator_ = Vector6d(msg->wrench.force.x, msg->wrench.force.y,
                              msg->wrench.force.z, msg->wrench.torque.x,
                              msg->wrench.torque.y, msg->wrench.torque.z);
}
}  // namespace bluerov_estimation
