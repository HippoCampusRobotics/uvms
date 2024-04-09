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

#include "uvms_kinematic_ctrl/kin_ctrl_node_interface.hpp"

namespace uvms_kin_ctrl {

void TaskInterface::initialize(rclcpp::Node *node_ptr, std::mutex *mutex_ptr,
                               const int &task_type,
                               const bool &publish_task_velocities) {
  node_ptr_ = node_ptr;
  mutex_ptr_ = mutex_ptr;
  initializeTask();
  if (publish_task_velocities) {
    debug_pub_ = node_ptr_->create_publisher<uvms_msgs::msg::TaskPriorityDebug>(
        "~/cmd_task_" + taskNames.at(task_type), rclcpp::SystemDefaultsQoS());
  }
}

void TaskInterface::publishCmds() {
  uvms_msgs::msg::TaskPriorityDebug msg;
  if (getTaskPtr()->isActive()) {
    const Eigen::Matrix<double, n_states_uvms, 1> debug_task_vel =
        getTaskPtr()->getDesiredVelocities();
    std::copy(debug_task_vel.begin(), debug_task_vel.end(),
              msg.desired_velocity.begin());
    const Eigen::Matrix<double, n_states_uvms, 1> debug_projected_vel =
        getTaskPtr()->getProjectedVelocities();
    std::copy(debug_projected_vel.begin(), debug_projected_vel.end(),
              msg.projected_velocity.begin());
    msg.active_tasks = getTaskPtr()->countActive();
    msg.activation_values.resize(getTaskPtr()->getFullTaskDimension());
    std::copy(getTaskPtr()->getActivationMatrix().diagonal().begin(),
              getTaskPtr()->getActivationMatrix().diagonal().end(),
              msg.activation_values.begin());
  } else {
    std::fill(msg.desired_velocity.begin(), msg.desired_velocity.end(), 0.0);
    std::fill(msg.projected_velocity.begin(), msg.projected_velocity.end(),
              0.0);
    msg.active_tasks = 0;
    msg.activation_values.resize(getTaskPtr()->getFullTaskDimension());
    std::fill(msg.activation_values.begin(), msg.activation_values.end(), 0.0);
  }
  debug_pub_->publish(msg);
}

void JointLimitTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};
  std::vector<std::string> description_suffix = {"joint 0", "joint 1",
                                                 "joint 2", "joint 3"};
  std::vector<double> default_params;
  default_params.resize(n_active_joints);
  default_params = {0.032, 0.0174533, 0.0174533, -1000000};

  StateVector joint_limits_min = StateVector::Zero();

  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits.q_min." + params[i];
    descr_text = "Lower joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_min(i) = default_params[i];
    joint_limits_min(i) =
        node_ptr_->declare_parameter(name, joint_limits_min(i), descr);
  }

  default_params = {6.02, 3.40339, 3.40339, 10000000.0};
  StateVector joint_limits_max = StateVector::Zero();
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits.q_max." + params[i];
    descr_text = "Upper joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_max(i) = default_params[i];
    joint_limits_max(i) =
        node_ptr_->declare_parameter(name, joint_limits_max(i), descr);
  }

  default_params = {1.0, 1.0, 1.0, 1.0};
  StateVector gains = StateVector::Zero();
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits.gain." + params[i];
    descr_text = "Gains for joint limits for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    gains(i) = default_params[i];
    gains(i) = node_ptr_->declare_parameter(name, gains(i), descr);
  }

  double safety = 0.05;
  name = "joint_limits.safety";
  descr_text = "Relative safety bound to hardware constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  safety = node_ptr_->declare_parameter(name, safety, descr);

  double delta = 0.05;
  name = "joint_limits.delta";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "joint_limits.alpha";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(safety, delta, alpha, joint_limits_min, joint_limits_max,
                   gains);
}

void JointLimitTaskInterface::addParameterCallback() {
  gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &JointLimitTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult JointLimitTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "joint_limits.gain." + params[i], gains_(i))) {
        task_.setTaskGain(gains_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gains_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void JointLimitDesiredTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};
  std::vector<std::string> description_suffix = {"joint 0", "joint 1",
                                                 "joint 2", "joint 3"};
  std::vector<double> default_params;
  default_params.resize(n_active_joints);
  default_params = {0.032, 0.0174533, 0.0174533, -1000000};

  StateVector joint_limits_min = StateVector::Zero();

  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits_desired.q_min." + params[i];
    descr_text = "Lower joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_min(i) = default_params[i];
    joint_limits_min(i) =
        node_ptr_->declare_parameter(name, joint_limits_min(i), descr);
  }

  default_params = {6.02, 3.40339, 3.40339, 10000000.0};
  StateVector joint_limits_max = StateVector::Zero();
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits_desired.q_max." + params[i];
    descr_text = "Upper joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_max(i) = default_params[i];
    joint_limits_max(i) =
        node_ptr_->declare_parameter(name, joint_limits_max(i), descr);
  }

  default_params = {1.0, 1.0, 1.0, 1.0};
  StateVector gains = StateVector::Zero();
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_limits_desired.gain." + params[i];
    descr_text = "Gains for joint limits for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    gains(i) = default_params[i];
    gains(i) = node_ptr_->declare_parameter(name, gains(i), descr);
  }

  double safety = 0.05;
  name = "joint_limits_desired.safety";
  descr_text = "Relative safety bound to hardware constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  safety = node_ptr_->declare_parameter(name, safety, descr);

  double delta = 0.05;
  name = "joint_limits_desired.delta";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "joint_limits_desired.alpha";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(safety, delta, alpha, joint_limits_min, joint_limits_max,
                   gains);
}

void JointLimitDesiredTaskInterface::addParameterCallback() {
  gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&JointLimitDesiredTaskInterface::onSetGains, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
JointLimitDesiredTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "joint_limits_desired.gain." + params[i], gains_(i))) {
        task_.setTaskGain(gains_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gains_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void JointCenteringTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};
  std::vector<std::string> description_suffix = {"joint 0", "joint 1",
                                                 "joint 2", "joint 3"};
  std::vector<double> default_params;
  default_params.resize(n_active_joints);
  default_params = {0.032, 0.0174533, 0.0174533, -1000000};

  StateVector joint_limits_min;
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_centering.q_min." + params[i];
    descr_text = "Lower joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_min(i) = default_params[i];
    joint_limits_min(i) =
        node_ptr_->declare_parameter(name, joint_limits_min(i), descr);
  }

  default_params = {6.02, 3.40339, 3.40339, 10000000.0};
  StateVector joint_limits_max;
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_centering.q_max." + params[i];
    descr_text = "Upper joint limit for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    joint_limits_max(i) = default_params[i];
    joint_limits_max(i) =
        node_ptr_->declare_parameter(name, joint_limits_max(i), descr);
  }

  default_params = {1.0, 1.0, 1.0, 1.0};
  StateVector gains;
  for (int i = 0; i < int(params.size()); i++) {
    name = "joint_centering.gain." + params[i];
    descr_text = "Gains for joint limits for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    gains[i] = default_params[i];
    gains[i] = node_ptr_->declare_parameter(name, gains[i], descr);
  }
  task_.initialize(joint_limits_min, joint_limits_max, gains);
}

void JointCenteringTaskInterface::addParameterCallback() {
  gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &JointCenteringTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
JointCenteringTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {"q0", "q1", "q2", "q3"};

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "joint_centering.gain." + params[i], gains_(i))) {
        task_.setTaskGain(gains_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gains_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void VelocityTaskInterface::initializeTask() { task_.initialize(); }

void SelfCollisionEllipseTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "ellipse_collision.idxs";
  descr_text = "Link indexes for ellipse constraints";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<int64_t> idxs;
  ros_param_utils::getParamArray(node_ptr_, idxs, name, {2, 4});

  std::vector<std::string> params = {"x", "z"};
  std::vector<std::string> description_suffix = {"x", "z"};
  std::array<double, 2> a = {0.26, 0.2};
  std::array<double, 2> c = {-0.203, -0.131675};

  for (int i = 0; i < int(params.size()); i++) {
    name = "ellipse_collision.a." + params[i];
    descr_text = "Ellipse main axis scaling, direction" + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    a[i] = node_ptr_->declare_parameter(name, a[i], descr);
  }
  for (int i = 0; i < int(params.size()); i++) {
    name = "ellipse_collision.c." + params[i];
    descr_text = "Ellipse center position, direction" + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    c[i] = node_ptr_->declare_parameter(name, c[i], descr);
  }

  double gain = 1.0;
  name = "ellipse_collision.gain";
  descr_text = "Gain for task";
  descr = hippo_common::param_utils::Description(descr_text);
  gain = node_ptr_->declare_parameter(name, gain, descr);

  double safety = 0.05;
  name = "ellipse_collision.safety";
  descr_text = "Relative safety bound to hardware constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  safety = node_ptr_->declare_parameter(name, safety, descr);

  double delta = 0.05;
  name = "ellipse_collision.delta";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "ellipse_collision.alpha";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::vector<int> idxs_int(idxs.size());
  std::copy(idxs.begin(), idxs.end(), idxs_int.begin());
  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(idxs_int, safety, delta, alpha, a[0], a[1], c[0], c[1],
                   gain);
}

void SelfCollisionEllipseTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&SelfCollisionEllipseTaskInterface::onSetGains, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
SelfCollisionEllipseTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "ellipse_collision.gain", gain_)) {
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void AUVConfinedSpaceTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"x", "y", "z"};
  std::vector<std::string> description_suffix = {"x", "y", "z"};

  std::vector<std::vector<double>> points(3);
  std::vector<std::vector<double>> normals(3);
  points[0] = {0.0, 0.0, 0.0, 2.0, 2.0, 2.0};  // default values
  points[1] = {0.0, 0.0, 0.0, 4.0, 4.0, 4.0};
  points[2] = {-1.3, -1.3, -1.3, 0.0, 0.0, 0.0};
  normals[0] = {1.0, 0.0, 0.0, -1.0, 0.0, 0.0};
  normals[1] = {0.0, 1.0, 0.0, 0.0, -1.0, 0.0};
  normals[2] = {0.0, 0.0, 1.0, 0.0, 0.0, -1.0};

  for (int i = 0; i < 3; i++) {
    name = "auv_confined_space.points." + params[i];
    ros_param_utils::getParamArray(node_ptr_, points[i], name, points[i]);
    name = "auv_confined_space.normals." + params[i];
    ros_param_utils::getParamArray(node_ptr_, normals[i], name, normals[i]);
    if (points[i].size() != normals[i].size()) {
      RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Input point size AUV plane constraints does not match normals size");
      return;
    }
  }
  if ((points[0].size() != points[1].size()) ||
      (points[1].size() != points[2].size()) ||
      (points[0].size() != points[2].size())) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "Input point sizes for AUV plane constraints not consistent");
    return;
  }

  double gain = 1.0;
  name = "auv_confined_space.gain";
  descr_text = "Gain for task";
  descr = hippo_common::param_utils::Description(descr_text);
  gain = node_ptr_->declare_parameter(name, gain, descr);

  double safety = 0.05;
  name = "auv_confined_space.safety";
  descr_text = "Relative safety bound to hardware constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  safety = node_ptr_->declare_parameter(name, safety, descr);

  double delta = 0.05;
  name = "auv_confined_space.delta";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "auv_confined_space.alpha";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  double radius_auv = 0.3;
  name = "auv_confined_space.auv_radius";
  descr_text = "AUV radius used to calculate bounding planes";
  descr = hippo_common::param_utils::Description(descr_text);
  radius_auv = node_ptr_->declare_parameter(name, radius_auv, descr);

  std::vector<Eigen::Vector3d> plane_pos;
  std::vector<Eigen::Vector3d> plane_dir;
  for (int i = 0; i < int(points[0].size()); i++) {
    Eigen::Vector3d point =
        Eigen::Vector3d(points[0][i], points[1][i], points[2][i]);
    Eigen::Vector3d normal =
        Eigen::Vector3d(normals[0][i], normals[1][i], normals[2][i]);
    point += radius_auv * normal;
    plane_pos.push_back(point);
    plane_dir.push_back(normal);
  }

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(safety, delta, alpha, plane_dir, plane_pos, gain);
}

void RestrictingPlaneTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "restricting_plane.idxs";
  descr_text = "Link indexes for plane constraints";
  descr = hippo_common::param_utils::Description(descr_text);
  std::vector<int64_t> idxs;
  ros_param_utils::getParamArray(node_ptr_, idxs, name, {1, 4});

  std::vector<std::string> params = {"x", "y", "z"};
  std::vector<std::string> description_suffix = {"x", "y", "z"};
  Eigen::Vector3d default_params(0.0, 0.0, 0.0);
  Eigen::Vector3d plane_pos;

  for (int i = 0; i < int(params.size()); i++) {
    name = "restricting_plane.plane.p." + params[i];
    descr_text =
        "Position for vertical plane, direction" + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    plane_pos(i) = default_params(i);
    plane_pos(i) = node_ptr_->declare_parameter(name, plane_pos(i), descr);
  }

  Eigen::Vector3d plane_dir;
  default_params << 1.0, 0.0, 0.0;
  for (int i = 0; i < int(params.size()); i++) {
    name = "restricting_plane.planes.n." + params[i];
    descr_text =
        "Direction for vertical plane, direction" + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    plane_dir(i) = default_params(i);
    plane_dir(i) = node_ptr_->declare_parameter(name, plane_dir(i), descr);
  }

  double gain = 1.0;
  name = "restricting_plane.gain";
  descr_text = "Gain for task";
  descr = hippo_common::param_utils::Description(descr_text);
  gain = node_ptr_->declare_parameter(name, gain, descr);

  double safety = 0.0;
  name = "restricting_plane.safety";
  descr_text = "Relative safety bound to hardware constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  safety = node_ptr_->declare_parameter(name, safety, descr);

  double delta = 0.05;
  name = "restricting_plane.delta";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "restricting_plane.alpha";
  descr_text = "Relative safety bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::vector<int> idxs_int(idxs.size());
  for (int i = 0; i < int(idxs_int.size()); i++) {
    idxs_int[i] = static_cast<int>(idxs[i]);
  }
  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(idxs_int, safety, delta, alpha, plane_dir, plane_pos, gain);
}

void RestrictingPlaneTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &RestrictingPlaneTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
RestrictingPlaneTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "restricting_plane.gain", gain_)) {
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void AUVConfinedSpaceTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &AUVConfinedSpaceTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
AUVConfinedSpaceTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "auv_confined_space.gain", gain_)) {
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void AUVAttitudeLimitTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"roll", "pitch", "yaw"};
  std::vector<std::string> description_suffix = params;

  std::vector<int64_t> idxs_tmp = {0, 1, 2};
  name = "attitude_limits.idxs";
  descr_text = "Indexes for relevant axes";
  descr = hippo_common::param_utils::Description(descr_text);
  idxs_tmp = node_ptr_->declare_parameter(name, idxs_tmp, descr);

  std::vector<int> idxs(idxs_tmp.size());
  for (int i = 0; i < int(idxs.size()); i++) {
    idxs[i] = int(idxs_tmp[i]);
    std::cout << "Loaded index: " << idxs[i] << std::endl;
  }

  std::vector<double> default_params;
  default_params = {-1.0 / 3, -1.0 / 3, -2.0 * 1};
  std::vector<double> attitude_limits_min(idxs.size());
  for (int i = 0; i < int(idxs.size()); i++) {
    name = "attitude_limits.min." + params[idxs[i]];
    descr_text = "Lower attitude limit for " + description_suffix[idxs[i]];
    descr = hippo_common::param_utils::Description(descr_text);
    attitude_limits_min[i] = default_params[idxs[i]];
    attitude_limits_min[i] =
        node_ptr_->declare_parameter(name, attitude_limits_min[i], descr);
    attitude_limits_min[i] *= M_PI;
  }

  default_params = {1.0 / 3, 1.0 / 3, 2.0 * 1};
  std::vector<double> attitude_limits_max(idxs.size());
  for (int i = 0; i < int(idxs.size()); i++) {
    name = "attitude_limits.max." + params[idxs[i]];
    descr_text = "Upper attitude limit for " + description_suffix[idxs[i]];
    descr = hippo_common::param_utils::Description(descr_text);
    attitude_limits_max[i] = default_params[idxs[i]];
    attitude_limits_max[i] =
        node_ptr_->declare_parameter(name, attitude_limits_max[i], descr);
    attitude_limits_max[i] *= M_PI;
  }

  default_params = {1.0, 1.0, 1.0};
  std::vector<double> gains = {0.0, 0.0, 0.0};
  for (int i = 0; i < int(idxs.size()); i++) {
    name = "attitude_limits.gain." + params[idxs[i]];
    descr_text = "Gains for attitude limits for " + description_suffix[idxs[i]];
    descr = hippo_common::param_utils::Description(descr_text);
    gains[i] = default_params[i];
    gains[i] = node_ptr_->declare_parameter(name, gains[i], descr);
  }

  double delta = 0.05;
  name = "attitude_limits.delta";
  descr_text = "Relative bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "attitude_limits.alpha";
  descr_text = "Relative bound for deactivating constraint again";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(idxs, delta, alpha, attitude_limits_min, attitude_limits_max,
                   gains);
}

void AUVAttitudeLimitTaskInterface::addParameterCallback() {
  gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &AUVAttitudeLimitTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
AUVAttitudeLimitTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {"roll", "pitch", "yaw"};

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "attitude_limits.gain." + params[i], gains_(i))) {
        task_.setTaskGain(gains_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(gains_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void AUVAttitudeInclinationLimitTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  double default_param;
  default_param = 0.5;
  double max_inclination;
  name = "attitude_inclination_limits.max";
  descr_text = "Maximum inclination limit";
  descr = hippo_common::param_utils::Description(descr_text);
  max_inclination = default_param;
  max_inclination = node_ptr_->declare_parameter(name, max_inclination, descr);
  max_inclination *= M_PI;

  double gain;
  name = "attitude_inclination_limits.gain";
  descr_text = "Gains for attitude inclination limit";
  descr = hippo_common::param_utils::Description(descr_text);
  gain = 1.0;
  gain = node_ptr_->declare_parameter(name, gain, descr);

  double delta = 0.05;
  name = "attitude_inclination_limits.delta";
  descr_text = "Relative bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  delta = node_ptr_->declare_parameter(name, delta, descr);

  double alpha = 0.05;
  name = "attitude_inclination_limits.alpha";
  descr_text = "Relative bound for deactivating constraint again";
  descr = hippo_common::param_utils::Description(descr_text);
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(delta, alpha, max_inclination, gain);
}

void AUVAttitudeInclinationLimitTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&AUVAttitudeInclinationLimitTaskInterface::onSetGain, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
AUVAttitudeInclinationLimitTaskInterface::onSetGain(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "attitude_inclination_limits.gain", gain_)) {
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void EndeffectorTrackingTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  std::vector<std::string> params = {"pos.x", "pos.y", "pos.z",
                                     "att.x", "att.y", "att.z"};
  std::vector<std::string> description_suffix = {"pos x", "pos y", "pos z",
                                                 "att x", "att y", "att z"};
  std::vector<double> default_params;
  default_params.resize(params.size());
  std::fill(default_params.begin(), default_params.end(), 1.0);

  for (int i = 0; i < int(params.size()); i++) {
    name = "eef_tracking.task_error_gain." + params[i];
    descr_text = "Proportional gain for endeffector kinematic control " +
                 description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(*mutex_ptr_);
      p_gains_(i) = default_params[i];
      p_gains_(i) = node_ptr_->declare_parameter(name, p_gains_(i), descr);
      task_.setEefGain(p_gains_(i), i);
    }
  }
}

void EndeffectorTrackingTaskInterface::addParameterCallback() {
  p_gains_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&EndeffectorTrackingTaskInterface::onSetPgains, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
EndeffectorTrackingTaskInterface::onSetPgains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {"pos.x", "pos.y", "pos.z",
                                     "att.x", "att.y", "att.z"};

  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "eef_tracking.task_error_gain." + params[i],
              p_gains_(i))) {
        task_.setEefGain(p_gains_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(p_gains_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

void ManipulabilityTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "manipulability.gain";
  descr_text = "Gain factor for manipulability cost ";
  descr = hippo_common::param_utils::Description(descr_text);
  gain_ = 1.0;
  gain_ = node_ptr_->declare_parameter(name, gain_, descr);
  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(gain_);
}

void ManipulabilityTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &ManipulabilityTaskInterface::onSetGains, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ManipulabilityTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "manipulability.gain", gain_)) {
      std::lock_guard<std::mutex> lock(*mutex_ptr_);
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void ManipulabilityLimitTaskInterface::initializeTask() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "manipulability_limit.delta";
  descr_text = "Bound for activating constraint";
  descr = hippo_common::param_utils::Description(descr_text);
  double delta = 1.0;
  delta = node_ptr_->declare_parameter(name, delta, descr);

  name = "manipulability_limit.alpha";
  descr_text = "Bound for deactivating constraint again";
  descr = hippo_common::param_utils::Description(descr_text);
  double alpha = 1.0;
  alpha = node_ptr_->declare_parameter(name, alpha, descr);

  name = "manipulability_limit.min";
  descr_text = "Lower limit for manipulability measure ";
  descr = hippo_common::param_utils::Description(descr_text);
  double min = 1.0;
  min = node_ptr_->declare_parameter(name, min, descr);

  name = "manipulability_limit.gain";
  descr_text = "Gain factor for manipulability limit gain ";
  descr = hippo_common::param_utils::Description(descr_text);
  gain_ = 1.0;
  gain_ = node_ptr_->declare_parameter(name, gain_, descr);

  std::lock_guard<std::mutex> lock(*mutex_ptr_);
  task_.initialize(delta, alpha, min, gain_);
}

void ManipulabilityLimitTaskInterface::addParameterCallback() {
  gain_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&ManipulabilityLimitTaskInterface::onSetGains, this,
                std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ManipulabilityLimitTaskInterface::onSetGains(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(*mutex_ptr_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "manipulability_limit.gain", gain_)) {
      task_.setTaskGain(gain_);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(gain_))
                      .c_str());
      break;
    }
  }
  return result;
}

void UVMSKinematicControlInterface::initialize(rclcpp::Node *node_ptr) {
  node_ptr_ = node_ptr;
  initializeController();
  // initializeTask();
}

void UVMSKinematicControlInterface::initializeController() {
  std::vector<param_utils::TFParam> tf_params(param_utils::n_links);
  bool print_load_output = true;
  bool dummy_active, dummy_inertial;
  for (size_t i = 0; i < param_utils::n_links; i++) {
    RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                static_cast<std::string>(param_utils::link_names[i]).c_str());
    ros_param_utils::loadLinkTFParams(
        node_ptr_, dummy_active, dummy_inertial, tf_params[i],
        static_cast<std::string>(param_utils::link_names[i]),
        print_load_output);
  }

  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;
  descr_text = "Decides which projection method is used";
  descr = hippo_common::param_utils::Description(descr_text);
  int algorithm_type;
  algorithm_type = node_ptr_->declare_parameter("algorithm_type", 0, descr);

  descr_text = "Decides if publishers are created for each task";
  descr = hippo_common::param_utils::Description(descr_text);
  publish_task_velocities_ = false;
  publish_task_velocities_ = node_ptr_->declare_parameter(
      "publish_task_velocities", publish_task_velocities_, descr);

  controller_.initialize(tf_params, algorithm_type);

  std::vector<std::string> params = {
      "linear.x",  "linear.y",  "linear.z",  "angular.x", "angular.y",
      "angular.z", "joints.q0", "joints.q1", "joints.q2", "joints.q3"};
  std::vector<std::string> description_suffix = {
      "linear x",  "linear y", "linear z", "angular x", "angular y",
      "angular z", "joints 0", "joints 1", "joints 2",  "joints 3"};
  std::vector<double> default_params;
  default_params.resize(params.size());
  std::fill(default_params.begin(), default_params.end(), 0.5);

  UVMSStateVector velocity_limits;
  for (int i = 0; i < int(params.size()); i++) {
    name = "velocity_limits." + params[i];
    descr_text = "Velocity limits for " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    velocity_limits(i) = default_params[i];
    velocity_limits(i) =
        node_ptr_->declare_parameter(name, velocity_limits(i), descr);
  }
  controller_.setVelocityLimits(-velocity_limits, velocity_limits);

  double accel_smooth_fac_min;
  name = "accel_smooth_fac_min";
  descr_text = "Minimum smoothing factor for jumps in acceleration";
  descr = hippo_common::param_utils::Description(descr_text);
  accel_smooth_fac_min = 0.5;
  accel_smooth_fac_min =
      node_ptr_->declare_parameter(name, accel_smooth_fac_min, descr);

  double accel_smooth_fac_max;
  name = "accel_smooth_fac_max";
  descr_text = "Maximum smoothing factor for jumps in acceleration";
  descr = hippo_common::param_utils::Description(descr_text);
  accel_smooth_fac_max = 0.9;
  accel_smooth_fac_max =
      node_ptr_->declare_parameter(name, accel_smooth_fac_max, descr);

  double delta_accel_min;
  name = "delta_accel_min";
  descr_text =
      "Minimum acceleration difference between time steps to apply smoothing "
      "factor";
  descr = hippo_common::param_utils::Description(descr_text);
  delta_accel_min = 0.3;
  delta_accel_min = node_ptr_->declare_parameter(name, delta_accel_min, descr);

  double delta_accel_max;
  name = "delta_accel_max";
  descr_text =
      "Maximum acceleration difference between time steps to apply smoothing "
      "factor";
  descr = hippo_common::param_utils::Description(descr_text);
  delta_accel_max = 1.0;
  delta_accel_max = node_ptr_->declare_parameter(name, delta_accel_max, descr);

  controller_.setAccelerationSmoothingFactors(accel_smooth_fac_min,
                                              accel_smooth_fac_max,
                                              delta_accel_min, delta_accel_max);
  double eta;
  name = "eta";
  descr_text =
      "Factor for shifting focus onto minimizing control directions in "
      "transition";
  descr = hippo_common::param_utils::Description(descr_text);
  eta = 1.0;
  eta = node_ptr_->declare_parameter(name, eta, descr);
  controller_.setEta(eta);

  bool nullspace_weighting;
  name = "nullspace_weighting";
  descr_text =
      "Decides if weighting is applied to whole Nullspace projection or only "
      "task-wise";
  descr = hippo_common::param_utils::Description(descr_text);
  nullspace_weighting = false;
  nullspace_weighting =
      node_ptr_->declare_parameter(name, nullspace_weighting, descr);
  controller_.setNullSpaceWeighting(nullspace_weighting);

  std::vector<int64_t> task_types;
  ros_param_utils::getParamArray(node_ptr_, task_types, "task_types", {1});
  std::vector<int64_t> task_keys = {TaskKey::joint_limits,
                                    TaskKey::endeffector_tracking};
  ros_param_utils::getParamArray(node_ptr_, task_keys, "task_keys", task_keys);
  if (task_keys.size() != task_types.size()) {
    RCLCPP_ERROR(
        node_ptr_->get_logger(), "%s",
        "Number of given task keys does not match their declared task types!");
    return;
  }

  descr_text = "Damping parameter for SVD regularization";
  descr = hippo_common::param_utils::Description(descr_text);
  double k_svd_damping;
  k_svd_damping = node_ptr_->declare_parameter("svd_damping", 0.01, descr);

  descr_text = "Minimum singular value for SVD regularization";
  descr = hippo_common::param_utils::Description(descr_text);
  double sigma_min = node_ptr_->declare_parameter("sigma_min", 1e-3, descr);

  descr_text = "Parameter for dynamic changing bounds in inequality task";
  descr = hippo_common::param_utils::Description(descr_text);
  double k_alpha_dyn;
  k_alpha_dyn = node_ptr_->declare_parameter("alpha_dyn", 0.02, descr);

  descr_text = "Maximum relative bound to deactivation bound alpha ";
  descr = hippo_common::param_utils::Description(descr_text);
  double k_alpha_max = node_ptr_->declare_parameter("alpha_max", 1.5, descr);

  params = {"pos.x", "pos.y",    "pos.z",    "att.x",    "att.y",
            "att.z", "joint.q0", "joint.q1", "joint.q2", "joint.q3"};
  description_suffix = {"pos x",    "pos y",   "pos z",    "att x",
                        "att y",    "att z",   "joint q0", "joint q1",
                        "joint q2", "joint q3"};
  default_params.resize(params.size());
  std::fill(default_params.begin(), default_params.end(), 1.0);
  for (int i = 0; i < int(params.size()); i++) {
    name = "weighting_matrix." + params[i];
    descr_text =
        "Weighting parameter for motion along " + description_suffix[i];
    descr = hippo_common::param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      weighting_params_(i) = default_params[i];
      weighting_params_(i) =
          node_ptr_->declare_parameter(name, weighting_params_(i), descr);
    }
  }

  for (int task_idx = 0; task_idx < int(task_keys.size()); task_idx++) {
    switch (task_keys[task_idx]) {
      case TaskKey::joint_limits:
        task_interfaces_.push_back(new JointLimitTaskInterface());
        break;
      case TaskKey::endeffector_tracking:
        task_interfaces_.push_back(new EndeffectorTrackingTaskInterface());
        dynamic_cast<EndeffectorTrackingTaskInterface *>(
            task_interfaces_.back())
            ->addReferences(controller_.getPositionTargetReference(),
                            controller_.getAttitudeTargetReference(),
                            controller_.getVelocitiesTargetReference(),
                            controller_.getTrackingMaskReference());
        break;
      case TaskKey::self_collision_ellipse:
        task_interfaces_.push_back(new SelfCollisionEllipseTaskInterface());
        break;
      case TaskKey::auv_attitude_limits:
        task_interfaces_.push_back(new AUVAttitudeLimitTaskInterface());
        break;
      case TaskKey::auv_attitude_inclination_limits:
        task_interfaces_.push_back(
            new AUVAttitudeInclinationLimitTaskInterface());
        break;
      case TaskKey::auv_confined_space:
        task_interfaces_.push_back(new AUVConfinedSpaceTaskInterface());
        break;
      case TaskKey::joint_centering:
        task_interfaces_.push_back(new JointCenteringTaskInterface());
        break;
      case TaskKey::joint_velocity:
        task_interfaces_.push_back(new VelocityTaskInterface());
        break;
      case TaskKey::manipulability:
        task_interfaces_.push_back(new ManipulabilityTaskInterface());
        break;
      case TaskKey::manipulability_limit:
        task_interfaces_.push_back(new ManipulabilityLimitTaskInterface());
        break;
      case TaskKey::joint_limits_desired:
        task_interfaces_.push_back(new JointLimitDesiredTaskInterface());
        break;
      case TaskKey::restricting_plane:
        task_interfaces_.push_back(new RestrictingPlaneTaskInterface());
        break;
      default:
        RCLCPP_ERROR(
            node_ptr_->get_logger(), "%s",
            ("Chosen task key is " + std::to_string(task_keys[task_idx]) +
             " which is not a valid task key!")
                .c_str());
        break;
    }
  }

  for (int task_idx = 0; task_idx < int(task_keys.size()); task_idx++) {
    task_interfaces_[task_idx]->getTaskPtr()->setIsContinuous(
        algorithm_type == AlgorithmType::continuous);
    task_interfaces_[task_idx]->getTaskPtr()->setSVDDamping(k_svd_damping);
    task_interfaces_[task_idx]->getTaskPtr()->setMinSingularValue(sigma_min);
    task_interfaces_[task_idx]->getTaskPtr()->setKAlphaDyn(k_alpha_dyn);
    task_interfaces_[task_idx]->getTaskPtr()->setKAlphaMax(k_alpha_max);
    task_interfaces_[task_idx]->initialize(
        node_ptr_, &mutex_, int(task_keys[task_idx]), publish_task_velocities_);
    task_interfaces_[task_idx]->addReferenceModel(
        controller_.getKinematicsPtr());
    controller_.addTask(task_interfaces_[task_idx]->getTaskPtr(),
                        TaskType(task_types[task_idx]));
  }

  // set here, as this also sets the weighting parameters for the task
  // velocities for the last velocity task, which must be initialized first!
  for (int i = 0; i < weighting_params_.rows(); i++) {
    controller_.setWeightingParam(weighting_params_(i), i);
  }

  for (TaskInterface *task_interface : task_interfaces_) {
    task_interface->addParameterCallback();
  }
  addParameterCallback();
}

void UVMSKinematicControlInterface::addParameterCallback() {
  svd_param_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&UVMSKinematicControlInterface::onSetSVDParam, this,
                std::placeholders::_1));
  weighting_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&UVMSKinematicControlInterface::onSetWeightingParams, this,
                std::placeholders::_1));
  eta_param_cb_handle_ = node_ptr_->add_on_set_parameters_callback(std::bind(
      &UVMSKinematicControlInterface::onSetEta, this, std::placeholders::_1));
  accel_smoothing_cb_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&UVMSKinematicControlInterface::onSetAccelSmoothingFactors,
                this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
UVMSKinematicControlInterface::onSetSVDParam(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double sigma_min, k_svd_damping;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(parameter, "sigma_min",
                                                 sigma_min)) {
      for (TaskInterface *task_interface : task_interfaces_) {
        task_interface->getTaskPtr()->setMinSingularValue(sigma_min);
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(sigma_min))
                      .c_str());
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "svd_damping", k_svd_damping)) {
      for (TaskInterface *task_interface : task_interfaces_) {
        task_interface->getTaskPtr()->setSVDDamping(k_svd_damping);
      }
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(k_svd_damping))
                      .c_str());
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult
UVMSKinematicControlInterface::onSetWeightingParams(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<std::string> params = {
      "pos.x", "pos.y",    "pos.z",    "att.x",    "att.y",
      "att.z", "joint.q0", "joint.q1", "joint.q2", "joint.q3"};
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < int(params.size()); i++) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, "weighting_matrix." + params[i],
              weighting_params_(i))) {
        controller_.setWeightingParam(weighting_params_(i), i);
        RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(weighting_params_(i)))
                        .c_str());
        break;
      }
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult
UVMSKinematicControlInterface::onSetEta(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double eta;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(parameter, "eta", eta)) {
      controller_.setEta(eta);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(eta))
                      .c_str());
      break;
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult
UVMSKinematicControlInterface::onSetAccelSmoothingFactors(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double accel_smoothing_factor_min, accel_smoothing_factor_max;
  for (const rclcpp::Parameter &parameter : parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "accel_smooth_fac_min", accel_smoothing_factor_min)) {
      controller_.setAccelSmoothingFactorMin(accel_smoothing_factor_min);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(accel_smoothing_factor_min))
                      .c_str());
      break;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "accel_smooth_fac_max", accel_smoothing_factor_max)) {
      controller_.setAccelSmoothingFactorMax(accel_smoothing_factor_max);
      RCLCPP_INFO(node_ptr_->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(accel_smoothing_factor_max))
                      .c_str());
      break;
    }
  }
  return result;
}

void UVMSKinematicControlInterface::getEndeffectorPose(
    geometry_msgs::msg::Pose &out_msg) {
  controller_.getEefPosition(out_msg.position.x, out_msg.position.y,
                             out_msg.position.z);
  controller_.getEefAttitude(out_msg.orientation.w, out_msg.orientation.x,
                             out_msg.orientation.y, out_msg.orientation.z);
}

void UVMSKinematicControlInterface::getEndeffectorPose(
    const nav_msgs::msg::Odometry &auv_msg,
    const sensor_msgs::msg::JointState &manipulator_msg,
    geometry_msgs::msg::Pose &out_msg) {
  Eigen::Vector3d pos;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.position, pos);
  Eigen::Quaterniond att;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.orientation, att);
  StateVector q;
  for (size_t i = 0; i < n_active_joints; i++) {
    auto entry =
        std::find(manipulator_msg.name.begin(), manipulator_msg.name.end(),
                  static_cast<std::string>(param_utils::joint_names[i]));
    if (entry != std::end(manipulator_msg.name)) {
      size_t idx = std::distance(manipulator_msg.name.begin(), entry);
      q(int(i)) = manipulator_msg.position[idx];
    } else {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Requested joint not found in published states");
      return;
    }
  }
  controller_.updateEef(pos, att, q);
  controller_.getEefPosition(out_msg.position.x, out_msg.position.y,
                             out_msg.position.z);
  controller_.getEefAttitude(out_msg.orientation.w, out_msg.orientation.x,
                             out_msg.orientation.y, out_msg.orientation.z);
}

void UVMSKinematicControlInterface::onSetpointTimeout() {}

void UVMSKinematicControlInterface::setSetpointTarget(
    const hippo_control_msgs::msg::ControlTarget::SharedPtr _msg) {
  Eigen::Matrix<bool, 6, 1> mask = Eigen::Matrix<bool, 6, 1>::Ones();
  if ((_msg->mask & _msg->IGNORE_POSITION_X) == _msg->IGNORE_POSITION_X) {
    mask(0) = false;
  }
  if ((_msg->mask & _msg->IGNORE_POSITION_Y) == _msg->IGNORE_POSITION_Y) {
    mask(1) = false;
  }
  if ((_msg->mask & _msg->IGNORE_POSITION_Z) == _msg->IGNORE_POSITION_Z) {
    mask(2) = false;
  }
  if ((_msg->mask & _msg->IGNORE_ATTITUDE_X) == _msg->IGNORE_ATTITUDE_X) {
    mask(3) = false;
  }
  if ((_msg->mask & _msg->IGNORE_ATTITUDE_Y) == _msg->IGNORE_ATTITUDE_Y) {
    mask(4) = false;
  }
  if ((_msg->mask & _msg->IGNORE_ATTITUDE_Z) == _msg->IGNORE_ATTITUDE_Z) {
    mask(5) = false;
  }
  Eigen::Vector3d pos_des, vel_des, ang_vel_des;
  Eigen::Quaterniond att_des;
  hippo_common::convert::RosToEigen(_msg->position, pos_des);
  hippo_common::convert::RosToEigen(_msg->attitude, att_des);
  hippo_common::convert::RosToEigen(_msg->velocity, vel_des);
  hippo_common::convert::RosToEigen(_msg->angular_velocity, ang_vel_des);
  std::lock_guard<std::mutex> lock(mutex_);
  controller_.setControlTarget(pos_des, att_des, vel_des, ang_vel_des);
  controller_.setTrackingMask(mask);
}

void UVMSKinematicControlInterface::setSetpointTargetForward(
    const hippo_control_msgs::msg::ControlTarget::SharedPtr _msg,
    const double &dt) {
  Eigen::Vector3d pos_des, vel_des, ang_vel_des;
  Eigen::Quaterniond att_des;
  hippo_common::convert::RosToEigen(_msg->position, pos_des);
  hippo_common::convert::RosToEigen(_msg->attitude, att_des);
  hippo_common::convert::RosToEigen(_msg->velocity, vel_des);
  hippo_common::convert::RosToEigen(_msg->angular_velocity, ang_vel_des);
  std::lock_guard<std::mutex> lock(mutex_);
  controller_.setControlTargetForward(pos_des, att_des, vel_des, ang_vel_des,
                                      dt);
}

void UVMSKinematicControlInterface::getControllerOutput(
    const nav_msgs::msg::Odometry &auv_msg,
    const sensor_msgs::msg::JointState &manipulator_msg,
    geometry_msgs::msg::TwistStamped &out_auv_cmds,
    alpha_msgs::msg::JointData &out_manipulator_cmds) {
  Eigen::Vector3d pos;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.position, pos);
  Eigen::Quaterniond att;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.orientation, att);
  StateVector q;
  for (size_t i = 0; i < n_active_joints; i++) {
    auto entry =
        std::find(manipulator_msg.name.begin(), manipulator_msg.name.end(),
                  static_cast<std::string>(param_utils::joint_names[i]));
    if (entry != std::end(manipulator_msg.name)) {
      size_t idx = std::distance(manipulator_msg.name.begin(), entry);
      q(int(i)) = manipulator_msg.position[idx];
    } else {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Requested joint not found in published states");
      return;
    }
  }
  param_utils::UVMSStateVector cmd_vel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // auto start = std::chrono::high_resolution_clock::now();
    controller_.getControlCmd(pos, att, q, cmd_vel);
    // std::cout <<
    // std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()
    // - start).count()*1e-9 << std::endl;
  }
  const Eigen::Ref<const Eigen::Vector3d> ref_auv_vel = cmd_vel.segment<3>(0);
  const Eigen::Ref<const Eigen::Vector3d> ref_auv_ang_vel =
      cmd_vel.segment<3>(3);
  hippo_common::convert::EigenToRos(ref_auv_vel, out_auv_cmds.twist.linear);
  hippo_common::convert::EigenToRos(ref_auv_ang_vel,
                                    out_auv_cmds.twist.angular);
  std::copy(cmd_vel.segment<n_active_joints>(6).begin(),
            cmd_vel.segment<n_active_joints>(6).end(),
            out_manipulator_cmds.data.begin());
  if (publish_task_velocities_) {
    for (auto &task_interface : task_interfaces_) {
      task_interface->publishCmds();
    }
  }
}

void UVMSKinematicControlInterface::getControllerOutputWithDerivative(
    const double &dt, const nav_msgs::msg::Odometry &auv_msg,
    const sensor_msgs::msg::JointState &manipulator_msg,
    geometry_msgs::msg::Twist &out_auv_vel_cmds,
    geometry_msgs::msg::Twist &out_auv_acc_cmds,
    alpha_msgs::msg::JointData &out_manipulator_cmds) {
  Eigen::Vector3d pos;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.position, pos);
  Eigen::Quaterniond att;
  hippo_common::convert::RosToEigen(auv_msg.pose.pose.orientation, att);
  StateVector q;
  for (size_t i = 0; i < n_active_joints; i++) {
    auto entry =
        std::find(manipulator_msg.name.begin(), manipulator_msg.name.end(),
                  static_cast<std::string>(param_utils::joint_names[i]));
    if (entry != std::end(manipulator_msg.name)) {
      size_t idx = std::distance(manipulator_msg.name.begin(), entry);
      q(int(i)) = manipulator_msg.position[idx];
    } else {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Requested joint not found in published states");
      return;
    }
  }
  param_utils::UVMSStateVector cmd_vel, cmd_acc;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // auto start = std::chrono::high_resolution_clock::now();
    controller_.getControlCmdWithDerivative(dt, pos, att, q, cmd_vel, cmd_acc);
    // std::cout <<
    // std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()
    // - start).count()*1e-9 << std::endl;
  }

  const Eigen::Ref<const Eigen::Vector3d> ref_auv_vel = cmd_vel.segment<3>(0);
  const Eigen::Ref<const Eigen::Vector3d> ref_auv_ang_vel =
      cmd_vel.segment<3>(3);
  hippo_common::convert::EigenToRos(ref_auv_vel, out_auv_vel_cmds.linear);
  hippo_common::convert::EigenToRos(ref_auv_ang_vel, out_auv_vel_cmds.angular);
  const Eigen::Ref<const Eigen::Vector3d> ref_auv_acc = cmd_acc.segment<3>(0);
  const Eigen::Ref<const Eigen::Vector3d> ref_auv_ang_acc =
      cmd_acc.segment<3>(3);
  hippo_common::convert::EigenToRos(ref_auv_acc, out_auv_acc_cmds.linear);
  hippo_common::convert::EigenToRos(ref_auv_ang_acc, out_auv_acc_cmds.angular);
  std::copy(cmd_vel.segment<n_active_joints>(6).begin(),
            cmd_vel.segment<n_active_joints>(6).end(),
            out_manipulator_cmds.data.begin());
  if (publish_task_velocities_) {
    for (auto &task_interface : task_interfaces_) {
      task_interface->publishCmds();
    }
  }
}
}  // namespace uvms_kin_ctrl
