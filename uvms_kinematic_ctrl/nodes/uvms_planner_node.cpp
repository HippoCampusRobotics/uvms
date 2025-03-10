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

#include "uvms_planner_node.hpp"

using std::placeholders::_1;
using namespace std::chrono; // used for time related tasks
using namespace alpha_ctrl;
using namespace uvms_traj_gen;

namespace uvms_kin_ctrl {


UVMSPlannerNode::UVMSPlannerNode()
    : Node("uvms_planner_node") {
  RCLCPP_INFO(this->get_logger(), "Declaring parameters.");
  initServices();
  initPublishers(); 
  declareParams(); 
  initTimers();
  initSubscriptions();

}

void UVMSPlannerNode::initPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "goal_pose_endeffector";
  eef_pose_goal_pub_ =
      create_publisher<hippo_msgs::msg::PoseStampedNumbered>(topic, qos);
  
  topic = "planner_control_mode";
  control_mode_pub_ =
      create_publisher<std_msgs::msg::Int64>(topic, qos);

  topic = "gripper_mode";
  gripper_mode_pub_ = create_publisher<std_msgs::msg::Int64>(topic, qos);

  topic = "planner_mode";
  planner_mode_pub_ = create_publisher<std_msgs::msg::Int64>(topic, qos);

}

void UVMSPlannerNode::declareParams() {
  ros_param_utils::getParam(this, offset_dist_,
                            "offset_distance", 0.05); //unit = m

  
  ros_param_utils::getParam(this, number_rounds_,
                            "number_test_rounds", 5);

  // must be the same as in experiment / simulation
  ros_param_utils::getParam(this, cylinder_height_,
                            "cylinder_height", 0.12); //unit = m

  ros_param_utils::getParam(this, automated_,
                            "automated", false); //unit = m

  ros_param_utils::getParam(this, discharge_height_,
                            "discharge_height", 0.03); //unit = m

  ros_param_utils::getParam(this, lift_surface_dist_,
                            "lift_surface_dist", 0.1); //unit = m

  counter_rounds_ = 0;

  planner_mode_ = PlannerMode::finishing_pick_and_place;

  // later detected by camera and published to topic
//   object_plane_normal_ = {0.0, 0.0, 1.0};

  // the placement pose should be set to the coordinate frame of the
  // hole or similar, where the object is suppose to go through / be placed
  // wenn ich ein Loch in einer Platte hätte, läge die Pose in der Mitte dieses Loches
//   place_pos_.x() = 1.0;
//   place_pos_.y() = 1.0;
//   place_pos_.z() = -1;

//   place_att_.w() = 1;
//   place_att_.x() = 0;
//   place_att_.y() = 0;
//   place_att_.z() = 0;

  pose_out_msg_.header.frame_id =
                hippo_common::tf2_utils::frame_id::kInertialName;
}

void UVMSPlannerNode::initTimers() {
  trajectory_status_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&UVMSPlannerNode::onStatusTrajectoryTimeout, this));

  gripper_status_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(500),
      std::bind(&UVMSPlannerNode::onStatusGripperTimeout, this));

  double freq = 50;

  run_planner_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(int(1000 * 1.0 / freq)),
      std::bind(&UVMSPlannerNode::runPlanner, this));     
}

void UVMSPlannerNode::initSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

//   topic = "pose_object";
//   object_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
//       topic, qos, std::bind(&UVMSPlannerNode::onPoseObject, this, _1));

  topic = "pose_endeffector";
  eef_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, qos, std::bind(&UVMSPlannerNode::onPoseEndeffector, this, _1));


  topic = "traj_status";
  traj_status_sub_ = create_subscription<std_msgs::msg::Int64>(
      topic, qos,
      std::bind(&UVMSPlannerNode::onStatusTrajectory, this, _1));

  topic = "gripper_status";
  gripper_status_sub_ = create_subscription<std_msgs::msg::Int64>(
      topic, qos,
      std::bind(&UVMSPlannerNode::onStatusGripper, this, _1));

  //starting a topic with / will look for absolut topic name on global level, outside the current namespace
//   topic = "/platform/ground_truth/odometry";
//   platform_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//       topic, qos,
//       std::bind(&UVMSPlannerNode::onOdometryPlatform, this, _1));

//   topic = "/cylinder/ground_truth/odometry";
//   cylinder_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//       topic, qos,
//       std::bind(&UVMSPlannerNode::onOdometryCylinder, this, _1));

//   topic = "/cylinder_holder/ground_truth/odometry";
//   cylinder_holder_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//       topic, qos,
//       std::bind(&UVMSPlannerNode::onOdometryCylinderHolder, this, _1));
}

void UVMSPlannerNode::initServices() {
    std::string name;

    name = "pap_test_start";
    test_run_service_ = create_service<std_srvs::srv::SetBool>(
        name, std::bind(&UVMSPlannerNode::serveTestRun, this,
                        std::placeholders::_1, std::placeholders::_2));
  }

void UVMSPlannerNode::serveTestRun(const std_srvs::srv::SetBool_Request::SharedPtr _request,
                   std_srvs::srv::SetBool_Response::SharedPtr _response) {
    if (_request->data) {
      if (test_run_activated_) {
        _response->message = "Already activated.";
        _response->success = false;
        return;
      } else {
        test_run_activated_ = _request->data;
        RCLCPP_INFO(get_logger(), "Activating Pick and Place Test Run.");
        _response->message = "Activated";
        _response->success = true;
        if (automated_) {
            RCLCPP_INFO(get_logger(), "Activating automated Test Run.");
            automated_check_ = true;    
        }
      }
    } else {
      if (test_run_activated_) {
        RCLCPP_INFO(get_logger(), "Deactivating Pick and Place Test Run.");
        test_run_activated_ = false;
        _response->message = "Deactivated.";
        _response->success = true;
      } else {
        _response->message = "Already deactivated.";
        _response->success = false;
      }
    }
  }

void UVMSPlannerNode::onStatusTrajectoryTimeout() {
  if (trajectory_status_timed_out_) return;

  trajectory_status_timed_out_ = true;

  RCLCPP_WARN(get_logger(), 
    "Trajectory status time out. Declaring as undefined");
}

void UVMSPlannerNode::onStatusGripperTimeout() {
  if (gripper_status_timed_out_) return;

  gripper_status_timed_out_ = true;

  RCLCPP_WARN(get_logger(), 
    "Gripper status time out. Declaring as undefined");
}

// instead onOdometryCylinder
// void UVMSPlannerNode::onPoseObject(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
//     hippo_common::convert::RosToEigen(_msg->pose.position, object_pos_);
//     hippo_common::convert::RosToEigen(_msg->pose.orientation, object_att_);

//     if (!got_first_object_pose_) got_first_object_pose_ = true; 
// }

void UVMSPlannerNode::onPoseEndeffector(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
    hippo_common::convert::RosToEigen(_msg->pose.position, eef_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.orientation, eef_att_);

    if (!got_first_eef_pose_) got_first_eef_pose_ = true; 
}

void UVMSPlannerNode::onStatusTrajectory(const std_msgs::msg::Int64::SharedPtr _msg) {
    traj_status_ = _msg->data;

    if (!got_first_traj_status_) got_first_traj_status_ = true;
}

void UVMSPlannerNode::onStatusGripper(const std_msgs::msg::Int64::SharedPtr _msg) {
    gripper_status_ = _msg->data;

    if (!got_first_gripper_status_) got_first_gripper_status_ = true;
}

void UVMSPlannerNode::onOdometryCylinder(const nav_msgs::msg::Odometry::SharedPtr _msg) {
    if (got_first_object_pose_) return;

    hippo_common::convert::RosToEigen(_msg->pose.pose.position, object_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, object_att_);
    
    object_plane_normal_.x() = 2*(object_att_.x()*object_att_.z()+object_att_.w()*object_att_.y());
    object_plane_normal_.y() = 2*(object_att_.y()*object_att_.z()-object_att_.w()*object_att_.x());
    object_plane_normal_.z() = 1-2*(object_att_.x()*object_att_.x()+object_att_.y()*object_att_.y());

    RCLCPP_INFO(get_logger(), "Changed object position to: (%.2f, %.2f, %.2f)", object_pos_.x(), object_pos_.y(), object_pos_.z());
    RCLCPP_INFO(get_logger(), "Changed object normal axis to: (%.2f, %.2f, %.2f)", object_plane_normal_.x(), object_plane_normal_.y(), object_plane_normal_.z());

    if (!got_first_object_pose_) got_first_object_pose_ = true;
}

void UVMSPlannerNode::onOdometryPlatform(const nav_msgs::msg::Odometry::SharedPtr _msg) {
    if (got_first_platform_pose_) return;
    
    hippo_common::convert::RosToEigen(_msg->pose.pose.position, place_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, place_att_);
    
    place_plane_normal_.x() = 2*(place_att_.x()*place_att_.z()+place_att_.w()*place_att_.y());
    place_plane_normal_.y() = 2*(place_att_.y()*place_att_.z()-place_att_.w()*place_att_.x());
    place_plane_normal_.z() = 1-2*(place_att_.x()*place_att_.x()+place_att_.y()*place_att_.y());

    RCLCPP_INFO(get_logger(), "Changed platform position to: (%.2f, %.2f, %.2f)", place_pos_.x(), place_pos_.y(), place_pos_.z());
    RCLCPP_INFO(get_logger(), "Changed platform normal axis to: (%.2f, %.2f, %.2f)", place_plane_normal_.x(), place_plane_normal_.y(), place_plane_normal_.z());

    if (!got_first_platform_pose_) got_first_platform_pose_ = true;
}

void UVMSPlannerNode::onOdometryCylinderHolder(const nav_msgs::msg::Odometry::SharedPtr _msg) {
    if (got_first_cylinder_holder_pose_) return;
    
    hippo_common::convert::RosToEigen(_msg->pose.pose.position, cylinder_holder_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, cylinder_holder_att_);
    
    cylinder_holder_plane_normal_.x() = 2*(cylinder_holder_att_.x()*cylinder_holder_att_.z()+cylinder_holder_att_.w()*cylinder_holder_att_.y());
    cylinder_holder_plane_normal_.y() = 2*(cylinder_holder_att_.y()*cylinder_holder_att_.z()-cylinder_holder_att_.w()*cylinder_holder_att_.x());
    cylinder_holder_plane_normal_.z() = 1-2*(cylinder_holder_att_.x()*cylinder_holder_att_.x()+cylinder_holder_att_.y()*cylinder_holder_att_.y());

    RCLCPP_INFO(get_logger(), "Changed cylinder holder position to: (%.2f, %.2f, %.2f)", cylinder_holder_pos_.x(), cylinder_holder_pos_.y(), cylinder_holder_pos_.z());
    RCLCPP_INFO(get_logger(), "Changed cylinder holder normal axis to: (%.2f, %.2f, %.2f)", cylinder_holder_plane_normal_.x(), cylinder_holder_plane_normal_.y(), cylinder_holder_plane_normal_.z());

    if (!got_first_cylinder_holder_pose_) got_first_cylinder_holder_pose_ = true;
}

void UVMSPlannerNode::vectorPlaneProjection(const Eigen::Vector3d &_plane_normal, const Eigen::Vector3d &_vector, Eigen::Vector3d &_proj) {
    Eigen::Vector3d plane_normal_n = _plane_normal.normalized();
    Eigen::Vector3d proj_plane_normal = _vector.dot(plane_normal_n) * plane_normal_n;
    _proj = _vector - proj_plane_normal;
}

// computes perpendicular vector to base_vector that is closest to the given vector
void UVMSPlannerNode::vectorVectorProjection(const Eigen::Vector3d &_base_vector, const Eigen::Vector3d &_vector, Eigen::Vector3d &_proj) {
    Eigen::Vector3d base_vector_n = _base_vector.normalized();
    Eigen::Vector3d proj_base_vector = _vector.dot(base_vector_n) * base_vector_n;
    _proj = _vector - proj_base_vector;
}

// void UVMSPlannerNode::rotation2Quaternion(const Eigen::Matrix3d _rotation, Eigen::Quaterniond &_quaternion) {
//     double r11 = _rotation(0,0);
//     double r12 = _rotation(0,1);
//     double r13 = _rotation(0,2);
//     double r21 = _rotation(1,0);
//     double r22 = _rotation(1,1);
//     double r23 = _rotation(1,2);
//     double r31 = _rotation(2,0);
//     double r32 = _rotation(2,1);
//     double r33 = _rotation(2,2);
    
    
//     _quaternion.w() = 0.25 * std::sqrt(std::pow(r11 + r22 + r33 + 1, 2) + 
//                                 std::pow(r32 - r23, 2) + 
//                                 std::pow(r13 - r31, 2) +
//                                 (r21 - r12));

//     double x = 0.25 * std::sqrt(std::pow(r32 - r23, 2) + 
//                                 std::pow(r11 - r22 - r33 + 1, 2) + 
//                                 std::pow(r21 + r12, 2) +
//                                 (r31 + r13));
//     _quaternion.x() = std::copysign(x, r32 - r23);
                        

//     double y = 0.25 * std::sqrt(std::pow(r13 - r31, 2) + 
//                                 std::pow(r21 + r12, 2) + 
//                                 std::pow(r22 - r11 - r33 + 1, 2) +
//                                 (r32 + r23));
//     _quaternion.y() = std::copysign(y, r13 - r31);

//     double z = 0.25 * std::sqrt(std::pow(r21 - r12, 2) + 
//                                 std::pow(r31 + r13, 2) + 
//                                 std::pow(r32 + r23, 2) +
//                                 (r33 - r11 - r22 + 1));
//     _quaternion.z() = std::copysign(z, r21 - r12);
// }

void UVMSPlannerNode::getRotationFromVector(const Eigen::Vector3d &_x_eff, const Eigen::Vector3d &_z_eff, Eigen::Matrix3d &_rotation) {
    Eigen::Vector3d x_eff_head;
    Eigen::Vector3d y_eff_head;
    Eigen::Vector3d z_eff_head;
    if (std::abs(_x_eff.dot(_z_eff)) > eps_orthogonal_) {
        RCLCPP_INFO(get_logger(), "Reference x- and z-axis are not perpendicular. Generating new reference for z, that is perpendicular to x and the closest to the old z");
        vectorVectorProjection(_x_eff, _z_eff, z_eff_head);
        z_eff_head.normalize();
    } else {
        z_eff_head = _z_eff.normalized();
    }

    x_eff_head = _x_eff.normalized();

    y_eff_head = z_eff_head.cross(x_eff_head); // z cross x
    
    _rotation.col(0) = x_eff_head;
    _rotation.col(1) = y_eff_head;
    _rotation.col(2) = z_eff_head;

    // std::stringstream ss;
    // ss << _rotation;
    // // Logging the whole matrix as a string
    // RCLCPP_INFO(this->get_logger(), "Matrix:\n%s", ss.str().c_str());
    // // applying 30 degree rotation, since gripper is rotated by 30 degrees to its frame by default
    // // _rotation = _rotation * rotation_x_60_;

    // std::stringstream sss;
    

    _rotation = _rotation * rotation_x_60_;
    // sss << _rotation;
    // // Logging the whole matrix as a string
    // RCLCPP_INFO(this->get_logger(), "Matrix * 30 degree:\n%s", sss.str().c_str());
}

Eigen::Quaterniond UVMSPlannerNode::alignZAxes(const Eigen::Quaterniond& qA, const Eigen::Quaterniond& qB) {
    // Step 1: Extract the z-axes of both frames A and B
    Eigen::Vector3d zA = qA * Eigen::Vector3d(0, 0, 1); // z-axis of frame A
    Eigen::Vector3d zB = qB * Eigen::Vector3d(0, 0, 1); // z-axis of frame B

    // Step 2: Project the z-axes onto the XY-plane (ignore the z component)
    Eigen::Vector2d zA_xy(zA.x(), zA.y());
    Eigen::Vector2d zB_xy(zB.x(), zB.y());

    // Step 3: Normalize the projected vectors
    zA_xy.normalize();
    zB_xy.normalize();

    // Step 4: Calculate the angle between the two projected vectors
    double angleA = std::atan2(zA_xy.y(), zA_xy.x());
    double angleB = std::atan2(zB_xy.y(), zB_xy.x());
    double theta = angleB - angleA; // The rotation angle around the z-axis

    // Step 5: Create a quaternion for the rotation around the z-axis
    Eigen::Quaterniond qZ(Eigen::AngleAxisd(theta, Eigen::Vector3d(0, 0, 1)));

    return qZ;
    // Step 6: Apply the z-axis rotation to frame A
    // qA = qZ * qA; // Update the orientation of frame A

    // The new orientation of frame A is now stored in qA
}



void UVMSPlannerNode::runPlanner() {
    if (!got_first_eef_pose_) return; // !got_first_traj_status_ || !got_first_gripper_status_

    switch(planner_mode_) {
        case PlannerMode::undefined:
            planner_mode_ = PlannerMode::waiting_for_all_processes;
            break;
        case PlannerMode::waiting_for_all_processes:
            // alle subs müssen die erste nachricht erhalten haben und traj sttaus muss 15 sein,
            // dmait der bluerov in seiner Ausgangsposition ist
            // gripper soll geschlossen sein
            gripper_mode_ = GripperMode::close;
            traj_mode_ = TrajMode::keep_eef_pose;
            
            // automated_check_ becomes only true after ther test run service has been called at least once
            if (automated_check_) {
                    // RCLCPP_INFO(get_logger(), "Activating Pick and Place Test Run.");
                    test_run_activated_ = true;
            }

            // if (gripper_status_ == GripperStatus::closed && traj_status_ == TrajStatus::waiting_for_planner && test_run_activated_) {
            if (traj_status_ == TrajStatus::waiting_for_planner && test_run_activated_) {
                RCLCPP_INFO(get_logger(), "Starting next test rotation\n");
                // as soon as UVMS is in initial position then object and platform is probbaly the most visible
                // hence allowing for new object and platform data happens just here
                
                
                
                got_first_platform_pose_ = true;
                got_first_object_pose_ = true;
                got_first_cylinder_holder_pose_ = true;
                planner_mode_ = PlannerMode::looking_for_object;
            }
            break;
        case PlannerMode::looking_for_object:
            // if (got_first_object_pose_ && got_first_platform_pose_ && got_first_cylinder_holder_pose_) {
            //     Eigen::Vector3d dist_place = place_pos_ - object_pos_;
            //     Eigen::Vector3d dist_holder = cylinder_holder_pos_ - object_pos_;
            //     if ((object_pos_.z() < place_pos_.z() && object_pos_.z() < cylinder_holder_pos_.z()) || object_pos_.z() < -1.4) {
            //         //check, if object is even above the placement and holder, in order for it to be reachable by the UVMS
            //         RCLCPP_INFO(get_logger(), "Object too low in an unfeasible position!\n");
            //         RCLCPP_INFO(get_logger(), "Terminating test run!\n");
            //         planner_mode_ = PlannerMode::done;
            //     }
            //     if (dist_place.norm() > dist_holder.norm()) {
            //         dropping_pos_ = place_pos_;
            //         dropping_att_ = place_att_;
            //         dropping_plane_normal_ = place_plane_normal_;
            //     } else {
            //         dropping_pos_ = cylinder_holder_pos_;
            //         dropping_att_ = cylinder_holder_att_;
            //         dropping_plane_normal_ = cylinder_holder_plane_normal_;
            //     }
            // } else {
            //     break;
            // }

            // hier April Tag Lokalisierung o.ä. starten
            // sobald absolut/relativ pose von Object weiter
            // zunächst ist absolut Pose von Objekt bekannt
            if (got_first_object_pose_) {
                std::lock_guard<std::mutex> lock(mutex_);
                Eigen::Vector3d direction = object_pos_ - eef_pos_;
                Eigen::Vector3d projection;
                Eigen::Matrix3d rotation_matrix;

                vectorPlaneProjection(object_plane_normal_, direction, projection);
                // eef_pos_des_ = object_pos_ - offset_dist_ * projection.normalized();
                eef_pos_des_ = object_pos_ + (offset_dist_ + 0.5*cylinder_height_) * object_plane_normal_.normalized(); //normal vector alwyas in the preferred direction
                
                projection_ = projection.normalized();

                // getRotationFromVector(x-axis gripper, z-axis gripper, rotation matrix for this configuration)
                getRotationFromVector(projection, object_plane_normal_, rotation_matrix);
                // rotation2Quaternion(rotation_matrix, eef_att_des_);
                Eigen::Quaterniond quat{rotation_matrix};
                eef_att_des_ = quat;

                counter_msg_pose_des_++;
                traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::approaching_offset_distance;
            }
            break;
        case PlannerMode::approaching_offset_distance:
            // die gefundene absolute pose des Objekts wird als goal pose gepublished und dort hin gefahren
            // traj_mode_ = TrajMode::keep_eef_pose;

            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
            
            eef_pose_goal_pub_->publish(pose_out_msg_);
            
            if (traj_status_ == TrajStatus::reached_goal) {

                eef_pos_des_ = object_pos_;
                counter_msg_pose_des_++;
                next_planner_mode_ = PlannerMode::approaching_object;
                next_traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::prepare_gripping:
            gripper_mode_ = GripperMode::open;

            if (gripper_status_ == GripperStatus::opened) {
                std::lock_guard<std::mutex> lock(mutex_);
                eef_pos_des_ = object_pos_;
                // eef_att_des_ remains the same
                // eef_pos_des_ = object_pos_ - 0.02 * object_plane_normal_.normalized();

                counter_msg_pose_des_++;
                traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::approaching_object;
            }
            break;
        case PlannerMode::approaching_object:
            // traj_mode_ = TrajMode::keep_eef_pose;

            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
        
            eef_pose_goal_pub_->publish(pose_out_msg_);

            if (traj_status_ == TrajStatus::reached_goal) {
                next_planner_mode_ = PlannerMode::releasing_object;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::gripping_object:
            gripper_mode_ = GripperMode::close;
            
            if (gripper_status_ == GripperStatus::gripped) { // ab || wieder weg machen  ???
                std::lock_guard<std::mutex> lock(mutex_);
                eef_pos_des_ = eef_pos_des_ + lift_surface_dist_ * object_plane_normal_.normalized();  //asuming world z-axis downwards
                // eef_att_des_ remains the same
                // eef_pos_des_ = eef_pos_des_ + (offset_dist_ + cylinder_height_) * object_plane_normal_.normalized();
                // eef_pos_des_ = eef_pos_ + lift_surface_dist_;

                counter_msg_pose_des_++;
                traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::lift_from_surface;
            } else if (gripper_status_ == GripperStatus::closed) {
                // in case the objet was not gripped
                // reset booleans to allow for new object and platform pose data
                // got_first_platform_pose_ = false;
                // got_first_object_pose_ = false;

                test_run_activated_ = false;
                RCLCPP_INFO(get_logger(), "Deactivating Pick and Place Test Run.");

                traj_mode_ = TrajMode::go_to_starting_position;
                planner_mode_ = PlannerMode::waiting_for_all_processes;
            }
            break;
        case PlannerMode::lift_from_surface:
            // traj_mode_ = TrajMode::keep_eef_pose;

            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
        
            eef_pose_goal_pub_->publish(pose_out_msg_);

            // ########################################## (this steps in here works, but will not correct positional grip deviation)
            // without approaching platform with regards to the gripped cylinder pose
            // if (traj_status_ == TrajStatus::reached_goal) { //approaching platform with respect to eef frame
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     direction_2_place_ = dropping_pos_ - eef_pos_;
            //     Eigen::Vector3d projection;
            //     Eigen::Vector3d plane_normal = dropping_plane_normal_; //Lage des Objekts im Raum soll später durch Objekterkennung bestimmt werden
            //     Eigen::Matrix3d rotation_matrix;

            //     eef_pos_des_ = dropping_pos_ + (offset_dist_ + 0.5*cylinder_height_) * dropping_plane_normal_.normalized(); //normal vector alwyas in the preferred direction

            //     vectorPlaneProjection(plane_normal, direction_2_place_, projection);
            //     // getRotationFromVector(x-axis gripper, z-axis gripper, rotation matrix for this configuration)
            //     getRotationFromVector(projection, plane_normal, rotation_matrix);
            //     // rotation2Quaternion(rotation_matrix, eef_att_des_);
            //     Eigen::Quaterniond quat(rotation_matrix);
            //     eef_att_des_ = quat;

            //     counter_msg_pose_des_++;

            //     next_planner_mode_ = PlannerMode::approach_placement_offset_pose;
            //     next_traj_mode_ = TrajMode::new_eff_trajectory;
            //     planner_mode_ = PlannerMode::transition_after_reaching_goal;
            // }
            // ##########################################

            // approaching platform with regards to the gripped cylinder pose
            if (traj_status_ == TrajStatus::reached_goal) { //approaching platform with respect to cylinder frame
                got_first_object_pose_ = false;

                next_planner_mode_ = PlannerMode::read_new_cylinder;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }

            break;
        case PlannerMode::read_new_cylinder:
            if (got_first_object_pose_) {
                std::lock_guard<std::mutex> lock(mutex_);

                direction_2_place_ = dropping_pos_ - eef_pos_;
                Eigen::Vector3d projection;
                Eigen::Vector3d plane_normal = dropping_plane_normal_; //Lage des Objekts im Raum soll später durch Objekterkennung bestimmt werden
                Eigen::Matrix3d rotation_matrix;

                eef_pos_des_ = dropping_pos_ + (offset_dist_ + 0.5*cylinder_height_) * dropping_plane_normal_.normalized(); //normal vector alwyas in the preferred direction

                vectorPlaneProjection(plane_normal, direction_2_place_, projection);
                getRotationFromVector(projection, plane_normal, rotation_matrix);

                Eigen::Quaterniond quat(rotation_matrix);
                eef_att_des_ = quat;

                //add position offset, here I don't correct rotation error to gripped object
                // offset from obj->eef
                Eigen::Vector3d pos_offset = eef_pos_ - object_pos_;
                // express in eff frame (the original , tilted by 30°, which is the one in the pose_endeffector topic)
                // eef_att_.toRotationMatrix().transpose() * pos_offset;
                // express offset, with respect to the goal eef frame
                // rotation_matrix * (eef_att_.toRotationMatrix().transpose() * pos_offset);
                eef_offset_obj_new_I_ = rotation_matrix * (eef_att_.toRotationMatrix().transpose() * pos_offset);
                eef_pos_des_ = eef_pos_des_ + eef_offset_obj_new_I_;

                

                // Eigen::Vector3d object_z = object_att_ * Eigen::Vector3d(0, 0, 1); 
                // Eigen::Vector3d eef_z = eef_att_ * Eigen::Vector3d(0, 0, 1);
                // object_z.normalize();
                // eef_z.normalize();
                // Eigen::Vector3d rotation_axis = object_z.cross(rotation_x_60_.transpose()*eef_z);
                // double theta = std::acos(object_z.dot(eef_z));

                // Eigen::AngleAxisd angle_axis(theta, rotation_axis);  // Create angle-axis representation
                // Eigen::Matrix3d rotation_matrix_correction = angle_axis.toRotationMatrix();  // Convert to rotation matrix

                // Eigen::Matrix3d final_rotation = rotation_matrix * rotation_x_60_.transpose() * rotation_matrix_correction;

                // Eigen::Quaterniond quat(final_rotation);


                // Eigen::Vector3d eef_offset_obj_I = eef_pos_ - object_pos_;
                // Eigen::Quaterniond eef_att_rot_60(eef_att_.toRotationMatrix() * rotation_x_60_.transpose());
                // Eigen::Matrix3d rot_inverse = eef_att_rot_60.toRotationMatrix().transpose();
                // Eigen::Vector3d eef_offset_obj_eef = rot_inverse * eef_offset_obj_I;
                
                counter_msg_pose_des_++;

                traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::approach_placement_offset_pose;
            }
            break;
        case PlannerMode::approach_placement_offset_pose:
            // traj_mode_ = TrajMode::keep_eef_pose;
            
            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
        
            eef_pose_goal_pub_->publish(pose_out_msg_);

            if (traj_status_ == TrajStatus::reached_goal) {
                std::lock_guard<std::mutex> lock(mutex_);
                // eef_pos_des_ = place_pos_; //ggf. kleiner offset wegen Ungenauigkeiten
                // attitude remains the same

                // ########################################## (this works, without correcting positional grip error)
                // without gripped offset correction
                // eef_pos_des_ = dropping_pos_ + (0.5*cylinder_height_ + discharge_height_) * dropping_plane_normal_.normalized();
                // ##########################################


                // // with gripped offset correction
                eef_pos_des_ = eef_offset_obj_new_I_ + dropping_pos_ + (0.5*cylinder_height_ + discharge_height_) * dropping_plane_normal_.normalized();


                counter_msg_pose_des_++;

                next_planner_mode_ = PlannerMode::approaching_placement_pose;
                next_traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::approaching_placement_pose:
            // for now keep the current attitude, just change the position
            // Problem for later: making sure not to touch the ground with object
            // maybe lift a little before approaching placement pose

            // traj_mode_ = TrajMode::keep_eef_pose;
            
            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
        
            eef_pose_goal_pub_->publish(pose_out_msg_);

            if (traj_status_ == TrajStatus::reached_goal) {
                next_planner_mode_ = PlannerMode::releasing_object;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::releasing_object:
            gripper_mode_ = GripperMode::open;

            if (gripper_status_ == GripperStatus::opened) {
                std::lock_guard<std::mutex> lock(mutex_);

                eef_pos_des_ = eef_pos_des_ - offset_dist_ * projection_.normalized();

                // eef_pos_des_ = eef_offset_obj_new_I_ + dropping_pos_ + (offset_dist_ + cylinder_height_) * dropping_plane_normal_.normalized() - offset_dist_ * direction_2_place_.normalized();
                // attitude remains the same

                counter_msg_pose_des_++;
                traj_mode_ = TrajMode::new_eff_trajectory;
                planner_mode_ = PlannerMode::retreat_from_object;
            }
            break;
        case PlannerMode::retreat_from_object:
            // traj_mode_ = TrajMode::keep_eef_pose;
        
            pose_out_msg_.number = counter_msg_pose_des_;
            pose_out_msg_.header.stamp = this->now();
            hippo_common::convert::EigenToRos(eef_pos_des_, pose_out_msg_.position);
            hippo_common::convert::EigenToRos(eef_att_des_, pose_out_msg_.orientation);
        
            eef_pose_goal_pub_->publish(pose_out_msg_);

            if (traj_status_ == TrajStatus::reached_goal) {
                next_planner_mode_ = PlannerMode::finishing_pick_and_place;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::finishing_pick_and_place:
            stream_counter_.str(std::string());
            stream_rounds_.str(std::string());
            stream_counter_ << counter_rounds_;
            stream_rounds_ << number_rounds_;
            RCLCPP_INFO(get_logger(), "Test %s/%s finished\n", stream_counter_.str().c_str(), stream_rounds_.str().c_str());

            test_run_activated_ = false;
            RCLCPP_INFO(get_logger(), "Deactivating Pick and Place Test Run.");

                
            counter_rounds_++;
            traj_mode_ = TrajMode::go_to_starting_position;
            gripper_mode_ = GripperMode::close;
            if (counter_rounds_ > number_rounds_) {
                RCLCPP_INFO(get_logger(), "Returning to initial position and finishing test run\n");
                next_planner_mode_ = PlannerMode::done;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            } else {
                // reset booleans to allow for new object and platform pose data
                // got_first_platform_pose_ = false;
                // got_first_object_pose_ = false;
                next_planner_mode_ = PlannerMode::waiting_for_all_processes;
                next_traj_mode_ = TrajMode::keep_eef_pose;
                planner_mode_ = PlannerMode::transition_after_reaching_goal;
            }
            break;
        case PlannerMode::done:
            traj_mode_ = TrajMode::keep_eef_pose;
            gripper_mode_ = GripperMode::close;
            break;
        case PlannerMode::transition_after_reaching_goal:
            traj_mode_ = TrajMode::received_status_reached_goal;
            if (traj_status_ == TrajStatus::waiting_for_planner){
              traj_mode_ = next_traj_mode_;
              planner_mode_ = next_planner_mode_;
            } 
            break;
        default:
            RCLCPP_ERROR(get_logger(),
                   "Unknown planner status. No Planner Commands.");
            break;
    }

    msg_gripper_.data = gripper_mode_;
    msg_traj_.data = traj_mode_;
    msg_planner_.data = planner_mode_;
    gripper_mode_pub_->publish(msg_gripper_);
    control_mode_pub_->publish(msg_traj_);
    planner_mode_pub_->publish(msg_planner_);
}
}  // namespace uvms_kin_ctrl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_kin_ctrl::UVMSPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
