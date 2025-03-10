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

#include "pap_evaluation_node.hpp"

using std::placeholders::_1;
using namespace std::chrono; // used for time related tasks

namespace uvms_kin_ctrl {


PaPEvaluationNode::PaPEvaluationNode()
    : Node("pap_evaluation_node") {
  initPublishers(); 
  declareParams(); 
  initTimers();
  initSubscriptions();

  msg_SM_change_.data = -1;
}

void PaPEvaluationNode::initPublishers() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "~/eef_position";
    eef_pos_pub_ =
        create_publisher<geometry_msgs::msg::PointStamped>(topic, qos);

    topic = "~/eef_setpoint_position";
    eef_pos_setpoint_pub_ =
        create_publisher<geometry_msgs::msg::PointStamped>(topic, qos);

    topic = "~/position_error_vector";
    eef_pos_vec_error_pub_ =
        create_publisher<geometry_msgs::msg::PointStamped>(topic, qos);

    topic = "~/attitude_error_vector";
    eef_att_vec_error_pub_ =
        create_publisher<geometry_msgs::msg::PointStamped>(topic, qos);

    topic = "~/SM_state";
    StateMachine_change_pub_ =
        create_publisher<hippo_msgs::msg::Int64Stamped>(topic, qos);

    topic = "~/position_error_scalar";
    eef_pos_error_pub_ =
        create_publisher<hippo_msgs::msg::Float64Stamped>(topic, qos);

    topic = "~/attitude_error_scalar";
    eef_att_error_pub_ =
        create_publisher<hippo_msgs::msg::Float64Stamped>(topic, qos);

    topic = "~/placed_pos_and_error";
    placed_pos_pub_ =
        create_publisher<hippo_msgs::msg::VectorError>(topic, qos);

    topic = "~/gripped_pos_and_error";
    gripped_pos_pub_ =
        create_publisher<hippo_msgs::msg::VectorError>(topic, qos);

    topic = "~/gripped_angle_and_error";
    gripped_angle_pub_ =
        create_publisher<hippo_msgs::msg::VectorError>(topic, qos);

    topic = "~/traj_status_stamped";
    traj_status_stamped_pub_ =
        create_publisher<hippo_msgs::msg::Int64Stamped>(topic, qos);
}

void PaPEvaluationNode::declareParams() {

}

void PaPEvaluationNode::initTimers() {
    double freq = 50;

    publish_evaluation_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(int(1000 * 1.0 / freq)),
        std::bind(&PaPEvaluationNode::publishEvaluation, this));   
}

void PaPEvaluationNode::initSubscriptions() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "pose_endeffector";
    eef_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onPoseEndeffector, this, _1));

    topic = "traj_setpoint";
    eef_traj_sub_ = create_subscription<hippo_msgs::msg::ControlTarget>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onSetpointEndeffector, this, _1));

    topic = "/platform/ground_truth/odometry";
    platform_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onPosePlatform, this, _1));

    topic = "/cylinder/ground_truth/odometry";
    cylinder_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onPoseCylinder, this, _1));

    topic = "traj_status";
    traj_staus_sub_ = create_subscription<std_msgs::msg::Int64>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onTrajStatus, this, _1));

    topic = "planner_mode";
    planner_mode_sub_ = create_subscription<std_msgs::msg::Int64>(
        topic, qos, 
        std::bind(&PaPEvaluationNode::onPlannerMode, this, _1));
}

void PaPEvaluationNode::getFrameZAxis(Eigen::Vector3d& zaxis, const Eigen::Quaterniond& frame_att) {
    zaxis.x() = 2*(frame_att.x()*frame_att.z()+frame_att.w()*frame_att.y());
    zaxis.y() = 2*(frame_att.y()*frame_att.z()-frame_att.w()*frame_att.x());
    zaxis.z() = 1-2*(frame_att.x()*frame_att.x()+frame_att.y()*frame_att.y());
}

void PaPEvaluationNode::getFrameYAxis(Eigen::Vector3d& yaxis, const Eigen::Quaterniond& frame_att) {
    yaxis.x() = 2*(frame_att.y()*frame_att.z()+frame_att.w()*frame_att.x());
    yaxis.y() = 1-2*(frame_att.x()*frame_att.x()+frame_att.z()*frame_att.z());
    yaxis.z() = 2*(frame_att.w()*frame_att.z()-frame_att.x()*frame_att.y());
}

void PaPEvaluationNode::onPoseEndeffector(const geometry_msgs::msg::PoseStamped::SharedPtr _msg){
    hippo_common::convert::RosToEigen(_msg->pose.position, eef_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.orientation, eef_att_);

    getFrameZAxis(eef_z_axis_, eef_att_);

    getFrameYAxis(eef_y_axis_, eef_att_);

    hippo_common::convert::EigenToRos(eef_pos_, msg_eef_pos_.point);
}

void PaPEvaluationNode::onSetpointEndeffector(const hippo_msgs::msg::ControlTarget::SharedPtr _msg){
    hippo_common::convert::RosToEigen(_msg->position, eef_setpoint_pos_);
    hippo_common::convert::RosToEigen(_msg->attitude, eef_setpoint_att_);

    hippo_common::convert::EigenToRos(eef_setpoint_pos_, msg_eef_pos_setpoint_.point);
}

void PaPEvaluationNode::onPosePlatform(const nav_msgs::msg::Odometry::SharedPtr _msg){
    hippo_common::convert::RosToEigen(_msg->pose.pose.position, platform_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, platform_att_);

    getFrameZAxis(platform_z_axis_, platform_att_);
}

void PaPEvaluationNode::onPoseCylinder(const nav_msgs::msg::Odometry::SharedPtr _msg){
    hippo_common::convert::RosToEigen(_msg->pose.pose.position, cylinder_pos_);
    hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, cylinder_att_);

    getFrameZAxis(cylinder_z_axis_, cylinder_att_);
}

void PaPEvaluationNode::onTrajStatus(const std_msgs::msg::Int64::SharedPtr _msg){
    // msg_traj_status_.data = _msg->data;
    // received_traj_status_ = true;

    std::lock_guard<std::mutex> lock(traj_queue_mutex);  // Ensure thread safety
    traj_status_queue.push(_msg->data);  // Add new data to the queue
}

void PaPEvaluationNode::onPlannerMode(const std_msgs::msg::Int64::SharedPtr _msg){
    // planner_mode is a constant stream of the current mode
    // publish only the mode together with a timestamp when a mode starts
    // thus, can read out data in trajectories SM phase per phase
    
    if (last_SM_change_.data != _msg->data) {
        std::lock_guard<std::mutex> lock(SM_queue_mutex);  // Ensure thread safety
        last_SM_change_.data = _msg->data;
        // SM_stage_changed_ = true;

        SM_queue.push(_msg->data);  // Add new data to the queue
    }
}

Eigen::Vector3d PaPEvaluationNode::intersectionPlatform(const Eigen::Vector3d& point_on_line, 
                                             const Eigen::Vector3d& direction_line,
                                             const Eigen::Vector3d& point_on_plane, 
                                             const Eigen::Vector3d& normal_plane){

    // Berechne den Skalar, der das Verhältnis von Normalvektor und Richtungsvektor beschreibt
    double denominator = normal_plane.dot(direction_line);

    // Wenn der Skalar null ist, sind die Gerade und die Ebene parallel
    if (std::abs(denominator) < 1e-6) {
        Eigen::Vector3d vec;
        vec << std::nan("1"), std::nan("1"), std::nan("1");
        return vec;
    }

    // Berechne den Parameter t für den Schnittpunkt
    double t = normal_plane.dot(point_on_plane - point_on_line) / denominator;

    // Berechne den Schnittpunkt
    Eigen::Vector3d intersection_point = point_on_line + t * direction_line;

    return intersection_point;
}

Eigen::Matrix4d PaPEvaluationNode::getHomogeneousTransformation() { // T_platform^I
    // Erstelle eine 4x4 Identitätsmatrix
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    // Füge die Rotationsmatrix (3x3) aus dem Quaternion in die homogene Transformationsmatrix ein
    transformation.block<3,3>(0,0) = platform_att_.toRotationMatrix();

    // Füge den Translationsvektor (3x1) in die Matrix ein
    transformation.block<3,1>(0,3) = platform_pos_;

    return transformation;
}

Eigen::Vector3d PaPEvaluationNode::transformPointToNewFrame(const Eigen::Vector3d& point_in_inertial,
                                                            const Eigen::Matrix4d& transformation_matrix) {
    // Erstelle den Punkt als homogenen Vektor (3D-Punkt erweitert um eine 1 für die Translation)
    Eigen::Vector4d point_homogeneous;
    point_homogeneous.head<3>() = point_in_inertial;
    point_homogeneous(3) = 1.0;

    // Berechne die inverse Transformation
    Eigen::Matrix4d inverse_transformation = transformation_matrix.inverse();

    // Wende die inverse Transformation auf den Punkt an
    Eigen::Vector4d point_in_frame_homogeneous = inverse_transformation * point_homogeneous;

    // Extrahiere den 3D-Punkt aus dem homogenen Ergebnis
    Eigen::Vector3d point_in_frame = point_in_frame_homogeneous.head<3>();

    return point_in_frame;
}

void PaPEvaluationNode::skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde) {
  x_tilde << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
}

void PaPEvaluationNode::publishEvaluation() {
    // damit alle daten dieselben Zeitstempel haben
    // daher vielleicht auch den traj status und den planner mode hier publishen unter demselben Zeitstempel
    rclcpp::Time time = this->now();
    msg_eef_pos_.header.stamp = time;
    msg_eef_pos_setpoint_.header.stamp = time;
    msg_eef_pos_vec_err_.header.stamp = time;
    msg_eef_att_vec_err_.header.stamp = time;
    msg_SM_change_.header.stamp = time;
    msg_eef_pos_err_.header.stamp = time;
    msg_eef_att_err_.header.stamp = time;
    msg_placed_pos_.header.stamp = time;
    msg_gripped_pos_.header.stamp = time;
    msg_gripped_angle_.header.stamp = time;
    msg_traj_status_.header.stamp = time;


    // eef position error with sign in x,y,z from setpoint
    // hippo_common::convert::EigenToRos(eef_pos_ - eef_setpoint_pos_, msg_eef_pos_vec_err_.point);
    Eigen::Vector3d err_pos = eef_pos_ - eef_setpoint_pos_;
    msg_eef_pos_vec_err_.point.x = err_pos.x();
    msg_eef_pos_vec_err_.point.y = err_pos.y();
    msg_eef_pos_vec_err_.point.z = err_pos.z();

    // eef attitude error with sign in roll, pitch, yaw from setpoint
    Eigen::Vector3d euler_eef = eef_att_.toRotationMatrix().eulerAngles(0, 1, 2); // 0=roll, 1=pitch, 2=yaw
    Eigen::Vector3d euler_setpoint = eef_setpoint_att_.toRotationMatrix().eulerAngles(0, 1, 2);
    // hippo_common::convert::EigenToRos(euler_eef - euler_setpoint, msg_eef_att_vec_err_.point);
    Eigen::Vector3d err_att = euler_eef - euler_setpoint;
    msg_eef_att_vec_err_.point.x = err_att.x();
    msg_eef_att_vec_err_.point.y = err_att.y();
    msg_eef_att_vec_err_.point.z = err_att.z();

    // eef position error norm at each timestamp
    msg_eef_pos_err_.data = (eef_pos_ - eef_setpoint_pos_).norm();

    // eef attitude error norm at each timestamp
    Eigen::Matrix3d setpoint_att_tilde;
    skew(eef_setpoint_att_.vec(), setpoint_att_tilde);
    Eigen::Vector3d att_error = eef_att_.w() * eef_setpoint_att_.vec() - 
                                eef_setpoint_att_.w() * eef_att_.vec() -
                                setpoint_att_tilde * eef_att_.vec();
    msg_eef_att_err_.data = att_error.norm();

    // cylinder deviation from center of platform
    // take mean over everything later by using SM phases to look at right times
    Eigen::Vector3d intersec = intersectionPlatform(cylinder_pos_, cylinder_z_axis_, platform_pos_, platform_z_axis_);
    if (std::isnan(intersec(0))) {
        msg_placed_pos_.vector.x = std::nan("1");
        msg_placed_pos_.vector.y = std::nan("1");
        msg_placed_pos_.vector.z = std::nan("1");
        msg_placed_pos_.error = std::nan("1");
    } else {
        intersec = transformPointToNewFrame(intersec, getHomogeneousTransformation());
        msg_placed_pos_.vector.x = intersec.x();
        msg_placed_pos_.vector.y = intersec.y();
        msg_placed_pos_.vector.z = intersec.z(); // müsste Null sein, ggf. = 0 setzen, falls erste Tests dies bestätigen
        msg_placed_pos_.error = intersec.norm();
    }

    // gripping point deviation along cylinder z-axis
    // take mean over everything later by using SM phases to look at right times
    //error vector expressed in inertial frame
    Eigen::Vector3d err_vec = cylinder_att_.toRotationMatrix().transpose() * (eef_pos_ - cylinder_pos_); //error vector expressed in cylinder frame
    msg_gripped_pos_.vector.x = err_vec.x();
    msg_gripped_pos_.vector.y = err_vec.y();
    msg_gripped_pos_.vector.z = err_vec.z();
    msg_gripped_pos_.error = err_vec.z();

    // rotation der z-Axe
    // Compute the projection of v onto the plane
    // Angle from eef z-axis "perfect grip" to actual cylinder z-axis
    Eigen::Matrix3d R_eef_perpendicular_top_I = eef_att_.toRotationMatrix() * rotation_x_60_.transpose();
    Eigen::Vector3d y_axis_eef = R_eef_perpendicular_top_I.col(1);
    Eigen::Vector3d z_axis_eef = R_eef_perpendicular_top_I.col(2);
    
    

    Eigen::Vector3d cylinder_z_axis_projected = cylinder_z_axis_ - (cylinder_z_axis_.dot(y_axis_eef)) * y_axis_eef;
    double angle_radians = std::acos((z_axis_eef.dot(cylinder_z_axis_projected)) / (z_axis_eef.norm() * cylinder_z_axis_projected.norm()));
    Eigen::Vector3d cross_product = z_axis_eef.cross(cylinder_z_axis_projected);
    if (cross_product.y() < 0) {
        angle_radians = -angle_radians;
    }
    double angle_degrees = angle_radians * (180.0 / M_PI);
    msg_gripped_angle_.vector.x = 0.0; // gff noch die roll pitch yaw winkel, von eef frame to cylinder frame
    msg_gripped_angle_.vector.y = 0.0;
    msg_gripped_angle_.vector.z = 0.0;
    msg_gripped_angle_.error = angle_degrees;





    eef_pos_pub_->publish(msg_eef_pos_); //cont
    eef_pos_setpoint_pub_->publish(msg_eef_pos_setpoint_); //cont
    eef_pos_vec_error_pub_->publish(msg_eef_pos_vec_err_); //cont
    eef_att_vec_error_pub_->publish(msg_eef_att_vec_err_); //cont
    // if (SM_stage_changed_) {
    //     StateMachine_change_pub_->publish(msg_SM_change_);
    //     SM_stage_changed_ = false;
    // }

    {
        std::lock_guard<std::mutex> lock(SM_queue_mutex);
        if(!SM_queue.empty()) {
            msg_SM_change_.data = SM_queue.front();
            StateMachine_change_pub_->publish(msg_SM_change_);
            SM_queue.pop();  // Remove after publishing
        }
    }

    eef_pos_error_pub_->publish(msg_eef_pos_err_); //cont
    eef_att_error_pub_->publish(msg_eef_att_err_); //cont
    placed_pos_pub_->publish(msg_placed_pos_);
    gripped_pos_pub_->publish(msg_gripped_pos_);
    gripped_angle_pub_->publish(msg_gripped_angle_);
    // if (received_traj_status_) {
    //     traj_status_stamped_pub_->publish(msg_traj_status_);
    //     received_traj_status_ = false;
    // } 

    {
        std::lock_guard<std::mutex> lock(traj_queue_mutex);
        if(!traj_status_queue.empty()) {
            msg_traj_status_.data = traj_status_queue.front();
            traj_status_stamped_pub_->publish(msg_traj_status_);
            traj_status_queue.pop();  // Remove after publishing
        }
    }
}

}  // namespace uvms_kin_ctrl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uvms_kin_ctrl::PaPEvaluationNode>());
  rclcpp::shutdown();
  return 0;
}
