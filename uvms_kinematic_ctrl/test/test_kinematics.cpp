#include <yaml-cpp/yaml.h>

#include "uvms_kinematic_ctrl/kinematics.hpp"
using param_utils::BasicTF;
using param_utils::DH;
void readDH(DH &dh, YAML::Node param) {
  dh.d = param["d"].as<double>();
  dh.theta0 = param["theta0"].as<double>();
  dh.a = param["a"].as<double>();
  dh.alp = param["alp"].as<double>();
}
void readBasicTF(BasicTF &tf, YAML::Node param) {
  tf.y = param["y"].as<double>();
  tf.p = param["p"].as<double>();
  tf.r = param["r"].as<double>();
  tf.vec(0) = param["vec"]["x"].as<double>();
  tf.vec(1) = param["vec"]["y"].as<double>();
  tf.vec(2) = param["vec"]["z"].as<double>();
}

bool loadLinkTFParams(bool &active, bool &inertial, param_utils::TFParam &tf,
                      YAML::Node param) {
  inertial = param["inertial"].as<bool>();
  active = param["active"].as<bool>();
  bool dh_link;
  dh_link = param["dh_link"].as<bool>();
  if (dh_link) {
    readDH(tf.dh, param);
  } else {
    readBasicTF(tf.basicTF, param);
  }
  return dh_link;
}

int main(int argc, char **argv) {
  std::string folder_name = "/home/niklast/MA/ros2_ws/src/alpha_model/config/";
  std::string dh_file = folder_name + "alpha_kin_params.yaml";
  std::string base_tf_file = folder_name + "alpha_base_tf_params_bluerov.yaml";
  YAML::Node base_tf_param = YAML::LoadFile(base_tf_file);
  base_tf_param = base_tf_param["/**"];
  base_tf_param = base_tf_param["ros__parameters"];
  std::vector<param_utils::TFParam> tf_params(param_utils::n_links);
  bool dummy_active, dummy_inertial;
  loadLinkTFParams(
      dummy_active, dummy_inertial, tf_params[0],
      base_tf_param[static_cast<std::string>(param_utils::link_names[0])]);

  YAML::Node dh_param = YAML::LoadFile(dh_file);
  dh_param = dh_param["/**"];
  dh_param = dh_param["ros__parameters"];
  for (size_t i = 1; i < param_utils::n_links; i++) {
    loadLinkTFParams(
        dummy_active, dummy_inertial, tf_params[i],
        dh_param[static_cast<std::string>(param_utils::link_names[i])]);
  }
  uvms_kinematics::Kinematics kinematics(tf_params);
  Eigen::Vector3d p_eef;
  Eigen::Quaterniond att_eef;
  param_utils::StateVector q;
  q << 0.1, 0.2, 0.3, 0.4;
  Eigen::Vector3d pos;
  Eigen::Quaterniond att;
  pos << 2.0, 1.0, -0.5;
  att = {1.0, 0.0, 0.0, 0.0};
  kinematics.update(q, pos, att);
  p_eef = kinematics.getEefPosition();
  att_eef = kinematics.getEefAttitude();
  Eigen::Matrix<double, 6, param_utils::n_states_uvms> J_tmp;
  Eigen::Matrix<double, 3, param_utils::n_states_uvms> J;
  Eigen::Matrix<double, 3, param_utils::n_states_uvms> J_rot;
  kinematics.getEefJacobian(J_tmp);
  J = J_tmp.block<3, param_utils::n_states_uvms>(0, 0);
  J_rot = J_tmp.block<3, param_utils::n_states_uvms>(3, 0);
  std::cout << "Endeffector pos: "
            << "x: " << p_eef(0) << " y: " << p_eef(1) << " z: " << p_eef(2)
            << std::endl;

  Eigen::Quaterniond att_eef_des =
      att_eef * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());

  Eigen::Vector<double, 6> error;
  // error.setZero();
  // error(3) =1.0;
  error.segment<3>(0).setZero();
  uvms_kinematics::quaternionError(att_eef_des, att_eef, error.segment<3>(3));
  Eigen::DiagonalMatrix<double, param_utils::n_states_uvms> W;
  W.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.05, 0.05, 0.05, 0.05;
  Eigen::Matrix<double, param_utils::n_states_uvms, 6> inverse =
      J_tmp.transpose() * (J_tmp * J_tmp.transpose()).inverse();
  Eigen::Matrix<double, param_utils::n_states_uvms, 6> inverse_weighted =
      W.inverse() * J_tmp.transpose() *
      (J_tmp * W.inverse() * J_tmp.transpose()).inverse();
  Eigen::Vector<double, param_utils::n_states_uvms> vel_des = inverse * error;
  Eigen::Vector<double, param_utils::n_states_uvms> vel_des_weighted =
      inverse_weighted * error;
  Eigen::Vector<double, 6> check_eef = J_tmp * vel_des;
  Eigen::Vector<double, 6> check_eef_weighted = J_tmp * vel_des_weighted;

  Eigen::Quaterniond test_quaternion =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
  Eigen::Vector3d test_rpy =
      test_quaternion.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
  std::cout << "test_rpy" << test_rpy << std::endl;

  Eigen::Vector3d test_error;
  uvms_kinematics::quaternionError(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())),
      Eigen::Quaterniond::Identity(), test_error);
  Eigen::Vector3d p_eef_new;
  Eigen::Quaterniond att_eef_new;
  param_utils::UVMSStateVector d_states;
  Eigen::Vector3d pos_new;
  Eigen::Quaterniond att_new;
  param_utils::StateVector q_new;
  double eps = 1e-5;
  for (int i = 0; i < int(param_utils::n_states_uvms); i++) {
    pos_new = pos;
    att_new = att;
    q_new = q;
    if (i < 3) {
      pos_new(i) += eps;
    } else if (i < 6) {
      Eigen::Vector3d unit_axis;
      Eigen::AngleAxisd epsAtt;
      unit_axis.setZero();
      unit_axis(i - 3) = 1.0;
      epsAtt = Eigen::AngleAxisd(eps, unit_axis);
      att_new = att_new * epsAtt;
    } else {
      q_new(i - 6) += eps;
    }

    // calculate vector containing variation
    d_states.segment<3>(0) = pos_new - pos;
    Eigen::Quaterniond datt = att_new * att.inverse();
    auto datt_axis = Eigen::AngleAxisd(datt);
    Eigen::Vector3d drot = datt_axis.angle() * datt_axis.axis();
    d_states.segment<3>(3) = drot;
    d_states.segment<param_utils::n_active_joints>(6) = q_new - q;

    // update states
    kinematics.update(q_new, pos_new, att_new);
    p_eef_new = kinematics.getEefPosition();
    att_eef_new = kinematics.getEefAttitude();

    // calculate difference
    Eigen::Vector3d dp_eef = J * d_states;
    Eigen::Vector3d dp_eef2 = p_eef_new - p_eef;
    std::cout << "delta position" << (dp_eef - dp_eef2).norm() / eps
              << std::endl;
    Eigen::Quaterniond datt_eef = att_eef_new * att_eef.inverse();
    Eigen::Vector3d drot_eef = J_rot * (d_states);
    Eigen::AngleAxisd datt_eef_axis = Eigen::AngleAxisd(datt_eef);
    Eigen::Vector3d drot_eef2 = datt_eef_axis.angle() * datt_eef_axis.axis();
    std::cout << "delta rotation" << (drot_eef - drot_eef2).norm() / eps
              << std::endl;
  }
  return 0;
}