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

#include "uvms_kinematic_ctrl/kinematics.hpp"
    void uvms_kinematics::skew(const Eigen::Ref<const Eigen::Vector3d> &x, Eigen::Ref<Eigen::Matrix3d> x_tilde) {
        x_tilde << 0, -x(2), x(1),
                x(2), 0, -x(0),
                -x(1), x(0), 0;
    }

    void uvms_kinematics::quaternionError(const Eigen::Quaterniond &des, const Eigen::Quaterniond &state, Eigen::Ref<Eigen::Vector3d> out){
        Eigen::Matrix3d v_des_tilde;
        skew(des.vec(), v_des_tilde);
        double scalar_error = des.w() * state.w() + des.vec().transpose() * state.vec();
        out = state.w() * des.vec() - des.w() * state.vec() - v_des_tilde * state.vec();
        out *= int(scalar_error >= 0) * 1 + int(scalar_error < 0)*-1;  // flip sign if scalar error negative, quaternion convention requires error quaternion to be have positive scalar
    }


    void uvms_kinematics::getTransformationEulerBodyRates(const double &r, const double &p, const double &y, Eigen::Ref<Eigen::Matrix3d> J){
        J << 1, 0, -sin(p),
                0, cos(r), cos(p) * sin(r),
                0, -sin(r), cos(p) * cos(r);
    }

    void uvms_kinematics::getTransformationBodyRatesEuler(const double &r, const double &p, const double &y, Eigen::Ref<Eigen::Matrix3d> J) {
        J << 1, sin(r) * tan(p), cos(r) * tan(p),
            0, cos(r), -sin(r),
            0, sin(r) / cos(p), cos(r) / cos(p);
    }


namespace uvms_kinematics {

    Kinematics::Kinematics(const std::vector<param_utils::TFParam> &tf_param):
            auv_pos_(states_.segment<3>(0)),
            auv_att_(states_.segment<3>(3)),
            q_(states_.segment<n_active_joints>(6)),
            r_B_0_(tf_param[0].basicTF.vec),
            R_B_0_(rot_utils::rpy_to_matrix(tf_param[0].basicTF.r, tf_param[0].basicTF.p, tf_param[0].basicTF.y)){
        states_(2) = 1.0;
        manipulator_kin_ = new alpha_kinematics::Kinematics(tf_param);
    }


    Kinematics::Kinematics(Eigen::Matrix<double, n_active_joints + 1, 4> &DH_table, Eigen::Vector3d r_b_0,
                           Eigen::Matrix3d &R_B_0):
            auv_pos_(states_.segment<3>(0)),
            auv_att_(states_.segment<3>(3)),
            q_(states_.segment<n_active_joints>(6)),
            r_B_0_(r_b_0), R_B_0_(R_B_0) {
        manipulator_kin_ = new alpha_kinematics::Kinematics(DH_table);
    }

    void Kinematics::update(const StateVector &q, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att) {
        states_.segment<3>(0) = pos;
        rot_utils::quat_to_rpy(att, states_.segment<3>(3));
        //states_.segment<3>(3) = att.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
        states_.segment<n_active_joints>(6) = q;
        manipulator_kin_->update(q);
        manipulator_kin_->getEefPosition(r_0_eef_);
        manipulator_kin_->getEefAttitude(att_0_eef_);
        manipulator_kin_->getFullJacobian(J_manipulator_pos_, J_manipulator_rot_);
        R_I_B_ = att.toRotationMatrix();
        p_eef_ = auv_pos_ + R_I_B_ * r_B_0_ + R_I_B_ * R_B_0_ * r_0_eef_;
        att_eef_ = R_I_B_ * R_B_0_ * att_0_eef_;
    }

    void Kinematics::getJacobian(const StateVector &q, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, param_utils::UVMSStateMatrix &J){
        J.setZero();
        J.block<3,3>(0,0) = att.toRotationMatrix();
        getTransformationBodyRatesEuler(auv_att_(0), auv_att_(1), auv_att_(2), J.block<3,3>(3,3));
        J.block<n_active_joints, n_active_joints>(6,6).setIdentity();
    }

    void Kinematics::getEefPosition(double &x, double &y, double &z) {
        x = p_eef_.x();
        y = p_eef_.y();
        z = p_eef_.z();

    }

    void Kinematics::getEefAttitude(double &w, double &x, double &y, double &z) {
        w = att_eef_.w();
        x = att_eef_.x();
        y = att_eef_.y();
        z = att_eef_.z();
    }

    const Eigen::Quaterniond& Kinematics::getEefAttitude() {
        return att_eef_;
    }

    void Kinematics::getEef(Eigen::Vector3d &pos_eef, Eigen::Quaterniond &att_eef) {
        getEefPosition(pos_eef.x(), pos_eef.y(), pos_eef.z());
        getEefAttitude(att_eef.w(), att_eef.x(), att_eef.y(), att_eef.z());
    }

    const Eigen::Ref<const Eigen::Vector3d> Kinematics::getLinkBaseRefPosition(const int &idx) {
        return manipulator_kin_->getLinkPosition(idx);
    }

    void Kinematics::getLinkBaseRefPositionJacobian(const int &idx, Eigen::Matrix<double, 3, n_states_uvms> &J){
        manipulator_kin_->getLinkPositionJacobian(idx, J.block<3, n_active_joints>(0, 6));
    }


    void Kinematics::getEefJacobian(Eigen::Matrix<double, 6, n_states_uvms> &J) {
        // translational part:
        J.block<3,3>(0, 0) = R_I_B_;
        Eigen::Vector3d vec_tmp = R_I_B_ * r_B_0_ + R_I_B_ * R_B_0_ * r_0_eef_;
        Eigen::Matrix3d skew_tmp;
        skew(vec_tmp, skew_tmp);
        J.block<3,3>(0, 3) = - skew_tmp * R_I_B_;
        J.block<3,n_active_joints>(0, 6) = R_I_B_ * R_B_0_ * J_manipulator_pos_;

        // rotational part:
        J.block<3,3>(3, 0).setZero();
        J.block<3,3>(3,3) = R_I_B_;
        J.block<3,n_active_joints>(3,6) = R_I_B_ * R_B_0_ * J_manipulator_rot_;
    }


}
