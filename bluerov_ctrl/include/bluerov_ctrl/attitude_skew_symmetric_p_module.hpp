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

#ifndef BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_HPP
#define BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
namespace bluerov_ctrl {
    void skew(const Eigen::Vector3d &x, Eigen::Matrix3d &x_tilde);

    class AttSkewSymmetricPControlModule {
    public:
        /**
         * @brief Computes the desired angular velocity in the body frame
         *
         * @param _orientation Current orientation as quaternion.
         * @return Eigen::Vector3d Desired angular velocity for cascaded controller in body-frame
         */
        void update(const Eigen::Quaterniond &_orientation, Eigen::Vector3d &out_vel);
        void setOrientationTarget(const Eigen::Quaterniond &_target) {
            orientation_target_ = _target;
        }
        void setOrientationTarget(const double &_roll, const double &_pitch,
                                  const double &_yaw);
        void setAngularVelocityTarget(const Eigen::Vector3d &_target) {
            v_angular_target_ = _target;
        }

        void setAngularVelocityTarget(const double &x, const double &y,
                                      const double &z) {
            v_angular_target_.x() = x;
            v_angular_target_.y() = y;
            v_angular_target_.z() = z;
        }
        void setRollGainP(double _gain) { p_gains_.x() = _gain; }

        void setPitchGainP(double _gain) { p_gains_.y() = _gain; }

        void setYawGainP(double _gain) { p_gains_.z() = _gain; }


        /**
         * @brief set proportional gains for roll, pitch and yaw angle.
         *
         * @param _gains
         */
        void setPgains(const Eigen::Array3d &_gains) {
            p_gains_ = Eigen::Array3d(_gains);
        }
        /**
         * @overload
         */
        void setPgains(const std::vector<double> &_gains) {
            p_gains_ = Eigen::Array3d(_gains.data());
        }
        /**
         * @overload
         */
        void setPgains(const std::array<double, 3> &_gains) {
            p_gains_ = Eigen::Array3d(_gains.data());
        }

        void setAttitudeMask(const Eigen::Vector3d &mask){
            att_mask_.diagonal() = mask;
        }

    private:
        // use array instead of vector to simplify coefficient wise multiplication
        Eigen::Array3d p_gains_{1.0, 1.0, 1.0};
        Eigen::Quaterniond orientation_target_{1.0, 0.0, 0.0, 0.0};
        Eigen::Vector3d v_angular_target_{0.0, 0.0, 0.0};  //!< angular velocity target in global frame
        Eigen::DiagonalMatrix<double, 3> att_mask_{1.0, 1.0, 1.0}; //!< mask used for selectively tracking only velocity for single DOF
    };



}

#endif //BLUEROV_CTRL_ATTITUDE_SKEW_SYMMETRIC_P_MODULE_HPP
