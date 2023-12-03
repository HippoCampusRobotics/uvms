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

#ifndef BLUEROV_CTRL_P_POSITION_CONTROL_HPP
#define BLUEROV_CTRL_P_POSITION_CONTROL_HPP
#include <eigen3/Eigen/Dense>
namespace bluerov_ctrl {
class PosPControlModule {
 public:
  PosPControlModule() = default;

  void setPositionTarget(const double &x, const double &y, const double &z) {
    pos_target.x() = x;
    pos_target.y() = y;
    pos_target.z() = z;
  }

  void setVelocityTarget(const double &x, const double &y, const double &z) {
    vel_target.x() = x;
    vel_target.y() = y;
    vel_target.z() = z;
  }

  void setPositionMask(const Eigen::Vector3d &mask) {
    pos_mask_.diagonal() = mask;
  }

  void setPgainX(const double &gain) { p_gains_.x() = gain; }

  void setPgainY(const double &gain) { p_gains_.y() = gain; }

  void setPgainZ(const double &gain) { p_gains_.z() = gain; }

  //! Returns desired velocity in body-fixed coordinate system
  //! \param pos position estimate in world-COS
  //! \param att attitude estimate world-COS -> body-COS
  //! \param out desired velocity-vector in body frame
  void update(const Eigen::Vector3d &pos, const Eigen::Quaterniond &att,
              Eigen::Vector3d &out_vel);

 private:
  Eigen::Array3d p_gains_{1.0, 1.0, 1.0};
  Eigen::Vector3d pos_target{0.0, 0.0,
                             0.0};  //!< position target in world frame
  Eigen::Vector3d vel_target{0.0, 0.0,
                             0.0};  //!< velocity target in world frame
  Eigen::DiagonalMatrix<double, 3> pos_mask_{
      1.0, 1.0, 1.0};  //!< mask used for selectively tracking only velocity for
                       //!< single DOF
};
}  // namespace bluerov_ctrl
#endif  // BLUEROV_CTRL_P_POSITION_CONTROL_HPP
