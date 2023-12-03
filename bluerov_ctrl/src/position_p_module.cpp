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

#include "bluerov_ctrl/position_p_module.hpp"

void bluerov_ctrl::PosPControlModule::update(const Eigen::Vector3d &pos,
                                             const Eigen::Quaterniond &att,
                                             Eigen::Vector3d &out_vel) {
  out_vel =
      att.toRotationMatrix().inverse() *
      (pos_mask_ * (p_gains_ * (pos_target.array() - pos.array())).matrix() +
       vel_target);
}
