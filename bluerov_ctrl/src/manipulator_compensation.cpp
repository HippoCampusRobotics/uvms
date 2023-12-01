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

#include "bluerov_ctrl/manipulator_compensation.hpp"

namespace bluerov_ctrl {
    ManipulatorCompensation::ManipulatorCompensation() {
        counter_force_.setZero();
        counter_torque_.setZero();
    }

    void ManipulatorCompensation::update(const Eigen::Quaterniond &orientation, Eigen::Vector3d &force,
                                                       Eigen::Vector3d &torque) {

        // calculate forces to counter buoyancy
        force = - orientation.toRotationMatrix().inverse() * added_buoyancy_mass_ * Eigen::Vector3d::UnitZ() * (param_utils::GRAVITY);
        torque = - buoyancy_origin_.cross(orientation.toRotationMatrix().inverse() * added_buoyancy_mass_ * Eigen::Vector3d::UnitZ() * (param_utils::GRAVITY));
        force += counter_force_;
        torque += counter_torque_;
    }

    void ManipulatorCompensation::setCompensationForce(const double &x, const double &y, const double &z) {
        counter_force_ = -Eigen::Vector3d(x, y, z);
    }

    void ManipulatorCompensation::setCompensationTorque(const double &x, const double &y, const double &z) {
        counter_torque_ = -Eigen::Vector3d(x, y, z);
    }

}