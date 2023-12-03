# Copyright (C) 2023  Niklas Trekel

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.8)
project(uvms_sim)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gflags REQUIRED)
find_package(ros_gz_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
find_package(ignition-math6 REQUIRED)
set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-transport11 REQUIRED)
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})

find_package(alpha_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)

add_executable(bridge
        src/static_bridge.cpp)

ament_target_dependencies(bridge
        rclcpp
        ros_gz_bridge
        ignition-transport${GZ_TRANSPORT_VER}
        alpha_msgs
        hippo_msgs)


install(DIRECTORY launch config models
        DESTINATION share/${PROJECT_NAME})


install(TARGETS bridge
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()
