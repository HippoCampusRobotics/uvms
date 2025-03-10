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
project(uvms_kinematic_ctrl)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_control_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(alpha_model REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uvms_common REQUIRED)
find_package(uvms_msgs REQUIRED)
find_package(alpha_ctrl REQUIRED)
find_package(bluerov_ctrl REQUIRED)
find_package(uvms_trajectory_gen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(kin_ctrl
        src/kinematics.cpp
        src/kin_ctrl.cpp)

target_include_directories(kin_ctrl PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(kin_ctrl
        alpha_model
        uvms_common)

add_library(kin_ctrl_interface
        src/kin_ctrl_node_interface.cpp)

target_include_directories(kin_ctrl_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(kin_ctrl_interface
        rclcpp
        alpha_msgs
        hippo_msgs
        hippo_control_msgs
        hippo_common
        std_msgs
        sensor_msgs
        nav_msgs
        uvms_common
        uvms_msgs
        )
target_link_libraries(kin_ctrl_interface
        kin_ctrl)


add_library(uvms_kin_ctrl_configuration_space
        nodes/uvms_kin_ctrl_node_configuation_space.cpp
        )

ament_target_dependencies(uvms_kin_ctrl_configuration_space
        rclcpp
        uvms_msgs
        alpha_ctrl
        bluerov_ctrl
        hippo_common
        )

add_library(uvms_switching_kin_ctrl_configuration_space
        nodes/uvms_switching_kin_ctrl_node_configuration_space.cpp
        )

ament_target_dependencies(uvms_switching_kin_ctrl_configuration_space
        rclcpp
        uvms_msgs
        alpha_ctrl
        bluerov_ctrl
        hippo_common
        )

add_executable(uvms_kin_ctrl_node
        nodes/uvms_kin_ctrl_node.cpp)

ament_target_dependencies(uvms_kin_ctrl_node
        )
target_link_libraries(uvms_kin_ctrl_node
        kin_ctrl_interface
        uvms_kin_ctrl_configuration_space)


add_executable(uvms_switching_kin_ctrl_node
        nodes/uvms_switching_kin_ctrl_node.cpp)

ament_target_dependencies(uvms_switching_kin_ctrl_node
        )
target_link_libraries(uvms_switching_kin_ctrl_node
        kin_ctrl_interface
        uvms_switching_kin_ctrl_configuration_space)

add_executable(uvms_planner_node
        nodes/uvms_planner_node.cpp)

ament_target_dependencies(uvms_planner_node
        rclcpp
        std_msgs
        Eigen3
        geometry_msgs
        alpha_ctrl
        uvms_trajectory_gen
        hippo_msgs
        uvms_common
        hippo_common
        nav_msgs
        std_srvs
        )

add_executable(pap_evaluation_node
        nodes/pap_evaluation_node.cpp)

ament_target_dependencies(pap_evaluation_node
        rclcpp
        Eigen3
        geometry_msgs
        hippo_msgs
        nav_msgs
        std_msgs
        hippo_common)

# ########
# target_include_directories(uvms_planner_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${uvms_trajectory_gen_INCLUDE_DIRS}
# )
# ###############


add_executable(estimation_drift_watchdog_node
        nodes/estimation_drift_watchdog_node.cpp)

ament_target_dependencies(estimation_drift_watchdog_node
        rclcpp
        nav_msgs
        hippo_msgs
        hippo_control_msgs
        hippo_common)

install(
        DIRECTORY config launch rviz
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS uvms_kin_ctrl_node uvms_switching_kin_ctrl_node estimation_drift_watchdog_node uvms_planner_node pap_evaluation_node
        DESTINATION lib/${PROJECT_NAME})

# for testing:
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

add_executable(test_kinematics test/test_kinematics.cpp)

target_link_libraries(test_kinematics PRIVATE
        ${YAMLCPP_LIBRARIES}
        kin_ctrl)

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

ament_package()
