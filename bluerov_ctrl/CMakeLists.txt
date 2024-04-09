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
project(bluerov_ctrl)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_control_msgs REQUIRED)
find_package(uvms_common REQUIRED)


# velocity controller
add_library(velocity_control_lib STATIC
        src/twist_pid.cpp
        src/manipulator_compensation.cpp
        src/twist_model_based.cpp)

ament_target_dependencies(velocity_control_lib
        uvms_common
        )

target_include_directories(velocity_control_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>
        )

add_library(velocity_control_interfaces_lib STATIC
        src/velocity_control_node_interfaces.cpp
        src/twist_pid_interface.cpp
        src/manipulator_compensation_interface.cpp
        src/twist_model_based_interface.cpp)

target_include_directories(velocity_control_interfaces_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>
        )

ament_target_dependencies(velocity_control_interfaces_lib PUBLIC
        rclcpp
        nav_msgs
        hippo_common
        hippo_msgs
        hippo_control_msgs)

target_link_libraries(velocity_control_interfaces_lib PUBLIC
        velocity_control_lib)

add_executable(velocity_control_node
        nodes/velocity_control_node.cpp)

target_include_directories(velocity_control_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(velocity_control_node PUBLIC
        rclcpp
        nav_msgs
        hippo_msgs
        hippo_control_msgs
        hippo_common
        uvms_common
        )

target_link_libraries(velocity_control_node PRIVATE
        velocity_control_interfaces_lib)

# pose control module

add_library(pose_control_module_lib STATIC
        src/position_p_module.cpp
        src/attitude_skew_symmetric_p_module.cpp)

ament_target_dependencies(pose_control_module_lib
        uvms_common
        )

target_include_directories(pose_control_module_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>
        )

add_library(pose_control_module_interfaces_lib STATIC
        src/position_p_module_interface.cpp
        src/attitude_skew_symmetric_p_module_interface.cpp)

target_include_directories(pose_control_module_interfaces_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>
        )

ament_target_dependencies(pose_control_module_interfaces_lib PUBLIC
        rclcpp
        nav_msgs
        hippo_common
        hippo_control_msgs
        hippo_msgs)

target_link_libraries(pose_control_module_interfaces_lib PUBLIC
        pose_control_module_lib)

add_executable(pose_control_module_node
        nodes/pose_control_module_node.cpp)

target_include_directories(pose_control_module_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(pose_control_module_node PUBLIC
        rclcpp
        nav_msgs
        hippo_msgs
        hippo_control_msgs
        hippo_common
        uvms_common
        )

target_link_libraries(pose_control_module_node PRIVATE
        pose_control_module_interfaces_lib)


add_executable(estimation_drift_watchdog_node
        nodes/estimation_drift_watchdog_node.cpp)

ament_target_dependencies(estimation_drift_watchdog_node
        rclcpp
        nav_msgs
        hippo_msgs
        hippo_control_msgs
        hippo_common)

install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME})

install(TARGETS velocity_control_node pose_control_module_node estimation_drift_watchdog_node
        DESTINATION lib/${PROJECT_NAME})


#export kinematic controller as library
install(DIRECTORY include/
        DESTINATION include)

install(
        TARGETS pose_control_module_interfaces_lib pose_control_module_lib
        EXPORT export_${PROJECT_NAME}}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME}} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        nav_msgs
        hippo_common
        hippo_msgs
        hippo_control_msgs
        uvms_common)


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
