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
project(uvms_trajectory_gen)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uvms_common REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_control_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(uvms_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(uvms_traj_gen STATIC
        src/traj.cpp
        )

target_include_directories(uvms_traj_gen PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(uvms_traj_gen
        uvms_common
        rclcpp
        hippo_msgs
        hippo_control_msgs
        hippo_common
        uvms_msgs
        Eigen3
        )

add_library(uvms_startup STATIC
        nodes/uvms_traj_gen_node_initial_startup.cpp)

target_include_directories(uvms_startup PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(uvms_startup
        hippo_common
        uvms_common
        uvms_msgs
        rclcpp
        nav_msgs
        sensor_msgs)

add_executable(uvms_traj_gen_node
        nodes/uvms_traj_gen_node.cpp)

ament_target_dependencies(uvms_traj_gen_node
        rclcpp
        hippo_msgs
        hippo_control_msgs
        hippo_common
        nav_msgs
        sensor_msgs
        Eigen3
        )

target_link_libraries(uvms_traj_gen_node
        uvms_traj_gen
        uvms_startup)

add_executable(uvms_traj_gen_pick_place_node
        nodes/uvms_traj_gen_pick_place_node.cpp)  
        
ament_target_dependencies(uvms_traj_gen_pick_place_node
        rclcpp
        hippo_msgs
        hippo_common
        nav_msgs
        sensor_msgs
        Eigen3
        )

target_link_libraries(uvms_traj_gen_pick_place_node
        uvms_traj_gen
        uvms_startup)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

install(TARGETS uvms_traj_gen_node uvms_traj_gen_pick_place_node
        DESTINATION lib/${PROJECT_NAME})

##############


# install(
#         DIRECTORY include/
#         DESTINATION include/${PROJECT_NAME}
#         )

ament_export_targets(uvms_traj_genTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(uvms_common
        rclcpp
        hippo_msgs
        hippo_common
        uvms_msgs)

install(
        DIRECTORY include/
        DESTINATION include
        )

install(
        TARGETS uvms_traj_gen
        EXPORT uvms_traj_genTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


#############


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
