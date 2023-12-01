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
project(uvms_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  find_package(rclcpp)
  find_package(geometry_msgs)
  find_package(nav_msgs)
  find_package(hippo_msgs)
  find_package(hippo_common)
  find_package(visualization_msgs)
  find_package(uvms_common)
  find_package(uvms_msgs)

  add_library(visualizations STATIC
          src/visualizations.cpp)

  target_include_directories(visualizations PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include/>)

  ament_target_dependencies(visualizations
          rclcpp
          geometry_msgs
          nav_msgs
          hippo_msgs
          hippo_common
          visualization_msgs
          uvms_msgs)

  add_executable(visualization_node
          nodes/visualization_node.cpp)

  ament_target_dependencies(visualization_node
          rclcpp
          geometry_msgs
          hippo_msgs
          hippo_common
          visualization_msgs
          uvms_common
          )

  target_link_libraries(visualization_node
          visualizations)


  install(DIRECTORY launch rviz
          DESTINATION share/${PROJECT_NAME})

  install(TARGETS visualization_node
          DESTINATION lib/${PROJECT_NAME})

  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
