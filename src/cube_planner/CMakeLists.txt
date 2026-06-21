cmake_minimum_required(VERSION 3.8)
project(cube_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(planner_node src/planner.cpp)

ament_target_dependencies(planner_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  control_msgs
  franka_msgs
)

add_executable(planner_bt src/planner_bt.cpp)
ament_target_dependencies(planner_bt
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  control_msgs
  franka_msgs
  behaviortree_cpp
)

install(TARGETS planner_node planner_bt
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
