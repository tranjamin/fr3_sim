cmake_minimum_required(VERSION 3.5)
project(fr3_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)

endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(moveit_ros_perception REQUIRED)

# Service generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveSingleJoint.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# Executables
add_executable(talker_node src/talker.cpp)
ament_target_dependencies(talker_node
  rclcpp
  std_msgs
)

add_executable(test_random_cobot_node src/test_random_cobot.cpp)
ament_target_dependencies(test_random_cobot_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

add_executable(goToPose_node src/goToPose.cpp)
ament_target_dependencies(goToPose_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

add_executable(small_traj_node src/small_traj_test.cpp)
ament_target_dependencies(small_traj_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

add_executable(process_node src/process.cpp)
ament_target_dependencies(process_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(joint_mover_node src/joint_mover.cpp)
ament_target_dependencies(joint_mover_node
  rclcpp
  std_msgs
  rclcpp_action
  moveit_ros_planning_interface
  sensor_msgs
  control_msgs
)

add_executable(move_single_joint_server src/move_joint_service.cpp)
ament_target_dependencies(move_single_joint_server
  rclcpp
  std_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
)

add_executable(pip_pose_node src/pip_tip_pose.cpp)
ament_target_dependencies(pip_pose_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

# Install executables
install(TARGETS
  talker_node
  test_random_cobot_node
  goToPose_node
  small_traj_node
  process_node
  joint_mover_node
  move_single_joint_server
  pip_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
