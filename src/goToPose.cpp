#include <memory>
#include <chrono>
#include <thread>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// MoveIt 2
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// TF2 (ROS 2 headers use .hpp)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

static constexpr double tau = 2.0 * M_PI;  // not used below, but kept if you need it

void addCollisionObject(PlanningSceneInterface& planning_scene_interface,
                        const rclcpp::Logger& logger)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects(1);

  // Add the table
  auto& table = collision_objects[0];
  table.id = "table";
  table.header.frame_id = "fr3_link0";

  table.primitives.resize(1);
  table.primitives[0].type = table.primitives[0].BOX;
  table.primitives[0].dimensions = {0.8, 0.8, 0.02};

  table.primitive_poses.resize(1);
  table.primitive_poses[0].position.x = 0.6;
  table.primitive_poses[0].position.y = 0.0;
  table.primitive_poses[0].position.z = 0.01;
  table.primitive_poses[0].orientation.w = 1.0;

  table.operation = table.ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(logger, "Added collision object: table");
}

void plan_move(MoveGroupInterface& move_group, const rclcpp::Logger& logger)
{
  move_group.setEndEffectorLink("fr3_link_pip_tip");   // adjust to your EE link
  move_group.setPoseReferenceFrame("world");

  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(logger, "current orientation x: %f", current_pose.pose.orientation.x);
  RCLCPP_INFO(logger, "current orientation y: %f", current_pose.pose.orientation.y);
  RCLCPP_INFO(logger, "current orientation z: %f", current_pose.pose.orientation.z);
  RCLCPP_INFO(logger, "current orientation w: %f", current_pose.pose.orientation.w);

  geometry_msgs::msg::Pose target_pose1;
  // Use a valid quaternion; w=1, x=y=z=0 is identity
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 1.0;

  target_pose1.position.x = 0.625;
  target_pose1.position.y = 0.10;
  target_pose1.position.z = 0.20;

  if (!move_group.setPoseTarget(target_pose1)) {
    RCLCPP_WARN(logger, "Pose target is invalid or not accepted.");
    return;
  }

  MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Planning (pose goal): %s", success ? "SUCCESS" : "FAILED");
}

void execute_move(MoveGroupInterface& move_group, const rclcpp::Logger& logger)
{
  move_group.setGoalJointTolerance(1e-3);
  move_group.setGoalPositionTolerance(1e-3);
  move_group.setGoalOrientationTolerance(1e-2);

  bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Execution: %s", success ? "SUCCESS" : "FAILED");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("goToPose_node");

  // Spin the node so MoveGroupInterface can communicate (parameters, services, etc.)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Create MoveGroupInterface (ROS 2 requires the node handle)
  MoveGroupInterface move_group(node, "fr3_arm");
  PlanningSceneInterface planning_scene_interface;

  move_group.setPlanningTime(45.0);

  // Let things initialize
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Add collision objects
  addCollisionObject(planning_scene_interface, node->get_logger());
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Plan
  plan_move(move_group, node->get_logger());
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Execute
  execute_move(move_group, node->get_logger());
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Clean shutdown
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}