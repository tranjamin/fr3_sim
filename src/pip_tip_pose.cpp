#include <rclcpp/rclcpp.hpp>
// MoveIt2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

const double tau = 2 * M_PI;

void get_pip_tip_pose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const rclcpp::Logger& logger)
{
  move_group.setEndEffectorLink("fr3_link_pip_tip");  // adjust to your robot's EE link
  move_group.setPoseReferenceFrame("world");

  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(logger, "Current orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);
  RCLCPP_INFO(logger, "Current position: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
}

void addCollisionObject(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the table
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "fr3_link0";

  // Dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
      shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions = {0.8, 0.8, 0.02};

  // Pose
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.6;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.01;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation =
      collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pip_pose_node");

  // MoveIt2 interfaces need the node pointer
  moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Add collision objects
  addCollisionObject(planning_scene_interface);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Get the current pose
  get_pip_tip_pose(move_group, node->get_logger());

  rclcpp::shutdown();
  return 0;
}
