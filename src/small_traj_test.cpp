#include <rclcpp/rclcpp.hpp>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
// tau = 1 rotation in radians
const double tau = 2 * M_PI;

void plan_move(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               std::array<double, 3> target_pos)
{
    move_group.setEndEffectorLink("fr3_link_pip_tip");
    move_group.setPoseReferenceFrame("world");

    auto current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "current orientation x: %f", current_pose.pose.orientation.x);
    RCLCPP_INFO(node->get_logger(), "current orientation y: %f", current_pose.pose.orientation.y);
    RCLCPP_INFO(node->get_logger(), "current orientation z: %f", current_pose.pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "current orientation w: %f", current_pose.pose.orientation.w);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    target_pose.position.x = target_pos[0];
    target_pose.position.y = target_pos[1];
    target_pose.position.z = target_pos[2];

    if (!move_group.setPoseTarget(target_pose)) {
        RCLCPP_WARN(node->get_logger(), "Pose target is invalid or not accepted.");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing plan (pose goal) %s", success ? "SUCCESS" : "FAILED");
}

void execute_move(const rclcpp::Node::SharedPtr& node,
                  moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.setGoalJointTolerance(0.001);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Execution %s", success ? "SUCCESS" : "FAILED");
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Table
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "fr3_link0";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {0.8, 0.8, 0.02};

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.6;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.01;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("small_traj_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::sleep_for(std::chrono::seconds(1));

    moveit::planning_interface::MoveGroupInterface group(node, "fr3_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    group.setPlanningTime(45.0);

    addCollisionObject(planning_scene_interface);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // First target
    plan_move(node, group, {0.625, 0.025, 0.2});
    rclcpp::sleep_for(std::chrono::seconds(1));
    execute_move(node, group);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Second target
    plan_move(node, group, {0.625, 0.025, 0.05});
    rclcpp::sleep_for(std::chrono::seconds(1));
    execute_move(node, group);

    rclcpp::shutdown();
    return 0;
}
