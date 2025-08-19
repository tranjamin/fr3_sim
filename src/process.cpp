#include <rclcpp/rclcpp.hpp>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// For keyboard input
#include <iostream>
#include <array>
#include <chrono>

// tau = 1 rotation in radians
const double tau = 2 * M_PI;

//Position of the elements
const double hole_spacing = 0.05;
const double table_space = 0.1;
//Well plate
const double well_plate_x = table_space + 0.3255;
const double well_plate_y = 0.1315;
const double well_plate_z = 0.07;
//Tip holder
const double tip_holder_x = table_space + 0.025 + 6*hole_spacing + 0.02;
const double tip_holder_y = -0.025 -6*hole_spacing + 0.005;
const double tip_holder_z = 0.09;
//Beaker
const double beaker_x = table_space + 0.025 + 14*hole_spacing;
const double beaker_y = hole_spacing/2;
const double beaker_z = 0.2;
//Tip ejector
const double tip_ejector_x = table_space + 0.025 + 15*hole_spacing - 0.045 - 0.04;
const double tip_ejector_y = -0.025 - 7*hole_spacing + 0.206;
const double tip_ejector_z = 0.155065;

void waitForEnter()
{
    std::cout << "Press Enter to continue to the next motion..." << std::endl;
    std::cin.get();
}

void plan_move(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               std::array<double, 3> target_pos)
{
    move_group.setEndEffectorLink("fr3_link_pip_tip");  
    move_group.setPoseReferenceFrame("world");

    double theta = atan2(target_pos[1], target_pos[0]) - M_PI/2;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    geometry_msgs::msg::Pose target_pose_msg;
    target_pose_msg.orientation = tf2::toMsg(q);
    target_pose_msg.position.x = target_pos[0];
    target_pose_msg.position.y = target_pos[1];
    target_pose_msg.position.z = target_pos[2];

    if (!move_group.setPoseTarget(target_pose_msg)) {
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
    collision_objects.resize(2);

    // Table
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "fr3_link0";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {0.8, 0.8, 0.02};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = table_space + 0.4;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.01;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Tip holder
    collision_objects[1].id = "tip_holder";
    collision_objects[1].header.frame_id = "fr3_link0";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {0.130, 0.08, 0.028};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = tip_holder_x + 0.03;
    collision_objects[1].primitive_poses[0].position.y = tip_holder_y - 0.03;
    collision_objects[1].primitive_poses[0].position.z = 0.02 + 0.028/2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("process_node");

    // Executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Give time for initialization
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveit::planning_interface::MoveGroupInterface group(node, "fr3_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setPlanningTime(45.0);

    addCollisionObject(planning_scene_interface);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Go to pipette tip box
    plan_move(node, group, {tip_holder_x, tip_holder_y, tip_holder_z});
    waitForEnter();
    execute_move(node, group);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Beaker
    plan_move(node, group, {beaker_x, beaker_y, beaker_z});
    waitForEnter();
    execute_move(node, group);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Well plate
    plan_move(node, group, {well_plate_x, well_plate_y, well_plate_z});
    waitForEnter();
    execute_move(node, group);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Tip ejector
    plan_move(node, group, {tip_ejector_x, tip_ejector_y, tip_ejector_z});
    waitForEnter();
    execute_move(node, group);

    rclcpp::shutdown();
    return 0;
}
