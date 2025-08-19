#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("move_group_interface_demo");

    // Start a ROS spinning thread
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Create MoveGroupInterface for the "fr3_arm" group
    moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");

    // Set a random target
    move_group.setRandomTarget();

    // Plan and execute the motion
    moveit::planning_interface::MoveItErrorCode success = move_group.move();
    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Random move succeeded");
    } else {
        RCLCPP_WARN(node->get_logger(), "Random move failed");
    }

    rclcpp::shutdown();
    return 0;
}
