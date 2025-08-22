#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/core/moveit_error_code.hpp>   // <- include this

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("move_group_interface_demo");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");

    move_group.setRandomTarget();

    moveit::core::MoveItErrorCode success = move_group.move();  // returns MoveItErrorCode
    if (success == moveit::core::MoveItErrorCode::SUCCESS) {    // check against core::MoveItErrorCode
        RCLCPP_INFO(node->get_logger(), "Random move succeeded");
    } else {
        RCLCPP_WARN(node->get_logger(), "Random move failed");
    }

    rclcpp::shutdown();
    return 0;
}
