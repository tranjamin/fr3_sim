#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("talker");

    auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World!";

        RCLCPP_INFO(node->get_logger(), "%s", msg.data.c_str());

        chatter_pub->publish(msg);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
