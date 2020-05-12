#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("talker");
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", qos);
	auto publish_count = 0;
	rclcpp::WallRate loop_rate(500ms);

	while (rclcpp::ok())
	{
		auto message = std::make_unique<std_msgs::msg::String>();
		message->data = "Hello, world! " + std::to_string(publish_count++);
		RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message->data.c_str());
		publisher->publish(std::move(message));
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}
