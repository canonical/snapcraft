#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("talker");

  auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", rmw_qos_profile_default);

  rclcpp::WallRate loopRate(1);

  int count = 0;
  while (rclcpp::ok())
  {
    auto message = std::make_shared<std_msgs::msg::String>();

    std::stringstream stream;
    stream << "Hello world " << count++;
    message->data = stream.str();

    std::cout << message->data << std::endl;

    publisher->publish(message);

    rclcpp::spin_some(node);

    loopRate.sleep();
  }

  return 0;
}
