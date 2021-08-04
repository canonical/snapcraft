#include "test_minimal_publisher/subscriber.h"

namespace test {

MinimalSubscriber::MinimalSubscriber()
: Node("test_minimal_subscriber") {
  this->declare_parameter<bool>("exit_after_receive", exit_after_receive_);
  this->get_parameter("exit_after_receive", exit_after_receive_);
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/test/topic",
    10,
    [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      if (exit_after_receive_) {
        rclcpp::shutdown();
      }
    });
}

} // namespace test
