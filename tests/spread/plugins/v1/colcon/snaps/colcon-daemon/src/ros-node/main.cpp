#include <iostream>
#include "rclcpp/rclcpp.hpp"


class MinimalNode : public rclcpp::Node
{
  public:
    MinimalNode() : Node("colcon_daemon") {
		std::cout << "Hello" << std::endl;
	}

	~MinimalNode(){
		std::cout << "Good bye!" << std::endl;
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalNode>());

  rclcpp::shutdown();
  return 0;
}
