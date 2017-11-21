cmake_minimum_required(VERSION 3.5)

project(talker_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_definitions("-std=c++11 -Wall -Wextra")

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker
  "rclcpp"
  "std_msgs")
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

ament_package()
