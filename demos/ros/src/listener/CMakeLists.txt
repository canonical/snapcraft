cmake_minimum_required(VERSION 2.8.3)
project(listener)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/listener_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  talk_and_listen.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
