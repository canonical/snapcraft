cmake_minimum_required(VERSION 2.8.3)
project(timezone_test)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

install(PROGRAMS
  scripts/timezone_test_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
