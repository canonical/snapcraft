cmake_minimum_required(VERSION 3.0.2)
project(snapcraft_hello)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package( Boost REQUIRED COMPONENTS
  filesystem
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hello.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
