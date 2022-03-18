cmake_minimum_required(VERSION 3.5)
project(this_package_should_not_be_compiled)

find_package(ament_cmake REQUIRED)
find_package(this_dependency_does_not_exist REQUIRED)

add_executable(NonCompiledTarget this-file-does-not-exist.cpp)

install(TARGETS
  NonCompiledTarget
  DESTINATION lib/${PROJECT_NAME})

ament_package()
