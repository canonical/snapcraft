cmake_minimum_required(VERSION 3.2)
project(simple-cmake-replace C)
add_executable(simple-cmake-replace test.c)
install(TARGETS simple-cmake-replace RUNTIME DESTINATION bin)
