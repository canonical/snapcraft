cmake_minimum_required(VERSION 3.5)
project(hello C)
add_executable(hello hello.c)
install(TARGETS hello RUNTIME DESTINATION bin)
