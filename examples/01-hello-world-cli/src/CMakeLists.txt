cmake_minimum_required(VERSION 2.6)
project(hello-world-01)

add_executable(hello hello.c)

install(TARGETS hello RUNTIME DESTINATION bin)
