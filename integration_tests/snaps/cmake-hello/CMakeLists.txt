cmake_minimum_required(VERSION 3.2)
project(cmake-hello C)
add_executable(cmake-hello test.c)
install(TARGETS cmake-hello RUNTIME DESTINATION bin)
