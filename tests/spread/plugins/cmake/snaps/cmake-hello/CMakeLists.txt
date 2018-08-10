cmake_minimum_required(VERSION 2.6)
project(cmake-hello C)
add_executable(cmake-hello test.c)
install(TARGETS cmake-hello RUNTIME DESTINATION bin)
