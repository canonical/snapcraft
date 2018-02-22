cmake_minimum_required(VERSION 3.2)
project(main C)

add_executable(main main.c)
target_link_libraries(main curl)

install(TARGETS main DESTINATION bin)
