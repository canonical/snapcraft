cmake_minimum_required(VERSION 3.2)
project(simple-cmake-replace C)
configure_file(
    "${PROJECT_SOURCE_DIR}/config.h.in"
    "${PROJECT_SOURCE_DIR}/config.h"
    )
include_directories("${PROJECT_SOURCE_DIR}")
add_executable(simple-cmake-replace test.c)
install(TARGETS simple-cmake-replace RUNTIME DESTINATION bin)
