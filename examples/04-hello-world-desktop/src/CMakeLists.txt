cmake_minimum_required(VERSION 2.8.11)

project(hello-world-desktop)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)

add_executable(hello-world-desktop main.cpp)

target_link_libraries(hello-world-desktop Qt5::Widgets)

install(TARGETS hello-world-desktop RUNTIME DESTINATION bin)
