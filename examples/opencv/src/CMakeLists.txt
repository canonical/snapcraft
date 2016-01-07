cmake_minimum_required(VERSION 2.6)
project(example)

find_package(OpenCV REQUIRED)

add_executable(example main.cpp)

target_link_libraries(example ${OpenCV_LIBS})

install(TARGETS example RUNTIME DESTINATION bin)
