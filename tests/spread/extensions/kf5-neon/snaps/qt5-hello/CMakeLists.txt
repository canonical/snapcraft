cmake_minimum_required(VERSION 2.6)
project(hello)

set (QT_MIN_VERSION "5.7.0")
find_package(Qt5 ${QT_MIN_VERSION} REQUIRED NO_MODULE COMPONENTS Core)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_executable(hello hello.cpp)
target_include_directories(hello PRIVATE ${Qt5Core_INCLUDE_DIRS})
target_link_libraries(hello PRIVATE Qt5::Core)

install(TARGETS hello RUNTIME DESTINATION bin)

