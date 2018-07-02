cmake_minimum_required (VERSION 2.6)
project (usefoo)
add_library(foo foo.h foo.cpp)
add_executable(usefoo main.cpp)
target_link_libraries(usefoo foo)
install(TARGETS usefoo
        RUNTIME DESTINATION bin)
