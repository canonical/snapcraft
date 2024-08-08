cmake_minimum_required(VERSION 3.5)
project(hello C)

# Add executable for hello
add_executable(hello hello.c)
install(TARGETS hello RUNTIME DESTINATION bin)

# Add executable for hello2
add_executable(hello2 hello2.c)
install(TARGETS hello2 RUNTIME DESTINATION bin)