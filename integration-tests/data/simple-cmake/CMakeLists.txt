project(simple-cmake C)
add_executable(simple-cmake test.c)
install(TARGETS simple-cmake RUNTIME DESTINATION bin)
