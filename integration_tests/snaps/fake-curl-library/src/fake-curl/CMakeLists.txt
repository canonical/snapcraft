cmake_minimum_required(VERSION 3.2)
project(fake_curl C)

add_library(curl SHARED fake_curl.c)

install(TARGETS curl DESTINATION lib)
