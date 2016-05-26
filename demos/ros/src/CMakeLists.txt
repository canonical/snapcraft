# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8.3)

# optionally provide a cmake file in the workspace to override arbitrary stuff
include(workspace.cmake OPTIONAL)

set(CATKIN_TOPLEVEL TRUE)

# include catkin directly or via find_package()
if(EXISTS "${CMAKE_SOURCE_DIR}/catkin/cmake/all.cmake" AND EXISTS "${CMAKE_SOURCE_DIR}/catkin/CMakeLists.txt")
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/catkin/cmake")
  # include all.cmake without add_subdirectory to let it operate in same scope
  include(catkin/cmake/all.cmake NO_POLICY_SCOPE)
  add_subdirectory(catkin)

else()
  # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
    endif()
  endif()

  # list of catkin workspaces
  set(catkin_search_path "")
  foreach(path ${CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.CATKIN_WORKSPACE")
      list(FIND catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        list(APPEND catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # search for catkin in all workspaces
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin REQUIRED
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)
endif()

catkin_workspace()
