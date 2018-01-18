cmake_minimum_required(VERSION 3.2)
project(simple-cmake NONE)

IF(NOT DEFINED DEP_DIR)
    message(FATAL_ERROR "DEP_DIR is not defined.")
ENDIF(NOT DEFINED DEP_DIR)

# This is here to test $SNAPCRAFT_STAGE
IF(NOT EXISTS "${DEP_DIR}/s")
    message(FATAL_ERROR "${DEP_DIR}/s does not exist.")
ENDIF(NOT EXISTS "${DEP_DIR}/s")

file(WRITE my-file "stub file")
install(FILES my-file DESTINATION share)
