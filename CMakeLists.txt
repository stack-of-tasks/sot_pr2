cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/python.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("jrl-mal")
ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")

# This is required by the Python prologue.
ADD_REQUIRED_DEPENDENCY("sot-dynamic")

# Python
INCLUDE(cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

set(SOURCES
    src/pr2_sot_controller.cpp
    src/pr2_device.cpp)

set(HEADERS
    include/sot_pr2/pr2_sot_controller.h
    include/sot_pr2/pr2_device.h)

rosbuild_add_library(sot_pr2 ${HEADERS} ${SOURCES})

target_link_libraries(sot_pr2 dynamic-graph)
target_link_libraries(sot_pr2 sot-core)
target_link_libraries(sot_pr2 dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "dynamic-graph-python")

ADD_SUBDIRECTORY(src)
