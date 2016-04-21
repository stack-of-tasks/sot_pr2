cmake_minimum_required(VERSION 2.6)

include(cmake/base.cmake)
include(cmake/python.cmake)

set(PROJECT_NAME sot_pr2)
set(PROJECT_DESCRIPTION "Insertion of Stack of Tasks into PR2 controller")
SET(PROJECT_URL "")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("jrl-mal")
ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_REQUIRED_DEPENDENCY("jrl-dynamics-urdf >= 2.0.1-14")

# This is required by the Python prologue.
ADD_REQUIRED_DEPENDENCY("sot-dynamic")

# Ros dependencies
ADD_REQUIRED_DEPENDENCY ("pr2_controller_interface")
ADD_REQUIRED_DEPENDENCY("control_toolbox")
ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge")
ADD_REQUIRED_DEPENDENCY("tf")

# Python
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

set(SOURCES
    src/pr2_controller_plugin.cpp
    src/pr2_threaded_sot_controller.cpp
    src/pr2_sot_controller.cpp
    src/pr2_device.cpp)

set(HEADERS
    include/sot_pr2/pr2_controller_plugin.h
    include/sot_pr2/pr2_threaded_sot_controller.h
    include/sot_pr2/pr2_sot_controller.h
    include/sot_pr2/pr2_device.h)

add_library(sot_pr2 ${HEADERS} ${SOURCES})

PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "jrl-dynamics-urdf")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "pr2_controller_interface")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "control_toolbox")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "dynamic_graph_bridge")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2 "tf")

install(TARGETS sot_pr2 DESTINATION ${CMAKE_INSTALL_LIBDIR})

add_executable(sot_pr2_fake_controller ${HEADERS} src/fake_controller.cpp)
PKG_CONFIG_USE_DEPENDENCY(sot_pr2_fake_controller "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2_fake_controller "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2_fake_controller "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(sot_pr2_fake_controller "jrl-dynamics-urdf")
target_link_libraries(sot_pr2_fake_controller sot_pr2)
install(TARGETS sot_pr2_fake_controller DESTINATION ${CMAKE_INSTALL_LIBDIR})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(python)

install(FILES package.xml DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
SETUP_PROJECT_FINALIZE()
