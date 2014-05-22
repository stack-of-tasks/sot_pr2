INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  rosbuild_add_library(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${NAME} PROPERTIES PREFIX "")

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)

  INSTALL(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/pr2/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot-pr2-${PYTHON_LIBRARY_NAME}-wrap
    )
ENDFUNCTION()

# Configure prologue to embed robot model directory.
#CONFIG_FILES(dynamic_graph/sot/pr2/prologue.py)

#COMPILE_PLUGIN(pr2_device pr2_device.cpp Pr2Device)
#PKG_CONFIG_USE_DEPENDENCY(pr2_device "dynamic-graph")
#PKG_CONFIG_USE_DEPENDENCY(pr2_device "sot-core")
#PKG_CONFIG_USE_DEPENDENCY(pr2_device "dynamic-graph-python")


PYTHON_INSTALL("dynamic_graph/sot/pr2" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "pr2_tasks.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "prologue.py" "${PYTHON_SITELIB}")
