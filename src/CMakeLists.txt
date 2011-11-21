# Configure prologue to embed robot model directory.
CONFIG_FILES(dynamic_graph/sot/pr2/prologue.py)

EXEC_PROGRAM(
  ${PYTHON_EXECUTABLE}
  ARGS
  "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )
PYTHON_INSTALL("dynamic_graph/sot/pr2" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "pr2.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2" "prologue.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/sot/pr2"
  "${CMAKE_CURRENT_BINARY_DIR}/robot.py" "${PYTHON_SITELIB}")
