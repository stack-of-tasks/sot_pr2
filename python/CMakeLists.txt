INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES pr2_catch.py pr2_sot_demo.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

