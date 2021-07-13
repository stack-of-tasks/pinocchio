#
# Copyright (c) 2020-2021 INRIA
#

SET(${PROJECT_NAME}_PYTHON_CASADI_TESTS
  bindings_main_algo
)

FOREACH(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
  ADD_PYTHON_UNIT_TEST("test-py-casadi-${test}" "unittest/python/casadi/${test}.py" "bindings/python")
ENDFOREACH(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
