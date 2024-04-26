#
# Copyright (c) 2020-2021 INRIA
#

SET(${PROJECT_NAME}_PYTHON_CASADI_TESTS
  bindings_main_algo
)

FOREACH(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
  SET(TEST_NAME "test-py-casadi-${test}")
  ADD_PYTHON_UNIT_TEST(${TEST_NAME} "unittest/python/casadi/${test}.py" "bindings/python")
  ADD_WINDOWS_DLL_PATH_TO_TEST(${TEST_NAME})
ENDFOREACH(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
