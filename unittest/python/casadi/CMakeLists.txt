#
# Copyright (c) 2020-2021 INRIA
#

set(${PROJECT_NAME}_PYTHON_CASADI_TESTS bindings_main_algo)

foreach(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
  set(TEST_NAME "${PROJECT_NAME}-test-py-casadi-${test}")
  ADD_PYTHON_UNIT_TEST(${TEST_NAME} "unittest/python/casadi/${test}.py" "bindings/python")
  ADD_WINDOWS_DLL_PATH_TO_TEST(${TEST_NAME})
endforeach()
