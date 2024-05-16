#
# Copyright (c) 2020-2021 INRIA
#

set(${PROJECT_NAME}_PYTHON_CASADI_TESTS bindings_main_algo)

foreach(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
  set(TEST_NAME "test-py-casadi-${test}")
  add_python_unit_test(${TEST_NAME} "unittest/python/casadi/${test}.py" "bindings/python")
  add_windows_dll_path_to_test(${TEST_NAME})
endforeach(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
