#
# Copyright (c) 2020-2021 INRIA
#

set(${PROJECT_NAME}_PYTHON_CASADI_TESTS
    bindings_main_algo
    bindings_joint_spline
)

foreach(test ${${PROJECT_NAME}_PYTHON_CASADI_TESTS})
    pinocchio_python_add_test(${test})
endforeach()
