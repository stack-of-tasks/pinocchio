#
# Copyright (c) 2024 INRIA
#

function(ADD_PINOCCHIO_CPP_CASADI_EXAMPLE EXAMPLE)
  add_pinocchio_cpp_example(${EXAMPLE} CASADI)
endfunction()

if(BUILD_WITH_CASADI_SUPPORT)
  add_pinocchio_cpp_casadi_example(casadi-aba)
  add_pinocchio_cpp_casadi_example(casadi-rnea)

  if(BUILD_PYTHON_INTERFACE)
    set(${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES quadrotor-ocp)
    if(BUILD_WITH_COLLISION_SUPPORT)
      list(APPEND ${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES cartpole)
    endif()

    foreach(EXAMPLE ${${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES})
      set(EXAMPLE_NAME "${PROJECT_NAME}-example-py-casadi-${EXAMPLE}")
      ADD_PYTHON_UNIT_TEST(${EXAMPLE_NAME} "examples/casadi/${EXAMPLE}.py" "bindings/python")
      ADD_WINDOWS_DLL_PATH_TO_TEST(${EXAMPLE_NAME})
    endforeach()
  endif()
endif()
