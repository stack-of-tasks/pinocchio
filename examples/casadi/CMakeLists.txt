#
# Copyright (c) 2025 INRIA
#

function(ADD_PINOCCHIO_CPP_CASADI_EXAMPLE EXAMPLE)
  add_pinocchio_cpp_example(${EXAMPLE} CASADI)
endfunction()

if(BUILD_WITH_CASADI_SUPPORT)

  add_pinocchio_cpp_casadi_example(casadi-aba)
  add_pinocchio_cpp_casadi_example(casadi-crba)
  add_pinocchio_cpp_casadi_example(casadi-rnea)

  if(BUILD_PYTHON_INTERFACE)

    set(${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES quadrotor-ocp)
    if(BUILD_WITH_COLLISION_SUPPORT)
      list(APPEND ${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES cartpole)
    endif()

    foreach(EXAMPLE ${${PROJECT_NAME}_CASADI_PYTHON_EXAMPLES})
      set(EXAMPLE_NAME "${PROJECT_NAME}-example-py-casadi-${EXAMPLE}")
      add_python_unit_test(${EXAMPLE_NAME} "examples/casadi/${EXAMPLE}.py" "bindings/python")
      add_windows_dll_path_to_test(${EXAMPLE_NAME})
    endforeach()
  endif()
endif()
