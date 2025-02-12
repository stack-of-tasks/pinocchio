#
# Copyright (c) 2022 INRIA
#

if(BUILD_WITH_AUTODIFF_SUPPORT)
  function(ADD_PINOCCHIO_CPP_AUTODIFF_EXAMPLE EXAMPLE)
    include_directories(SYSTEM ${cppadcg_INCLUDE_DIR})
    add_pinocchio_cpp_example(${EXAMPLE})
    target_link_libraries(
      ${PROJECT_NAME}-example-cpp-${EXAMPLE}
      PUBLIC ${CMAKE_DL_LIBS} ${cppad_LIBRARY}
    )
  endfunction()

  add_pinocchio_cpp_autodiff_example(autodiff-rnea)

  if(BUILD_PYTHON_INTERFACE)
    set(${PROJECT_NAME}_AUTODIFF_PYTHON_EXAMPLES autodiff-rnea)

    foreach(EXAMPLE ${${PROJECT_NAME}_AUTODIFF_PYTHON_EXAMPLES})
      set(EXAMPLE_NAME "${PROJECT_NAME}-example-py-${EXAMPLE}")
      ADD_PYTHON_UNIT_TEST(${EXAMPLE_NAME} "examples/cppad/${EXAMPLE}.py" "bindings/python")
      ADD_WINDOWS_DLL_PATH_TO_TEST(${EXAMPLE_NAME})
    endforeach()
  endif()
endif()
