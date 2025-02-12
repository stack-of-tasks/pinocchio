#
# Copyright (c) 2020-2023 INRIA
#

if(cppadcg_FOUND AND BUILD_WITH_CODEGEN_SUPPORT AND BUILD_WITH_URDF_SUPPORT)
  function(ADD_PINOCCHIO_CPP_CODEGEN_EXAMPLE EXAMPLE)
    add_pinocchio_cpp_example(
      ${EXAMPLE}
      CPPADCG
      PARSERS
    )
    target_link_libraries(${PROJECT_NAME}-example-cpp-${EXAMPLE} PUBLIC ${CMAKE_DL_LIBS})
    target_compile_definitions(
      ${PROJECT_NAME}-example-cpp-${EXAMPLE}
      PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
    )
  endfunction()

  add_pinocchio_cpp_codegen_example(codegen-crba)
  add_pinocchio_cpp_codegen_example(codegen-rnea)

  if(BUILD_PYTHON_INTERFACE)
    set(${PROJECT_NAME}_CODEGEN_PYTHON_EXAMPLES codegen-rnea)

    foreach(EXAMPLE ${${PROJECT_NAME}_CODEGEN_PYTHON_EXAMPLES})
      set(EXAMPLE_NAME "${PROJECT_NAME}-example-py-${EXAMPLE}")
      ADD_PYTHON_UNIT_TEST(${EXAMPLE_NAME} "examples/codegen/${EXAMPLE}.py" "bindings/python")
      ADD_WINDOWS_DLL_PATH_TO_TEST(${EXAMPLE_NAME})
    endforeach()
  endif()
endif()
