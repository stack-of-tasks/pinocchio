macro(ADD_CPPADCG_UNIT_TEST name)
  include_directories(SYSTEM ${cppad_INCLUDE_DIR})
  include_directories(SYSTEM ${cppadcodegen_INCLUDE_DIR})
  ADD_PINOCCHIO_UNIT_TEST(${name} PACKAGES ${cppad_LIBRARY} ${CMAKE_DL_LIBS})
  GET_CPP_TEST_NAME(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  add_dependencies(${PROJECT_NAME}-test-cpp-cppadcg ${TEST_NAME})
  target_compile_definitions(${TEST_NAME} PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")
endmacro()

if(BUILD_WITH_AUTODIFF_SUPPORT)
  if(BUILD_WITH_CODEGEN_SUPPORT)
    add_custom_target(${PROJECT_NAME}-test-cpp-cppadcg)
    set_target_properties(
      ${PROJECT_NAME}-test-cpp-cppadcg
      PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
    )

    ADD_CPPADCG_UNIT_TEST(basic)
    if(BUILD_ADVANCED_TESTING)
      ADD_CPPADCG_UNIT_TEST(algorithms)
      ADD_CPPADCG_UNIT_TEST(joint-configurations)
    endif()
  endif()
endif()
