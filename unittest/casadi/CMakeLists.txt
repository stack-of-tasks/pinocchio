#
# Copyright (c) 2021 INRIA
#

macro(ADD_CASADI_UNIT_TEST name)
  ADD_PINOCCHIO_UNIT_TEST(${name} PACKAGES casadi)
  GET_CPP_TEST_NAME(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 3000)
  add_dependencies(${PROJECT_NAME}-test-casadi ${TEST_NAME})
endmacro()

if(BUILD_WITH_CASADI_SUPPORT)
  add_custom_target(${PROJECT_NAME}-test-casadi)
  set_target_properties(${PROJECT_NAME}-test-casadi PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  ADD_CASADI_UNIT_TEST(basic)
  ADD_CASADI_UNIT_TEST(spatial)
  ADD_CASADI_UNIT_TEST(explog)
  ADD_CASADI_UNIT_TEST(joints)
  ADD_CASADI_UNIT_TEST(algorithms)
  if(BUILD_ADVANCED_TESTING)
    ADD_CASADI_UNIT_TEST(aba)
    ADD_CASADI_UNIT_TEST(constraint-dynamics-derivatives)
    ADD_CASADI_UNIT_TEST(constraint-dynamics)
    ADD_CASADI_UNIT_TEST(aba-derivatives)
  endif()
  ADD_CASADI_UNIT_TEST(integrate-derivatives)
  ADD_CASADI_UNIT_TEST(rnea-derivatives)
endif()
