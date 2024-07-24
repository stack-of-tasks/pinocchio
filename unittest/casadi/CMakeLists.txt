#
# Copyright (c) 2021 INRIA
#

macro(ADD_CASADI_UNIT_TEST name)
  add_pinocchio_unit_test(${name} PACKAGES casadi)
  get_cpp_test_name(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 3000)
  add_dependencies(${PROJECT_NAME}-test-casadi ${TEST_NAME})
endmacro()

if(BUILD_WITH_CASADI_SUPPORT)
  add_custom_target(${PROJECT_NAME}-test-casadi)
  set_target_properties(${PROJECT_NAME}-test-casadi PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  add_casadi_unit_test(basic)
  add_casadi_unit_test(spatial)
  add_casadi_unit_test(explog)
  add_casadi_unit_test(joints)
  add_casadi_unit_test(algorithms)
  if(BUILD_ADVANCED_TESTING)
    add_casadi_unit_test(aba)
    add_casadi_unit_test(constraint-dynamics-derivatives)
    add_casadi_unit_test(constraint-dynamics)
    add_casadi_unit_test(aba-derivatives)
  endif()
  add_casadi_unit_test(integrate-derivatives)
  add_casadi_unit_test(rnea-derivatives)
endif()
