#
# Copyright (c) 2021 INRIA
#

macro(ADD_CASADI_UNIT_TEST name)
  add_pinocchio_unit_test(${name} PACKAGES casadi)
  set_property(TARGET test-cpp-casadi-${name} PROPERTY CXX_STANDARD 11)
  set_tests_properties(test-cpp-casadi-${name} PROPERTIES TIMEOUT 3000)
  add_dependencies(test-casadi test-cpp-casadi-${name})
endmacro()

if(BUILD_WITH_CASADI_SUPPORT)
  add_custom_target(test-casadi)
  set_target_properties(test-casadi PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

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
endif(BUILD_WITH_CASADI_SUPPORT)
