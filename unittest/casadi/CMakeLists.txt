#
# Copyright (c) 2021 INRIA
#

MACRO(ADD_CASADI_UNIT_TEST name)
  ADD_PINOCCHIO_UNIT_TEST(${name} PACKAGES casadi)
  SET_PROPERTY(TARGET test-cpp-casadi-${name} PROPERTY CXX_STANDARD 11)
  SET_TESTS_PROPERTIES(test-cpp-casadi-${name} PROPERTIES TIMEOUT 3000)
  TARGET_LINK_LIBRARIES(test-cpp-casadi-${name} PUBLIC casadi)
  ADD_DEPENDENCIES(test-casadi test-cpp-casadi-${name})
ENDMACRO()

IF(BUILD_WITH_CASADI_SUPPORT)
  ADD_CUSTOM_TARGET(test-casadi)
  SET_TARGET_PROPERTIES(test-casadi PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  ADD_CASADI_UNIT_TEST(basic)
  ADD_CASADI_UNIT_TEST(spatial)
  ADD_CASADI_UNIT_TEST(joints)
  ADD_CASADI_UNIT_TEST(algorithms)
  ADD_CASADI_UNIT_TEST(aba)
  ADD_CASADI_UNIT_TEST(explog)
  ADD_CASADI_UNIT_TEST(constraint-dynamics-derivatives)
  ADD_CASADI_UNIT_TEST(constraint-dynamics)
  ADD_CASADI_UNIT_TEST(aba-derivatives)
  ADD_CASADI_UNIT_TEST(integrate-derivatives)
  ADD_CASADI_UNIT_TEST(rnea-derivatives)
ENDIF(BUILD_WITH_CASADI_SUPPORT)

