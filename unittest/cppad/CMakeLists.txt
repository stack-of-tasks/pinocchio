#
# Copyright (c) 2021 INRIA
#

# Automatic differentiation
macro(ADD_CPPAD_UNIT_TEST name)
  include_directories(SYSTEM ${cppad_INCLUDE_DIR})
  ADD_PINOCCHIO_UNIT_TEST(${name} PACKAGES ${cppad_LIBRARY})
  GET_CPP_TEST_NAME(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  add_dependencies(${PROJECT_NAME}-test-cpp-cppad ${TEST_NAME})
endmacro()

if(BUILD_WITH_AUTODIFF_SUPPORT)
  add_custom_target(${PROJECT_NAME}-test-cpp-cppad)
  set_target_properties(${PROJECT_NAME}-test-cpp-cppad PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  ADD_CPPAD_UNIT_TEST(basic)
  ADD_CPPAD_UNIT_TEST(spatial)
  ADD_CPPAD_UNIT_TEST(joints)
  ADD_CPPAD_UNIT_TEST(algorithms)
  ADD_CPPAD_UNIT_TEST(derivatives)
  ADD_CPPAD_UNIT_TEST(joint-configurations)
endif()
