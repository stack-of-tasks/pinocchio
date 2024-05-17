#
# Copyright (c) 2021 INRIA
#

# Automatic differentiation
macro(ADD_CPPAD_UNIT_TEST name)
  include_directories(SYSTEM ${cppad_INCLUDE_DIR})
  add_pinocchio_unit_test(${name} PACKAGES ${cppad_LIBRARY})
  add_dependencies(test-cpp-cppad test-cpp-cppad-${name})
endmacro()

if(BUILD_WITH_AUTODIFF_SUPPORT)
  add_custom_target(test-cpp-cppad)
  set_target_properties(test-cpp-cppad PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  add_cppad_unit_test(basic)
  add_cppad_unit_test(spatial)
  add_cppad_unit_test(joints)
  add_cppad_unit_test(algorithms)
  add_cppad_unit_test(derivatives)
  add_cppad_unit_test(joint-configurations)
endif(BUILD_WITH_AUTODIFF_SUPPORT)
