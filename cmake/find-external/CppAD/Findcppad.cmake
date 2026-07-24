# Copyright 2026 Inria

find_path(cppad_INCLUDE_DIR NAMES cppad/configure.hpp)
find_library(cppad_LIBRARY NAMES cppad_lib)

mark_as_advanced(cppad_LIBRARY cppad_INCLUDE_DIR)

if(cppad_INCLUDE_DIR AND NOT TARGET cppad::cppad)
  file(
    READ "${cppad_INCLUDE_DIR}/cppad/configure.hpp"
    cppad_configure_hpp
  )
  # Version is stored on the following line:
  # `# define CPPAD_PACKAGE_STRING "cppad-20260000.0"`
  string(
    REGEX MATCH
    "#[\t ]+define[\t ]+CPPAD_PACKAGE_STRING[\t ]+\"cppad-([0-9\.]*)\""
    _
    ${cppad_configure_hpp}
  )
  set(cppad_VERSION ${CMAKE_MATCH_1})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  cppad
  REQUIRED_VARS cppad_LIBRARY cppad_INCLUDE_DIR
  VERSION_VAR cppad_VERSION
)

if(cppad_FOUND AND NOT TARGET cppad::cppad)
    add_library(cppad::cppad UNKNOWN IMPORTED)
    set_target_properties(
        cppad::cppad
        PROPERTIES
            IMPORTED_LOCATION ${cppad_LIBRARY}
            VERSION ${cppad_VERSION}
            INCLUDE_DIRECTORIES ${cppad_INCLUDE_DIR}
    )
endif()
