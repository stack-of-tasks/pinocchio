# Copyright 2026 Inria

find_path(
  cppadcg_INCLUDE_DIR
  NAMES cppad/cg.hpp
)

mark_as_advanced(cppadcg_INCLUDE_DIR)

if(cppadcg_INCLUDE_DIR AND NOT TARGET cppadcg::cppadcg)
  file(
    READ "${cppadcg_INCLUDE_DIR}/cppad/cg/configure.hpp"
    cppadcg_configure_hpp
  )
  # Version is stored on the following line:
  # `#define CPPAD_CG_VERSION "cppadcg-2.5.0"`
  string(
    REGEX MATCH
    "#define[\t ]+CPPAD_CG_VERSION[\t ]+\"cppadcg-([0-9\.]*)\""
    _
    ${cppadcg_configure_hpp}
  )
  set(cppadcg_VERSION ${CMAKE_MATCH_1})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  cppadcg
  REQUIRED_VARS cppadcg_INCLUDE_DIR
  VERSION_VAR cppadcg_VERSION
)

include(CMakeFindDependencyMacro)
find_dependency(cppad REQUIRED)

if(cppadcg_FOUND AND NOT TARGET cppadcg::cppadcg)
    add_library(cppadcg::cppadcg INTERFACE IMPORTED)
    set_target_properties(
        cppadcg::cppadcg
        PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${cppadcg_INCLUDE_DIR}
            INTERFACE_VERSION ${cppadcg_VERSION}
            INTERFACE_LINK_LIBRARIES cppad::cppad
    )
endif()
