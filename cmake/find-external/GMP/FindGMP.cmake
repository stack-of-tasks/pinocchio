# Copyright 2026 Inria

find_path(GMP_INCLUDE_DIR NAMES gmp.h)
find_library(GMP_LIBRARY NAMES gmp)

mark_as_advanced(GMP_LIBRARY GMP_INCLUDE_DIR)

if(GMP_INCLUDE_DIR AND NOT TARGET GMP::GMP)
    # Version is stored on the following line:
    # `#define __GNU_MP_VERSION            6`
    # `#define __GNU_MP_VERSION_MINOR      3`
    # `#define __GNU_MP_VERSION_PATCHLEVEL 0`
    file(READ ${GMP_INCLUDE_DIR}/gmp.h gmp_h)
    string(
        REGEX MATCH "#define[ \t]__GNU_MP_VERSION[ \t]+([0-9]+)"
        _match_major
        ${gmp_h}
    )
    set(GMP_VERSION_MAJOR ${CMAKE_MATCH_1})
    string(
        REGEX MATCH "#define[ \t]__GNU_MP_VERSION_MINOR[ \t]+([0-9]+)"
        _match_major
        ${gmp_h}
    )
    set(GMP_VERSION_MINOR ${CMAKE_MATCH_1})
    string(
        REGEX MATCH "#define[ \t]__GNU_MP_VERSION_PATCHLEVEL[ \t]+([0-9]+)"
        _match_major
        ${gmp_h}
    )
    set(GMP_VERSION_MICRO ${CMAKE_MATCH_1})
    set(GMP_VERSION
        ${GMP_VERSION_MAJOR}.${GMP_VERSION_MINOR}.${GMP_VERSION_MICRO}
    )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    GMP
    REQUIRED_VARS GMP_LIBRARY GMP_INCLUDE_DIR
    VERSION_VAR GMP_VERSION
)

if(GMP_FOUND AND NOT TARGET GMP::GMP)
    add_library(GMP::GMP UNKNOWN IMPORTED)
    set_target_properties(
        GMP::GMP
        PROPERTIES
            IMPORTED_LOCATION ${GMP_LIBRARY}
            VERSION ${GMP_VERSION}
            INCLUDE_DIRECTORIES ${GMP_INCLUDE_DIR}
    )
endif()
