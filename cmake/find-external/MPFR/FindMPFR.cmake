# Copyright 2026 Inria

find_path(MPFR_INCLUDE NAMES mpfr.h)
find_library(MPFR_LIBRARY mpfr)

mark_as_advanced(MPFR_LIBRARY MPFR_INCLUDE)

if(MPFR_INCLUDE AND NOT TARGET MPFR::MPFR)
    file(READ "${MPFR_INCLUDE}/mpfr.h" mpfr_h)

    string(
        REGEX MATCH "define[ \t]+MPFR_VERSION_MAJOR[ \t]+([0-9]+)"
        _mpfr_major_version_match
        "${mpfr_h}"
    )
    set(MPFR_MAJOR_VERSION "${CMAKE_MATCH_1}")
    string(
        REGEX MATCH "define[ \t]+MPFR_VERSION_MINOR[ \t]+([0-9]+)"
        _mpfr_minor_version_match
        "${mpfr_h}"
    )
    set(MPFR_MINOR_VERSION "${CMAKE_MATCH_1}")
    string(
        REGEX MATCH "define[ \t]+MPFR_VERSION_PATCHLEVEL[ \t]+([0-9]+)"
        _mpfr_patchlevel_version_match
        "${mpfr_h}"
    )
    set(MPFR_PATCHLEVEL_VERSION "${CMAKE_MATCH_1}")

    set(MPFR_VERSION
        ${MPFR_MAJOR_VERSION}.${MPFR_MINOR_VERSION}.${MPFR_PATCHLEVEL_VERSION}
    )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    MPFR
    REQUIRED_VARS MPFR_LIBRARY MPFR_INCLUDE
    VERSION_VAR MPFR_VERSION
)

# mpfr depend of gmp
find_package(GMP REQUIRED)

if(MPFR_FOUND AND NOT TARGET MPFR::MPFR)
    add_library(MPFR::MPFR UNKNOWN IMPORTED)
    set_target_properties(
        MPFR::MPFR
        PROPERTIES
            IMPORTED_LOCATION ${MPFR_LIBRARY}
            VERSION ${MPFR_VERSION}
            INCLUDE_DIRECTORIES ${MPFR_INCLUDE}
            INTERFACE_LINK_LIBRARIES GMP::GMP
    )
endif()
