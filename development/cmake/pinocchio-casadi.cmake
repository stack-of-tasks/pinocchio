include_guard(GLOBAL)

include(CheckIncludeFileCXX)

function(pinocchio_prepare_casadi_target out_var)
    if(TARGET casadi::casadi)
        set(_pinocchio_casadi_target casadi::casadi)
        set(_pinocchio_casadi_target_to_patch casadi::casadi)
    elseif(TARGET casadi)
        set(_pinocchio_casadi_target casadi)
        set(_pinocchio_casadi_target_to_patch casadi)
    else()
        message(
            FATAL_ERROR
            "CasADi support was requested, but neither casadi::casadi nor casadi was exported by the package."
        )
    endif()

    get_target_property(
        _pinocchio_casadi_include_dirs
        ${_pinocchio_casadi_target_to_patch}
        INTERFACE_INCLUDE_DIRECTORIES
    )

    if(NOT _pinocchio_casadi_include_dirs)
        foreach(
            _pinocchio_casadi_include_var
            IN ITEMS
                casadi_INCLUDE_DIRS
                casadi_INCLUDE_DIR
                CASADI_INCLUDE_DIRS
                CASADI_INCLUDE_DIR
        )
            if(
                DEFINED ${_pinocchio_casadi_include_var}
                AND NOT "${${_pinocchio_casadi_include_var}}" STREQUAL ""
            )
                set_property(
                    TARGET ${_pinocchio_casadi_target_to_patch}
                    APPEND
                    PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                             "${${_pinocchio_casadi_include_var}}"
                )
            endif()
        endforeach()

        get_target_property(
            _pinocchio_casadi_include_dirs
            ${_pinocchio_casadi_target_to_patch}
            INTERFACE_INCLUDE_DIRECTORIES
        )
    endif()

    set(_pinocchio_saved_required_includes "${CMAKE_REQUIRED_INCLUDES}")
    if(_pinocchio_casadi_include_dirs)
        set(CMAKE_REQUIRED_INCLUDES "${_pinocchio_casadi_include_dirs}")
    else()
        unset(CMAKE_REQUIRED_INCLUDES)
    endif()

    check_include_file_cxx("casadi/casadi.hpp" PINOCCHIO_CASADI_HEADER_FOUND)

    set(CMAKE_REQUIRED_INCLUDES "${_pinocchio_saved_required_includes}")

    if(NOT PINOCCHIO_CASADI_HEADER_FOUND)
        message(
            FATAL_ERROR
            "CasADi support requires the exported target to expose casadi headers. "
            "Set casadi_INCLUDE_DIRS/CASADI_INCLUDE_DIRS or use a package that exports "
            "INTERFACE_INCLUDE_DIRECTORIES."
        )
    endif()

    if(NOT TARGET casadi::casadi AND TARGET casadi)
        add_library(casadi::casadi ALIAS casadi)
    endif()

    set(${out_var} casadi::casadi PARENT_SCOPE)
endfunction()
