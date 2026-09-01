cmake_minimum_required(VERSION 3.22)

project(pinocchio-casadi-legacy-target-smoke LANGUAGES CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/pinocchio-casadi.cmake")

add_library(casadi INTERFACE IMPORTED)
set(casadi_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")

pinocchio_prepare_casadi_target(PINOCCHIO_CASADI_LINK_TARGET)

if(NOT PINOCCHIO_CASADI_LINK_TARGET STREQUAL "casadi::casadi")
    message(FATAL_ERROR "Expected Pinocchio to normalize the CasADi link target name.")
endif()

get_target_property(_pinocchio_casadi_include_dirs casadi INTERFACE_INCLUDE_DIRECTORIES)

if(NOT _pinocchio_casadi_include_dirs MATCHES "include")
    message(
        FATAL_ERROR
        "Expected the legacy casadi target to pick up fallback include directories."
    )
endif()
