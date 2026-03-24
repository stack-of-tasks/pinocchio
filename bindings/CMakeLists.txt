#
# Copyright (c) 2016-2019 CNRS INRIA, 2026 INRIA
#

set(BINDINGS_FRAMEWORK
    Boost
    CACHE STRING "Choose the bindings framework." FORCE)
# Set the possible values for the bindings framework
set_property(CACHE BINDINGS_FRAMEWORK PROPERTY STRINGS "Boost" "Nanobind")
message(STATUS "Setting Python bindings framework to ${BINDINGS_FRAMEWORK}")

if(BINDINGS_FRAMEWORK STREQUAL Boost)
  add_subdirectory(python)
else()
  add_subdirectory(nanobind)
endif()

if(BUILD_PYTHON_INTERFACE)
    set(PYWRAP ${PYWRAP} PARENT_SCOPE)
endif(BUILD_PYTHON_INTERFACE)
