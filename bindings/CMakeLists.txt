#
# Copyright (c) 2016-2019 CNRS INRIA, 2026 INRIA
#

message(STATUS "Python bindings framework: ${BINDINGS_FRAMEWORK}")

if(BINDINGS_FRAMEWORK STREQUAL Boost)
    add_subdirectory(python)
else()
    find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
    # jrl-cmakemodules uses PYTHON_EXECUTABLE defined in FINDPYTHON macro.
    # Since we doesn't call it in this branch, we must redefine it.
    set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})
    add_subdirectory(nanobind)
endif()

if(BUILD_PYTHON_INTERFACE)
    set(PYWRAP ${PYWRAP} PARENT_SCOPE)
endif(BUILD_PYTHON_INTERFACE)
