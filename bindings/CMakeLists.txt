#
# Copyright (c) 2016-2019 CNRS INRIA, 2026 INRIA
#

message(STATUS "Python bindings framework: ${BINDINGS_FRAMEWORK}")

# Capture this directory so install_python_files() resolves Python sources
# relative to bindings/ regardless of which subdirectory calls it.
# PYTHON_BUILD is a macro and inherits CMAKE_CURRENT_SOURCE_DIR from its
# call-site; we shadow it inside the function to point here.
set(_BINDINGS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

function(install_python_files)
    set(options)
    set(oneValueArgs MODULE)
    set(multiValueArgs FILES)
    cmake_parse_arguments(
        ARGS
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )

    set(SOURCE_PATH ${PROJECT_NAME})
    set(INSTALL_PATH ${PINOCCHIO_PYTHON_INSTALL_DIR})
    if(ARGS_MODULE)
        set(SOURCE_PATH ${SOURCE_PATH}/${ARGS_MODULE})
        set(INSTALL_PATH ${INSTALL_PATH}/${ARGS_MODULE})
    endif()

    # Override CMAKE_CURRENT_SOURCE_DIR so the PYTHON_BUILD macro (which is a
    # macro and therefore sees this scope) finds the Python sources under
    # bindings/ rather than the subdirectory that called this function.
    set(CMAKE_CURRENT_SOURCE_DIR "${_BINDINGS_SOURCE_DIR}")

    foreach(f ${ARGS_FILES})
        python_build(${SOURCE_PATH} ${f})
        install(
            FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_PATH}/${f}
            DESTINATION ${INSTALL_PATH}
        )
    endforeach()
endfunction()

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
