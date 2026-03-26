#
# Copyright (c) 2016-2019 CNRS INRIA, 2026 INRIA
#

message(STATUS "Python bindings framework: ${BINDINGS_FRAMEWORK}")

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

    foreach(f ${ARGS_FILES})
        python_build(${SOURCE_PATH} ${f})
        install(FILES ${SOURCE_PATH}/${f} DESTINATION ${INSTALL_PATH})
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
