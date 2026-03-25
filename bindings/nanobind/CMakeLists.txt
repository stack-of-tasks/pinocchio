#
# Copyright (c) 2026 INRIA
#

# Try cmake config first (system install), then fall back to Python module.
find_package(nanobind CONFIG QUIET)
if(NOT nanobind_FOUND)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE _NB_CMAKE_DIR
        RESULT_VARIABLE _NB_RESULT
    )
    if(NOT _NB_RESULT EQUAL 0)
        message(
            FATAL_ERROR
            "nanobind not found. Install it with: pip install nanobind"
        )
    endif()
    list(APPEND CMAKE_PREFIX_PATH "${_NB_CMAKE_DIR}")
    find_package(nanobind CONFIG REQUIRED)
endif()

# --- PYTHON TARGET --- #
set(PYWRAP ${PROJECT_NAME}_pywrap)
set(PYWRAP ${PYWRAP} PARENT_SCOPE)

set(PYWRAP_SOURCES module.cpp expose-spatial.cpp expose-multibody.cpp expose-algorithm.cpp
                   algorithm/aba.cpp multibody/sample-models.cpp)

file(
    GLOB_RECURSE PYWRAP_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python-nb/*.hpp
)

add_custom_target(${PROJECT_NAME}-python)
set_target_properties(
    ${PROJECT_NAME}-python
    PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
)

if(IS_ABSOLUTE ${PYTHON_SITELIB})
    set(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
else()
    set(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
endif()
set(PINOCCHIO_PYTHON_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})

# --- BUILD MODULE --- #
set(NB_MODULE_NAME ${PYWRAP}_default)

nanobind_add_module(${NB_MODULE_NAME} NB_STATIC NB_SUPPRESS_WARNINGS ${PYWRAP_SOURCES}
                    ${PYWRAP_HEADERS})

add_dependencies(${PROJECT_NAME}-python ${NB_MODULE_NAME})

target_link_libraries(${NB_MODULE_NAME} PUBLIC ${PROJECT_NAME}::${PROJECT_NAME})

if(BUILD_WITH_URDF_SUPPORT)
    # Link directly against console_bridge since we bind some enums and call
    # console_bridge::setLogLevel function.
    modernize_target_link_libraries(
      ${NB_MODULE_NAME}
      SCOPE PUBLIC
      TARGETS console_bridge::console_bridge
      LIBRARIES ${console_bridge_LIBRARIES}
      INCLUDE_DIRS ${console_bridge_INCLUDE_DIRS}
    )
endif()

if(BUILD_WITH_COLLISION_SUPPORT)
    target_compile_definitions(
        ${NB_MODULE_NAME}
        PRIVATE PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS
    )
endif()

if(BUILD_WITH_OPENMP_SUPPORT)
    # OpenMP is linked with pinocchio_parallel target
    target_compile_definitions(
        ${NB_MODULE_NAME}
        PRIVATE PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
    )
endif()

if(BUILD_WITH_ACCELERATE_SUPPORT)
    # todo: probably use FindAccelerate instead?
    target_link_libraries(${NB_MODULE_NAME} PRIVATE "-framework accelerate")
endif()

target_compile_definitions(
    ${NB_MODULE_NAME}
    PRIVATE PINOCCHIO_PYTHON_MODULE_NAME=${NB_MODULE_NAME}
)

set_target_properties(
    ${NB_MODULE_NAME}
    PROPERTIES
        OUTPUT_NAME "${NB_MODULE_NAME}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        # On Windows, shared libraries are treated as binaries
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        VERSION ${PROJECT_VERSION}
)

if(UNIX)
    get_relative_rpath(${PINOCCHIO_PYTHON_INSTALL_DIR} _NB_INSTALL_RPATH)
    set_target_properties(
        ${NB_MODULE_NAME}
        PROPERTIES INSTALL_RPATH "${_NB_INSTALL_RPATH}"
    )
endif()

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

install_python_files(FILES __init__.py)

install(
    TARGETS ${NB_MODULE_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${PINOCCHIO_PYTHON_INSTALL_DIR}
)

# --- STUBS --- #
if(GENERATE_PYTHON_STUBS)
    nanobind_add_stub(
      ${NB_MODULE_NAME}_stub
      VERBOSE
      MODULE
      ${NB_MODULE_NAME}
      OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${NB_MODULE_NAME}.pyi"
      PYTHON_PATH
      "${CMAKE_CURRENT_BINARY_DIR}"
      DEPENDS
      ${NB_MODULE_NAME}
      MARKER_FILE
      "${CMAKE_CURRENT_BINARY_DIR}/py.typed"
    )

    install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/${NB_MODULE_NAME}.pyi"
        DESTINATION ${PINOCCHIO_PYTHON_INSTALL_DIR}
    )
    install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/py.typed"
        DESTINATION ${PINOCCHIO_PYTHON_INSTALL_DIR}
    )
endif(GENERATE_PYTHON_STUBS)
