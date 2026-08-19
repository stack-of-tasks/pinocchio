#
# Copyright (c) 2015-2025 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME pinocchio)
set(PROJECT_DESCRIPTION
    "A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives"
)
set(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
    set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)
set(PINOCCHIO_PROJECT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})
# To enable jrl-cmakemodules compatibility with workspace we must define the two following lines
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# ----------------------------------------------------
# --- OPTIONS  ---------------------------------------
# Need to be set before including base.cmake
# ----------------------------------------------------
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
    message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
    find_package(jrl-cmakemodules QUIET CONFIG)
    if(jrl-cmakemodules_FOUND)
        get_property(
            JRL_CMAKE_MODULES
            TARGET jrl-cmakemodules::jrl-cmakemodules
            PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        )
        message(
            STATUS
            "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}"
        )
    else()
        message(STATUS "JRL cmakemodules not found. Let's fetch it.")
        include(FetchContent)
        FetchContent_Declare(
            "jrl-cmakemodules"
            GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
        )
        FetchContent_MakeAvailable("jrl-cmakemodules")
        FetchContent_GetProperties(
            "jrl-cmakemodules"
            SOURCE_DIR JRL_CMAKE_MODULES
        )
    endif()
endif()

set(DOXYGEN_USE_MATHJAX YES)

# ----------------------------------------------------
# --- Policy -----------------------------------------
# CMake Policy setup
# ----------------------------------------------------
# Policy can be removed when cmake_minimum_required is updated.

# We also set CMAKE_POLICY_DEFAULT_CMPXXXX because CMake modules can reset policy and redefine some
# macros like `find_dependency` that will not use our policy.

# Use BoostConfig module distributed by boost library instead of using FindBoost module distributed
# by CMake (to remove in 3.30). This policy is not working when using clang-cl.
if(NOT WIN32 OR NOT ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    if(POLICY CMP0167)
        cmake_policy(SET CMP0167 NEW)
        set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
    endif()
endif()
# install() DESTINATION paths are normalized (to remove in 3.31).
if(POLICY CMP0177)
    cmake_policy(SET CMP0177 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0177 NEW)
endif()
include("${JRL_CMAKE_MODULES}/base.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

set(INSTALL_PKG_CONFIG_FILE OFF CACHE BOOL "" FORCE)

include("${JRL_CMAKE_MODULES}/tracy.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")
include(CMakeDependentOption)

# If needed, set CMake policy for APPLE systems
apply_default_apple_configuration()

# Force C++ standard to be C++17 at least
check_minimal_cxx_standard(17 ENFORCE)

# --- OPTIONS ----------------------------------------
option(BUILD_BENCHMARK "Build the benchmarks" OFF)
option(BUILD_EXAMPLES "Build the examples" ON)
option(BUILD_UTILS "Build the utils" OFF)
option(
    GENERATE_PYTHON_STUBS
    "Generate the Python stubs associated to the Python library"
    OFF
)
option(
    BUILD_WITH_COMMIT_VERSION
    "Build libraries by setting specific commit version"
    OFF
)

if(DEFINED BUILD_UNIT_TESTS)
    message(
        AUTHOR_WARNING
        "BUILD_UNIT_TESTS is deprecated. Use BUILD_TESTING instead.\
    If you are manually building Pinocchio from source in an existing build folder,\
    we suggest that you delete your build folder and make a new one."
    )
    set(BUILD_TESTING ${BUILD_UNIT_TESTS})
endif(DEFINED BUILD_UNIT_TESTS)

option(
    BUILD_ADVANCED_TESTING
    "Build the advanced tests (multiprecision, etc.) of Pinocchio"
    OFF
)
option(PINOCCHIO_BUILD_MPFR_TESTING "Build MPFR dependent tests" OFF)

# --- OPTIONAL DEPENDENCIES -------------------------
option(
    BUILD_WITH_URDF_SUPPORT
    "Build the library with the URDF format support"
    ON
)
option(
    BUILD_WITH_SDF_SUPPORT
    "Build the library with the SDF format support"
    OFF
)
option(
    BUILD_WITH_COLLISION_SUPPORT
    "Build the library with the collision support (required coal)"
    OFF
)
option(
    BUILD_WITH_AUTODIFF_SUPPORT
    "Build the library with the automatic differentiation support (via CppAD)"
    OFF
)
option(
    BUILD_WITH_CASADI_SUPPORT
    "Build the library with the support of CASADI"
    OFF
)
# cppadcodgen don't build well with CL
if(NOT WIN32)
    option(
        BUILD_WITH_CODEGEN_SUPPORT
        "Build the library with the support of code generation (via CppADCodeGen)"
        OFF
    )
endif()
option(
    BUILD_WITH_OPENMP_SUPPORT
    "Build the library with the OpenMP support"
    OFF
)
cmake_dependent_option(
    BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT
    "Build the Python interface with Boost.Multiprecision MPFR support"
    OFF
    BUILD_PYTHON_INTERFACE
    OFF
)
if(BUILD_PYTHON_INTERFACE)
    set(BINDINGS_FRAMEWORK
        "Boost"
        CACHE STRING
        "Python bindings framework to use."
    )
    set_property(CACHE BINDINGS_FRAMEWORK PROPERTY STRINGS "Boost" "Nanobind")
endif()
cmake_dependent_option(
    BUILD_WITH_LIBPYTHON
    "Build the library with Python format support"
    ON
    "BUILD_PYTHON_INTERFACE"
    OFF
)
if(APPLE)
    option(
        BUILD_WITH_ACCELERATE_SUPPORT
        "Build Pinocchio with the Accelerate support"
        OFF
    )
else(APPLE)
    set(BUILD_WITH_ACCELERATE_SUPPORT FALSE)
endif(APPLE)

option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
option(
    ASSERT_ON_EXCEPTION
    "Turns Pinocchio excpetions into asserts for debug purposes"
    OFF
)

option(
    ENABLE_TEMPLATE_INSTANTIATION
    "Template instantiation of the main library"
    ON
)

option(
    PINOCCHIO_BUILD_WITH_TRACY
    "Build with tracy profiler for performance analysis"
    OFF
)

option(
    BUILD_WITH_RTSAN
    "Build with the Realtime Sanitizer (clang >=20, linux/mac only)"
    OFF
)

option(
    PINOCCHIO_INSTALL_DEPRECATED_HEADERS
    "Use and install deprecated headers"
    ON
)

set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/models")
if(BUILD_BENCHMARK OR BUILD_EXAMPLES OR BUILD_TESTING)
    # example-robot-data is required
    if(EXAMPLE_ROBOT_DATA_MODEL_DIR)
        file(
            REAL_PATH "${EXAMPLE_ROBOT_DATA_MODEL_DIR}"
            EXAMPLE_ROBOT_DATA_MODEL_DIR
            EXPAND_TILDE
        )
        message(
            STATUS
            "Using provided example-robot-data at: ${EXAMPLE_ROBOT_DATA_MODEL_DIR}"
        )
    elseif(
        EXISTS
            "${CMAKE_CURRENT_LIST_DIR}/models/example-robot-data/CMakeLists.txt"
    )
        set(EXAMPLE_ROBOT_DATA_MODEL_DIR
            "${PINOCCHIO_MODEL_DIR}/example-robot-data/robots"
        )
    else()
        find_package(example-robot-data REQUIRED CONFIG)
        get_target_property(
            EXAMPLE_ROBOT_DATA_MODEL_DIR_DEF
            example-robot-data::example-robot-data
            INTERFACE_COMPILE_DEFINITIONS
        )
        string(
            REPLACE "EXAMPLE_ROBOT_DATA_MODEL_DIR="
            ""
            EXAMPLE_ROBOT_DATA_MODEL_DIR_QUOTES
            "${EXAMPLE_ROBOT_DATA_MODEL_DIR_DEF}"
        )
        string(
            REPLACE "\""
            ""
            EXAMPLE_ROBOT_DATA_MODEL_DIR
            "${EXAMPLE_ROBOT_DATA_MODEL_DIR_QUOTES}"
        )
        message(
            STATUS
            "Using system example-robot-data at: ${EXAMPLE_ROBOT_DATA_MODEL_DIR}"
        )
    endif()
endif()

if(BUILD_WITH_CODEGEN_SUPPORT)
    set(BUILD_WITH_CPPAD_CODEGEN_SUPPORT TRUE)
    set(BUILD_WITH_AUTODIFF_SUPPORT ON)
endif()
if(BUILD_WITH_AUTODIFF_SUPPORT)
    set(BUILD_WITH_CPPAD_SUPPORT TRUE)
endif()
if(BUILD_WITH_LIBPYTHON AND BINDINGS_FRAMEWORK STREQUAL Boost)
    set(BUILD_WITH_PYTHON_PARSER_SUPPORT TRUE)
endif()

if(INITIALIZE_WITH_NAN)
    message(STATUS "Initialize with NaN all the Eigen entries.")
endif(INITIALIZE_WITH_NAN)

if(ENABLE_TEMPLATE_INSTANTIATION)
    message(STATUS "Template instantiation of the main library")
endif(ENABLE_TEMPLATE_INSTANTIATION)

macro(TAG_LIBRARY_VERSION target)
    set_target_properties(${target} PROPERTIES SOVERSION ${PROJECT_VERSION})
endmacro(TAG_LIBRARY_VERSION)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
set(CMAKE_MODULE_PATH
    "${JRL_CMAKE_MODULES}/find-external/CppAD/"
    ${CMAKE_MODULE_PATH}
)

add_project_dependency(Eigen3 REQUIRED)

if(BUILD_WITH_URDF_SUPPORT)
    add_project_dependency(urdfdom_headers REQUIRED)
    add_project_dependency(urdfdom REQUIRED)
    set(urdfdom_VERSION ${urdfdom_headers_VERSION})

    if(${urdfdom_VERSION} VERSION_GREATER "0.4.2")
        check_minimal_cxx_standard(11 ENFORCE)
        message(
            STATUS
            "Since urdfdom >= 1.0.0, the default C++ standard is C++11. The project is then compiled with C++11 standard."
        )
    endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
    include(${JRL_CMAKE_MODULES}/sdformat.cmake)
    search_for_sdformat(REQUIRED)
    if(SDFormat_FOUND)
        check_minimal_cxx_standard(11 REQUIRED)
        include_directories(${SDFormat_INCLUDE_DIRS})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SDFormat_CXX_FLAGS}")
    endif()
endif()

if(BUILD_WITH_URDF_SUPPORT OR BUILD_WITH_SDF_SUPPORT)
    set(BUILD_WITH_PARSERS_SUPPORT TRUE)
else()
    set(BUILD_WITH_PARSERS_SUPPORT FALSE)
endif()

# -- tracy (optional)
if(PINOCCHIO_BUILD_WITH_TRACY)
    # assume it is installed somewhere
    add_project_dependency(Tracy REQUIRED)
    set_target_properties(
        Tracy::TracyClient
        PROPERTIES POSITION_INDEPENDENT_CODE True
    )
    if(${Tracy_FOUND})
        message(STATUS "Tracy found on your system at ${Tracy_DIR}")
    else()
        message(
            FATAL_ERROR
            "Pinocchio support for tracy is enabled, but tracy was not found on your system."
            " Install it, or set the option PINOCCHIO_DOWNLOAD_TRACY to ON so we can fetch it."
        )
    endif()
endif(PINOCCHIO_BUILD_WITH_TRACY)

if(BUILD_WITH_AUTODIFF_SUPPORT)
    # Check first CppADCodeGen
    if(BUILD_WITH_CODEGEN_SUPPORT)
        add_project_dependency(cppadcg 2.4.1 REQUIRED)
    endif(BUILD_WITH_CODEGEN_SUPPORT)

    add_project_dependency(
      cppad 20180000.0 REQUIRED
      FIND_EXTERNAL "CppAD"
    )
endif(BUILD_WITH_AUTODIFF_SUPPORT)

if(BUILD_WITH_CASADI_SUPPORT)
    add_project_dependency(casadi 3.4.5 REQUIRED)

    # Normalize CasADi link target across versions. Prefer imported target (casadi::casadi), then
    # legacy target (casadi)
    if(NOT TARGET casadi::casadi AND TARGET casadi)
        add_library(casadi::casadi ALIAS casadi)
    endif()
endif(BUILD_WITH_CASADI_SUPPORT)

if(BUILD_WITH_OPENMP_SUPPORT)
    add_project_dependency(OpenMP REQUIRED)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
    add_project_dependency(Qhull COMPONENTS qhullcpp qhull_r REQUIRED)
    message(STATUS "Found Qhull.")
endif()

set_boost_default_options()
export_boost_default_options()
add_project_dependency(Boost REQUIRED COMPONENTS filesystem serialization)

if(Boost_VERSION_STRING VERSION_LESS 1.81)
    if(BUILD_WITH_URDF_SUPPORT AND "${urdfdom_VERSION}" VERSION_GREATER "0.4.2")
        check_minimal_cxx_standard(11 ENFORCE)
        message(
            STATUS
            "Since urdfdom >= 1.0.0, the default C++ standard is C++11. The project is then compiled with C++11 standard."
        )
    endif()
else()
    # Boost.Math will be C++14 starting in July 2023 (Boost 1.82 release)
    check_minimal_cxx_standard(14 ENFORCE)
endif()

if(BUILD_PYTHON_INTERFACE)
    message(
        STATUS
        "The Python bindings of Pinocchio will be compiled along the main library. If you want to disable this feature, please set the option BUILD_PYTHON_INTERFACE to OFF."
    )

    set(PYTHON_COMPONENTS Interpreter Development NumPy)
    findpython(REQUIRED)
    if(BINDINGS_FRAMEWORK STREQUAL Boost)
        add_project_dependency(eigenpy 2.7.10 REQUIRED)
    endif()
    if(BUILD_STANDALONE_PYTHON_INTERFACE)
        add_project_dependency(${PROJECT_NAME} REQUIRED CONFIG)
    endif()

    if(BUILD_WITH_URDF_SUPPORT)
        # console_bridge is urdfdom dependency.
        #
        # We bind some enum of this library and then must link against it.
        add_project_dependency(console_bridge REQUIRED)
    endif()
else()
    message(
        STATUS
        "Pinocchio won't be compiled with its Python bindings. If you want to enable this feature, please set the option BUILD_PYTHON_INTERFACE to ON."
    )
endif()

if(BUILD_WITH_COLLISION_SUPPORT)
    add_project_dependency(coal REQUIRED)
endif()

if(BUILD_WITH_ACCELERATE_SUPPORT)
    if(NOT ${Eigen3_VERSION} VERSION_GREATER_EQUAL "3.4.90")
        message(
            FATAL_ERROR
            "Your version of Eigen is too low. Should be at least 3.4.90. Current version is ${Eigen3_VERSION}."
        )
    endif()

    set(CMAKE_MODULE_PATH
        ${JRL_CMAKE_MODULES}/find-external/Accelerate
        ${CMAKE_MODULE_PATH}
    )
    # FIND_EXTERNAL "Accelerate".
    #
    # We don't export yet as there might be an issue on AMR64 platforms.
    find_package(Accelerate REQUIRED)
    message(STATUS "Build with Accelerate support framework.")
    add_definitions(-DPINOCCHIO_WITH_ACCELERATE_SUPPORT)
endif()

if(
    PINOCCHIO_BUILD_MPFR_TESTING
    OR BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT
)
    message(STATUS "Build with MPFR support")
    set(CMAKE_MODULE_PATH
        "${JRL_CMAKE_MODULES}/find-external/GMP"
        "${JRL_CMAKE_MODULES}/find-external/MPFR"
        ${CMAKE_MODULE_PATH}
    )
    find_package(GMP REQUIRED 6.0.0)
    find_package(MPFR REQUIRED 4.0.0)
endif()

# Sources definition
include(sources.cmake)

# Dependency free headers
list(
    APPEND ${PROJECT_NAME}_CORE_HEADERS
    ${${PROJECT_NAME}_CORE_PUBLIC_HEADERS}
    ${${PROJECT_NAME}_CORE_PRIVATE_HEADERS}
    ${${PROJECT_NAME}_CORE_GENERATED_PUBLIC_HEADERS}
)

# Casadi, cppad and cppadcg are still installed.
# User should link by himself with the right library to include them.
list(
    APPEND ${PROJECT_NAME}_CORE_HEADERS
    ${${PROJECT_NAME}_CASADI_PUBLIC_HEADERS}
    ${${PROJECT_NAME}_CASADI_PRIVATE_HEADERS}
    ${${PROJECT_NAME}_CPPAD_PUBLIC_HEADERS}
    ${${PROJECT_NAME}_CPPAD_PRIVATE_HEADERS}
    ${${PROJECT_NAME}_CPPADCG_PUBLIC_HEADERS}
    ${${PROJECT_NAME}_CPPADCG_PRIVATE_HEADERS}
)

# Header only part + deprecated
if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
    list(
        APPEND ${PROJECT_NAME}_CORE_HEADERS
        ${${PROJECT_NAME}_CORE_DEPRECATED_HEADERS}
    )
    list(
        APPEND ${PROJECT_NAME}_CORE_HEADERS
        ${${PROJECT_NAME}_CPPADCG_DEPRECATED_HEADERS}
    )
endif()

# Template instantiation sources
if(ENABLE_TEMPLATE_INSTANTIATION)
    list(
        APPEND ${PROJECT_NAME}_CORE_SOURCES
        ${${PROJECT_NAME}_TEMPLATE_INSTANTIATION_SOURCES}
    )
endif()

# OpenMP sources
if(BUILD_WITH_OPENMP_SUPPORT)
    list(
        APPEND ${PROJECT_NAME}_PARALLEL_HEADERS
        ${${PROJECT_NAME}_PARALLEL_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_PARALLEL_PRIVATE_HEADERS}
    )

    # OpenMP + deprecated
    if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
        list(
            APPEND ${PROJECT_NAME}_PARALLEL_HEADERS
            ${${PROJECT_NAME}_PARALLEL_DEPRECATED_HEADERS}
        )
    endif()

    # Bindings
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
        ${${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_SOURCES}
    )
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
        ${${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_PUBLIC_HEADERS}
    )
endif()

# Collision sources
if(BUILD_WITH_COLLISION_SUPPORT)
    list(
        APPEND ${PROJECT_NAME}_COLLISION_HEADERS
        ${${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_COLLISION_PRIVATE_HEADERS}
    )

    if(ENABLE_TEMPLATE_INSTANTIATION)
        list(
            APPEND ${PROJECT_NAME}_COLLISION_SOURCES
            ${${PROJECT_NAME}_COLLISION_TEMPLATE_INSTANTIATION_SOURCES}
        )
    endif()

    # Collision + OpenMP
    if(BUILD_WITH_OPENMP_SUPPORT)
        list(
            APPEND ${PROJECT_NAME}_COLLISION_HEADERS
            ${${PROJECT_NAME}_COLLISION_PARALLEL_PUBLIC_HEADERS}
            ${${PROJECT_NAME}_COLLISION_PARALLEL_PRIVATE_HEADERS}
        )
    endif()

    # Collision + Deprecated
    if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
        list(
            APPEND ${PROJECT_NAME}_COLLISION_HEADERS
            ${${PROJECT_NAME}_COLLISION_DEPRECATED_HEADERS}
        )
    endif()

    # Bindings
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
        ${${PROJECT_NAME}_BINDINGS_GRAPH_GEOMETRY_SOURCES}
    )
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
        ${${PROJECT_NAME}_BINDINGS_PYTHON_COLLISION_SOURCES}
    )
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
        ${${PROJECT_NAME}_BINDINGS_PYTHON_COLLISION_PUBLIC_HEADERS}
    )
    if(BUILD_WITH_OPENMP_SUPPORT)
        list(
            APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
            ${${PROJECT_NAME}_BINDINGS_PYTHON_COLLISION_PARALLEL_SOURCES}
        )
        list(
            APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
            ${${PROJECT_NAME}_BINDINGS_PYTHON_COLLISION_PARALLEL_PUBLIC_HEADERS}
        )
    endif()
endif()

# Dependencies free parsers
if(BUILD_WITH_PARSERS_SUPPORT)
    list(
        APPEND ${PROJECT_NAME}_PARSERS_HEADERS
        ${${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_PARSERS_PRIVATE_HEADERS}
    )

    if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
        list(
            APPEND ${PROJECT_NAME}_PARSERS_HEADERS
            ${${PROJECT_NAME}_PARSERS_DEPRECATED_HEADERS}
        )
    endif()

    # Collision + parsers sources
    if(BUILD_WITH_COLLISION_SUPPORT)
        list(
            APPEND ${PROJECT_NAME}_PARSERS_SOURCES
            ${${PROJECT_NAME}_GEOMETRY_SOURCES}
        )
    endif()
endif()

# URDF parser sources
if(BUILD_WITH_URDF_SUPPORT)
    list(APPEND ${PROJECT_NAME}_PARSERS_SOURCES ${${PROJECT_NAME}_URDF_SOURCES})
    list(
        APPEND ${PROJECT_NAME}_PARSERS_HEADERS
        ${${PROJECT_NAME}_URDF_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_URDF_PRIVATE_HEADERS}
    )

    # URDF parsers + deprecated
    if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
        list(
            APPEND ${PROJECT_NAME}_PARSERS_HEADERS
            ${${PROJECT_NAME}_URDF_DEPRECATED_HEADERS}
        )
    endif()
endif()

# SDF parser sources
if(BUILD_WITH_SDF_SUPPORT)
    list(APPEND ${PROJECT_NAME}_PARSERS_SOURCES ${${PROJECT_NAME}_SDF_SOURCES})
    list(
        APPEND ${PROJECT_NAME}_PARSERS_HEADERS
        ${${PROJECT_NAME}_SDF_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_SDF_PRIVATE_HEADERS}
    )

    # SDF parsers + deprecated
    if(PINOCCHIO_INSTALL_DEPRECATED_HEADERS)
        list(
            APPEND ${PROJECT_NAME}_PARSERS_HEADERS
            ${${PROJECT_NAME}_SDF_DEPRECATED_HEADERS}
        )
    endif()
endif()

# Python parser sources
if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
    list(
        APPEND ${PROJECT_NAME}_PYTHON_PARSER_HEADERS
        ${${PROJECT_NAME}_PYTHON_PARSER_PUBLIC_HEADERS}
    )
endif()

# Extra sources
if(BUILD_WITH_EXTRA_SUPPORT)
    list(APPEND ${PROJECT_NAME}_EXTRA_SOURCES ${${PROJECT_NAME}_EXTRA_SOURCES})
    list(
        APPEND ${PROJECT_NAME}_EXTRA_HEADERS
        ${${PROJECT_NAME}_EXTRA_PUBLIC_HEADERS}
        ${${PROJECT_NAME}_EXTRA_PRIVATE_HEADERS}
    )
endif()

list(
    APPEND ${PROJECT_NAME}_VISUALIZERS_HEADERS
    ${${PROJECT_NAME}_VISUALIZERS_PUBLIC_HEADERS}
    ${${PROJECT_NAME}_VISUALIZERS_PRIVATE_HEADERS}
)

# List headers to install
list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_CORE_HEADERS})

if(BUILD_WITH_OPENMP_SUPPORT)
    list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_PARALLEL_HEADERS})
endif()

if(BUILD_WITH_COLLISION_SUPPORT)
    list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_COLLISION_HEADERS})
endif()

if(BUILD_WITH_PARSERS_SUPPORT)
    list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_PARSERS_HEADERS})
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
    list(
        APPEND ${PROJECT_NAME}_HEADERS
        ${${PROJECT_NAME}_PYTHON_PARSER_HEADERS}
    )
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
    list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_EXTRA_HEADERS})
    list(
        APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
        ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_SOURCES}
    )
endif()

list(
    APPEND ${PROJECT_NAME}_HEADERS
    ${${PROJECT_NAME}_VISUALIZERS_PUBLIC_HEADERS}
)

list(
    APPEND ${PROJECT_NAME}_HEADERS
    ${${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS}
)

# Define PINOCCHIO_WINDOWS_DLL_PATH environment variable on Windows
function(ADD_WINDOWS_DLL_PATH_TO_TEST TEST_NAME)
    if(WIN32)
        get_test_property(${TEST_NAME} ENVIRONMENT ENV_VARIABLES)
        # TODO don't do this in the future
        list(
            APPEND ENV_VARIABLES
            "PINOCCHIO_WINDOWS_DLL_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}::${PROJECT_NAME}_default>"
        )
        set_tests_properties(
            ${TEST_NAME}
            PROPERTIES ENVIRONMENT "${ENV_VARIABLES}"
        )
    endif()
endfunction()

# --- MAIN LIBRARY -------------------------------------------------------------
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
    add_subdirectory(src)
endif()

# --- BINDINGS -----------------------------------------------------------------
if(BUILD_PYTHON_INTERFACE)
    add_subdirectory(bindings)
endif()

# --- EXECUTABLES --------------------------------------------------------------
add_subdirectory(utils)

# --- UNIT TESTS ---------------------------------------------------------------
add_subdirectory(unittest)

# --- CHECK EXAMPLES -----------------------------------------------------------
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# --- BENCHMARKS ---------------------------------------------------------------
if(BUILD_BENCHMARK AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
    add_subdirectory(benchmark)
endif()

# --- PACKAGING ----------------------------------------------------------------
# Export variables in CMake module and in parent scope for workspace compatibility
# PARENT_SCOPE_ONLY: If set: only export in parent scope
# CONDITION: Condition to setup the variable
# VAR_NAME: Variable name
# VAR_VALUE: Varible value
macro(EXPORT_VARIABLE)
    set(options PARENT_SCOPE_ONLY)
    set(oneValueArgs CONDITION VAR_NAME VAR_VALUE)
    set(multiValueArgs)
    cmake_parse_arguments(
        ARGS
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )

    if(${ARGS_CONDITION})
        get_directory_property(has_parent PARENT_DIRECTORY)
        if(has_parent)
            set(${ARGS_VAR_NAME} ${ARGS_VAR_VALUE} PARENT_SCOPE)
        else()
            set(${ARGS_VAR_NAME} ${ARGS_VAR_VALUE})
        endif()
        if(NOT ${ARGS_PARENT_SCOPE_ONLY})
            string(
                APPEND PACKAGE_EXTRA_MACROS
                "\nset(${ARGS_VAR_NAME} ${ARGS_VAR_VALUE})"
            )
        endif()
    endif()
endmacro()

export_variable(CONDITION PINOCCHIO_BUILD_WITH_TRACY
                VAR_NAME PINOCCHIO_USE_TRACY
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_URDF_SUPPORT
                VAR_NAME PINOCCHIO_USE_URDFDOM
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_COLLISION_SUPPORT
                VAR_NAME PINOCCHIO_USE_COAL
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_CPPAD_SUPPORT
                VAR_NAME PINOCCHIO_USE_CPPAD
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_CPPAD_CODEGEN_SUPPORT
                VAR_NAME PINOCCHIO_USE_CPPAD_CODEGEN
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_CASADI_SUPPORT
                VAR_NAME PINOCCHIO_USE_CASADI
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_PYTHON_INTERFACE
                VAR_NAME PINOCCHIO_WITH_PYTHON_INTERFACE
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_EXTRA_SUPPORT
                VAR_NAME PINOCCHIO_USE_EXTRA
                VAR_VALUE ON
)
export_variable(CONDITION BUILD_WITH_PYTHON_PARSER_SUPPORT
                VAR_NAME PINOCCHIO_USE_PYTHON_PARSER
                VAR_VALUE ON
)
export_variable(CONDITION TRUE
                VAR_NAME pinocchio_VERSION
                VAR_VALUE ${pinocchio_VERSION}
                PARENT_SCOPE_ONLY
)

setup_project_finalize()
