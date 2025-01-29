#
# Copyright (c) 2015-2023 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME pinocchio)
set(PROJECT_DESCRIPTION
    "A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives"
)
set(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
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
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare("jrl-cmakemodules"
                         GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
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

include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")
if(APPLE) # Use the handmade approach
  if(${CMAKE_VERSION} VERSION_LESS "3.18.0") # Need to find the right version
    set(CMAKE_MODULE_PATH ${JRL_CMAKE_MODULES}/find-external/OpenMP ${CMAKE_MODULE_PATH})
  endif()
elseif(UNIX)
  if(${CMAKE_VERSION} VERSION_EQUAL "3.20.0")
    set(CMAKE_MODULE_PATH ${JRL_CMAKE_MODULES}/find-external/OpenMP ${CMAKE_MODULE_PATH})
  endif()
endif()
include(CMakeDependentOption)

# If needed, set CMake policy for APPLE systems
apply_default_apple_configuration()
if(CMAKE_VERSION VERSION_GREATER "3.12")
  cmake_policy(SET CMP0074 NEW)
endif()

# Force C++ standard to be C++11 at least
check_minimal_cxx_standard(11 ENFORCE)

# --- OPTIONS ----------------------------------------
option(BUILD_BENCHMARK "Build the benchmarks" OFF)
option(BUILD_UTILS "Build the utils" OFF)
option(BUILD_PYTHON_INTERFACE "Build the Python bindings" ON)
option(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" OFF)
option(BUILD_WITH_COMMIT_VERSION "Build libraries by setting specific commit version" OFF)

if(DEFINED BUILD_UNIT_TESTS)
  message(
    AUTHOR_WARNING
      "BUILD_UNIT_TESTS is deprecated. Use BUILD_TESTING instead.\
    If you are manually building Pinocchio from source in an existing build folder,\
    we suggest that you delete your build folder and make a new one.")
  set(BUILD_TESTING ${BUILD_UNIT_TESTS})
endif(DEFINED BUILD_UNIT_TESTS)

option(BUILD_ADVANCED_TESTING "Build the advanced tests (multiprecision, etc.) of Pinocchio" OFF)

# --- OPTIONAL DEPENDENCIES -------------------------
option(BUILD_WITH_URDF_SUPPORT "Build the library with the URDF format support" ON)
option(BUILD_WITH_SDF_SUPPORT "Build the library with the SDF format support" OFF)
option(BUILD_WITH_COLLISION_SUPPORT
       "Build the library with the collision support (required coal)" OFF)
option(BUILD_WITH_AUTODIFF_SUPPORT
       "Build the library with the automatic differentiation support (via CppAD)" OFF)
option(BUILD_WITH_CASADI_SUPPORT "Build the library with the support of CASADI" OFF)
# cppadcodgen don't build well with CL
if(NOT WIN32)
  option(BUILD_WITH_CODEGEN_SUPPORT
         "Build the library with the support of code generation (via CppADCodeGen)" OFF)
endif()
option(BUILD_WITH_OPENMP_SUPPORT "Build the library with the OpenMP support" OFF)
option(BUILD_WITH_EXTRA_SUPPORT "Build the library with extra algorithms support" OFF)
cmake_dependent_option(
  BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT
  "Build the Python interface with Boost.Multiprecision MPFR support" OFF BUILD_PYTHON_INTERFACE
  OFF)
cmake_dependent_option(BUILD_WITH_LIBPYTHON "Build the library with Python format support" ON
                       "BUILD_PYTHON_INTERFACE" OFF)
if(APPLE)
  option(BUILD_WITH_ACCELERATE_SUPPORT "Build Pinocchio with the Accelerate support" OFF)
else(APPLE)
  set(BUILD_WITH_ACCELERATE_SUPPORT FALSE)
endif(APPLE)
option(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" OFF)

option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
option(CHECK_RUNTIME_MALLOC "Check if some memory allocations are performed at runtime" OFF)

option(ENABLE_TEMPLATE_INSTANTIATION "Template instantiation of the main library" ON)

# Variable containing all the cflags definition, options and libraries to setup pkg-config.
set(CFLAGS_OPTIONS)
set(CFLAGS_DEPENDENCIES)
set(LIBRARIES_DEPENDENCIES)

if(BUILD_WITH_COLLISION_SUPPORT)
  set(BUILD_WITH_COAL_SUPPORT TRUE)
endif()
if(BUILD_WITH_CODEGEN_SUPPORT)
  set(BUILD_WITH_CPPAD_CODEGEN_SUPPORT TRUE)
  set(BUILD_WITH_AUTODIFF_SUPPORT ON)
endif()
if(BUILD_WITH_AUTODIFF_SUPPORT)
  set(BUILD_WITH_CPPAD_SUPPORT TRUE)
endif()
if(BUILD_WITH_LIBPYTHON)
  set(BUILD_WITH_PYTHON_PARSER_SUPPORT TRUE)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
  list(APPEND CFLAGS_OPTIONS "-DPINOCCHIO_WITH_EXTRA_SUPPORT")
endif()

if(INITIALIZE_WITH_NAN)
  message(STATUS "Initialize with NaN all the Eigen entries.")
endif(INITIALIZE_WITH_NAN)

if(CHECK_RUNTIME_MALLOC)
  message(STATUS "Check if some memory allocations are performed at runtime.")
endif(CHECK_RUNTIME_MALLOC)

if(ENABLE_TEMPLATE_INSTANTIATION)
  message(STATUS "Template instantiation of the main library")
  list(APPEND CFLAGS_OPTIONS "-DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION")
  list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_default)
endif(ENABLE_TEMPLATE_INSTANTIATION)

macro(TAG_LIBRARY_VERSION target)
  set_target_properties(${target} PROPERTIES SOVERSION ${PROJECT_VERSION})
endmacro(TAG_LIBRARY_VERSION)

set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/models")

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
set(CMAKE_MODULE_PATH "${JRL_CMAKE_MODULES}/find-external/CppAD/" ${CMAKE_MODULE_PATH})
add_project_dependency(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")

if(BUILD_WITH_URDF_SUPPORT)
  add_project_dependency(urdfdom_headers REQUIRED)
  add_project_dependency(urdfdom REQUIRED PKG_CONFIG_REQUIRES "urdfdom >= 0.2.0")
  set(urdfdom_VERSION ${urdfdom_headers_VERSION})
  list(APPEND CFLAGS_DEPENDENCIES "-DPINOCCHIO_WITH_URDFDOM")

  if(${urdfdom_VERSION} VERSION_GREATER "0.4.2")
    check_minimal_cxx_standard(11 ENFORCE)
    message(
      STATUS
        "Since urdfdom >= 1.0.0, the default C++ standard is C++11. The project is then compiled with C++11 standard."
    )
  endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
  include(${CMAKE_CURRENT_LIST_DIR}/cmake/sdformat.cmake)
  search_for_sdformat(REQUIRED)
  if(SDFormat_FOUND)
    check_minimal_cxx_standard(11 REQUIRED)
    include_directories(${SDFormat_INCLUDE_DIRS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SDFormat_CXX_FLAGS}")
    list(APPEND CFLAGS_DEPENDENCIES "-DPINOCCHIO_WITH_SDFORMAT")
  endif()
endif()

set(BUILD_WITH_PARSERS_SUPPORT BUILD_WITH_URDF_SUPPORT OR BUILD_WITH_SDF_SUPPORT)

if(BUILD_WITH_PARSERS_SUPPORT)
  list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_parsers)
endif()

if(BUILD_WITH_AUTODIFF_SUPPORT)
  # Check first CppADCodeGen
  if(BUILD_WITH_CODEGEN_SUPPORT)
    # No need to add cppadcg to pkg-config (no lib and header in the same directory than cppad)
    add_project_dependency(cppadcg 2.4.1 REQUIRED)
    list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_cppadcg)
  endif(BUILD_WITH_CODEGEN_SUPPORT)

  add_project_dependency(
    cppad 20180000.0 REQUIRED
    PKG_CONFIG_REQUIRES "cppad >= 20220624.0"
    FIND_EXTERNAL "CppAD")
  list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_cppad)
endif(BUILD_WITH_AUTODIFF_SUPPORT)

if(BUILD_WITH_CASADI_SUPPORT)
  add_project_dependency(casadi 3.4.5 REQUIRED PKG_CONFIG_REQUIRES "casadi >= 3.4.5")
  list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_casadi)
endif(BUILD_WITH_CASADI_SUPPORT)

if(BUILD_WITH_OPENMP_SUPPORT)
  add_project_dependency(OpenMP REQUIRED)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
  list(APPEND LIBRARIES_DEPENDENCIES ${PROJECT_NAME}_extra qhullcpp qhull_r)
  add_project_dependency(Qhull COMPONENTS qhullcpp qhull_r REQUIRED)
  message(STATUS "Found Qhull.")
endif()

set(BOOST_REQUIRED_COMPONENTS filesystem serialization system)

set_boost_default_options()
export_boost_default_options()
add_project_dependency(Boost REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

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
  add_project_dependency(eigenpy 2.7.10 REQUIRED)

  if(BUILD_WITH_URDF_SUPPORT)
    # console_bridge is urdfdom dependency.
    #
    # We bind some enum of this library and then must link against it.
    add_project_dependency(console_bridge REQUIRED)
  endif()

  # Check wether this a PyPy Python
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import platform; print(platform.python_implementation())"
    OUTPUT_VARIABLE _python_implementation_value
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

  message(STATUS "Python compiler: ${_python_implementation_value}")
  if(_python_implementation_value MATCHES "PyPy")
    set(BUILD_WITH_PYTHON_PARSER_SUPPORT OFF)
    message(
      STATUS
        "PyPy detected, therefore libpython is not available and BUILD_WITH_LIBPYTHON set to OFF.")
  endif()
else()
  message(
    STATUS
      "Pinocchio won't be compiled with its Python bindings. If you want to enable this feature, please set the option BUILD_PYTHON_INTERFACE to ON."
  )
endif()

if(BUILD_WITH_COAL_SUPPORT)
  add_project_dependency(coal 3.0.0 REQUIRED)
endif()

if(BUILD_WITH_ACCELERATE_SUPPORT)
  if(NOT ${Eigen3_VERSION} VERSION_GREATER_EQUAL "3.4.90")
    message(
      FATAL_ERROR
        "Your version of Eigen is too low. Should be at least 3.4.90. Current version is ${Eigen3_VERSION}."
    )
  endif()

  set(CMAKE_MODULE_PATH ${JRL_CMAKE_MODULES}/find-external/Accelerate ${CMAKE_MODULE_PATH})
  # FIND_EXTERNAL "Accelerate".
  #
  # We don't export yet as there might be an issue on AMR64 platforms.
  find_package(Accelerate REQUIRED)
  message(STATUS "Build with Accelerate support framework.")
  add_definitions(-DPINOCCHIO_WITH_ACCELERATE_SUPPORT)
endif(BUILD_WITH_ACCELERATE_SUPPORT)

# Sources definition
include(sources.cmake)

# Template instantiation sources
if(ENABLE_TEMPLATE_INSTANTIATION)
  list(APPEND ${PROJECT_NAME}_CORE_SOURCES ${${PROJECT_NAME}_TEMPLATE_INSTANTIATION_SOURCES})
  list(APPEND ${PROJECT_NAME}_CORE_PUBLIC_HEADERS
       ${${PROJECT_NAME}_TEMPLATE_INSTANTIATION_PUBLIC_HEADERS})
endif()

# URDF sources
if(BUILD_WITH_URDF_SUPPORT)
  list(APPEND ${PROJECT_NAME}_PARSERS_SOURCES ${${PROJECT_NAME}_URDF_SOURCES})
  list(APPEND ${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS ${${PROJECT_NAME}_URDF_PUBLIC_HEADERS})
endif(BUILD_WITH_URDF_SUPPORT)

# SDF sources
if(BUILD_WITH_SDF_SUPPORT)
  list(APPEND ${PROJECT_NAME}_PARSERS_SOURCES ${${PROJECT_NAME}_SDF_SOURCES})
  list(APPEND ${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS ${${PROJECT_NAME}_SDF_PUBLIC_HEADERS})
endif(BUILD_WITH_SDF_SUPPORT)

# Extra sources
if(BUILD_WITH_EXTRA_SUPPORT)
  list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
       ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_SOURCES})
endif()

# COAL sources
if(BUILD_WITH_COAL_SUPPORT)
  if(ENABLE_TEMPLATE_INSTANTIATION)
    list(APPEND ${PROJECT_NAME}_COLLISION_SOURCES
         ${${PROJECT_NAME}_COLLISION_TEMPLATE_INSTANTIATION_SOURCES})
    list(APPEND ${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS
         ${${PROJECT_NAME}_COLLISION_TEMPLATE_INSTANTIATION_PUBLIC_HEADERS})
  endif()
  list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
      ${${PROJECT_NAME}_BINDINGS_PYTHON_COAL_SOURCES})
  list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
      ${${PROJECT_NAME}_BINDINGS_PYTHON_COAL_PUBLIC_HEADERS})
  if(BUILD_WITH_OPENMP_SUPPORT)
    list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
        ${${PROJECT_NAME}_BINDINGS_PYTHON_COAL_PARALLEL_SOURCES})
    list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
        ${${PROJECT_NAME}_BINDINGS_PYTHON_COAL_PARALLEL_PUBLIC_HEADERS})
  endif()
endif(BUILD_WITH_COAL_SUPPORT)

# OpenMP sources
if(BUILD_WITH_OPENMP_SUPPORT)
  list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
       ${${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_SOURCES})
  list(APPEND ${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
       ${${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_PUBLIC_HEADERS})
endif()

# List headers to install
list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_CORE_PUBLIC_HEADERS})

if(BUILD_WITH_EXTRA_SUPPORT)
  list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_EXTRA_PUBLIC_HEADERS})
endif()

if(BUILD_WITH_PARSERS_SUPPORT)
  list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS})
endif()

if(BUILD_WITH_COAL_SUPPORT)
  list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS})
  if(BUILD_WITH_OPENMP_SUPPORT)
    list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_COLLISION_PARALLEL_PUBLIC_HEADERS})
  endif()
endif()

if(BUILD_WITH_OPENMP_SUPPORT)
  list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_PARALLEL_PUBLIC_HEADERS})
endif()

if(BUILD_PYTHON_INTERFACE)
  list(APPEND ${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS})
endif()

# Define PINOCCHIO_WINDOWS_DLL_PATH environment variable on Windows
function(ADD_WINDOWS_DLL_PATH_TO_TEST TEST_NAME)
  if(WIN32)
    get_test_property(${TEST_NAME} ENVIRONMENT ENV_VARIABLES)
    # TODO don't do this in the future
    list(APPEND ENV_VARIABLES
         "PINOCCHIO_WINDOWS_DLL_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}_default>")
    set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
  endif()
endfunction()

# --- MAIN LIBRARY -------------------------------------------------------------
add_subdirectory(src)

# --- BINDINGS -----------------------------------------------------------------
add_subdirectory(bindings)

# --- EXECUTABLES --------------------------------------------------------------
add_subdirectory(utils)

# --- UNIT TESTS ---------------------------------------------------------------
add_subdirectory(unittest)

# --- CHECK EXAMPLES -----------------------------------------------------------
add_subdirectory(examples)

# --- BENCHMARKS ---------------------------------------------------------------
add_subdirectory(benchmark)

# --- PACKAGING ----------------------------------------------------------------
macro(EXPORT_VARIABLE var_name var_value)
  get_directory_property(has_parent PARENT_DIRECTORY)
  if(has_parent)
    set(${var_name}
        ${var_value}
        PARENT_SCOPE)
  else()
    set(${var_name} ${var_value})
  endif()
endmacro(
  EXPORT_VARIABLE
  var_name
  var_value)

if(BUILD_WITH_URDF_SUPPORT)
  export_variable(PINOCCHIO_USE_URDFDOM ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_URDFDOM \"\")")
endif()
if(BUILD_WITH_COAL_SUPPORT)
    export_variable(PINOCCHIO_USE_COAL ON)
    set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_HPP_FCL \"\")")
    set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_COAL \"\")")
endif()
if(BUILD_WITH_CPPAD_SUPPORT)
  export_variable(PINOCCHIO_USE_CPPAD ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CPPAD \"\")")
endif()
if(BUILD_WITH_CPPAD_CODEGEN_SUPPORT)
  export_variable(PINOCCHIO_USE_CPPAD_CODEGEN ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CPPAD_CODEGEN \"\")")
endif()
if(BUILD_WITH_CASADI_SUPPORT)
  export_variable(PINOCCHIO_USE_CASADI ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CASADI \"\")")
endif()
if(BUILD_PYTHON_INTERFACE)
  export_variable(PINOCCHIO_WITH_PYTHON_INTERFACE ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_WITH_PYTHON_INTERFACE \"\")")
endif()
if(BUILD_WITH_EXTRA_SUPPORT)
  export_variable(PINOCCHIO_USE_EXTRA ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_EXTRA \"\")")
endif()
if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
  export_variable(PINOCCHIO_USE_PYTHON_PARSER ON)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_PYTHON_PARSER \"\")")
endif()
export_variable(pinocchio_VERSION ${pinocchio_VERSION})

# Setup pkg-configs flags and libs
pkg_config_append_libs("${LIBRARIES_DEPENDENCIES}")
foreach(boostlib ${BOOST_REQUIRED_COMPONENTS})
  pkg_config_append_libs("boost_${boostlib}")
endforeach()
pkg_config_append_cflags("${CFLAGS_DEPENDENCIES}")
pkg_config_append_cflags("${CFLAGS_OPTIONS}")

# Install catkin package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

setup_project_finalize()
