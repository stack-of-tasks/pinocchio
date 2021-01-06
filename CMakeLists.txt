#
# Copyright (c) 2015-2020 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

SET(PROJECT_NAME pinocchio)
SET(PROJECT_DESCRIPTION "A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives")
SET(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")
SET(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

# Check if the submodule cmake have been initialized
IF(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/cmake/base.cmake")
  MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
ENDIF()

SET(DOXYGEN_USE_MATHJAX YES)
SET(DOXYGEN_USE_TEMPLATE_CSS YES)

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cmake/base.cmake)
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cmake/boost.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cmake/python.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cmake/ide.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cmake/apple.cmake)

# If needed, set CMake policy for APPLE systems
APPLY_DEFAULT_APPLE_CONFIGURATION()

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

# --- OPTIONS ----------------------------------------
OPTION(BUILD_BENCHMARK "Build the benchmarks" OFF)
OPTION(BUILD_UTILS "Build the utils" OFF)
OPTION(BUILD_PYTHON_INTERFACE "Build the Python bindings" ON)
OPTION(BUILD_WITH_COMMIT_VERSION "Build libraries by setting specific commit version" OFF)

IF(DEFINED BUILD_UNIT_TESTS)
  MESSAGE(AUTHOR_WARNING "BUILD_UNIT_TESTS is deprecated. Use BUILD_TESTING instead.\
    If you are manually building Pinocchio from source in an existing build folder,\
    we suggest that you delete your build folder and make a new one.")
  SET(BUILD_TESTING ${BUILD_UNIT_TESTS})
ENDIF(DEFINED BUILD_UNIT_TESTS)

OPTION(BUILD_ADVANCED_TESTING "Build the advanced tests (multiprecision, etc.) of Pinocchio" OFF)

# --- OPTIONAL DEPENDENCIES -------------------------
OPTION(BUILD_WITH_URDF_SUPPORT "Build the library with the URDF format support" ON)
OPTION(BUILD_WITH_COLLISION_SUPPORT "Build the library with the collision support (required HPP-FCL)" OFF)
OPTION(BUILD_WITH_AUTODIFF_SUPPORT "Build the library with the automatic differentiation support (via CppAD)" OFF)
OPTION(BUILD_WITH_CASADI_SUPPORT "Build the library with the support of CASADI" OFF)
OPTION(BUILD_WITH_CODEGEN_SUPPORT "Build the library with the support of code generation (via CppADCodeGen)" OFF)

OPTION(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)

IF(BUILD_WITH_COLLISION_SUPPORT)
  SET(BUILD_WITH_HPP_FCL_SUPPORT TRUE)
ENDIF()
IF(BUILD_WITH_AUTODIFF_SUPPORT)
  SET(BUILD_WITH_CPPAD_SUPPORT TRUE)
ENDIF()
IF(BUILD_WITH_CODEGEN_SUPPORT)
  SET(BUILD_WITH_CPPAD_CODEGEN_SUPPORT TRUE)
ENDIF()

IF(INITIALIZE_WITH_NAN)
  MESSAGE (STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

MACRO(TAG_LIBRARY_VERSION target)
  SET_TARGET_PROPERTIES(${target} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDMACRO(TAG_LIBRARY_VERSION)

SET(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/models")

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/find-external/CppAD/" ${CMAKE_MODULE_PATH})
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")

# Variable containing all the cflags definition relative to optional dependencies
SET(CFLAGS_DEPENDENCIES)

IF(BUILD_WITH_URDF_SUPPORT)
  ADD_PROJECT_DEPENDENCY(urdfdom_headers)
  ADD_PROJECT_DEPENDENCY(urdfdom PKG_CONFIG_REQUIRES "urdfdom >= 0.2.0")
  SET(urdfdom_VERSION ${urdfdom_headers_VERSION})
  ADD_DEFINITIONS(-DPINOCCHIO_WITH_URDFDOM)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DPINOCCHIO_WITH_URDFDOM")

  IF(${urdfdom_VERSION} VERSION_GREATER "0.4.2")
    SET(CMAKE_CXX_STANDARD 11)
    SET(CMAKE_CXX_STANDARD_REQUIRED ON)
    MESSAGE(STATUS "Since urdfdom >= 1.0.0, the default C++ standard is C++11. The project is then compiled with C++11 standard.")
  ENDIF(${urdfdom_VERSION} VERSION_GREATER "0.4.2")
ENDIF(BUILD_WITH_URDF_SUPPORT)

IF(BUILD_WITH_AUTODIFF_SUPPORT)
  # Check first CppADCodeGen
  IF(BUILD_WITH_CODEGEN_SUPPORT)
    ADD_PROJECT_DEPENDENCY(cppadcg 2.4.1 REQUIRED PKG_CONFIG_REQUIRES "cppadcg >= 2.4.1") # CppADCodeGen 2.4.1 is the first version to check the minimal version of CppAD
  ENDIF(BUILD_WITH_CODEGEN_SUPPORT)

  ADD_PROJECT_DEPENDENCY(cppad 20180000.0 REQUIRED PKG_CONFIG_REQUIRES "cppad >= 20180000.0")
ENDIF(BUILD_WITH_AUTODIFF_SUPPORT)

IF(BUILD_WITH_CASADI_SUPPORT)
  ADD_PROJECT_DEPENDENCY(casadi 3.4.5 REQUIRED PKG_CONFIG_REQUIRES "casadi >= 3.4.5")
ENDIF(BUILD_WITH_CASADI_SUPPORT)

SET(BOOST_REQUIRED_COMPONENTS filesystem serialization system)

SET_BOOST_DEFAULT_OPTIONS()
EXPORT_BOOST_DEFAULT_OPTIONS()
ADD_PROJECT_DEPENDENCY(Boost REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

IF(BUILD_PYTHON_INTERFACE)
  MESSAGE(STATUS "The Python bindings of Pinocchio will be compiled along the main library. If you want to disable this feature, please set the option BUILD_PYTHON_INTERFACE to OFF.")
  FINDPYTHON(REQUIRED)
  SEARCH_FOR_BOOST_PYTHON(REQUIRED)
  ADD_PROJECT_DEPENDENCY(eigenpy 2.5.0 REQUIRED)
ELSE(BUILD_PYTHON_INTERFACE)
  MESSAGE(STATUS "Pinocchio won't be compiled with its Python bindings. If you want to enable this feature, please set the option BUILD_PYTHON_INTERFACE to ON.")
ENDIF(BUILD_PYTHON_INTERFACE)

IF(BUILD_WITH_HPP_FCL_SUPPORT)
  ADD_DEFINITIONS(-DPINOCCHIO_WITH_HPP_FCL)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DPINOCCHIO_WITH_HPP_FCL")
  ADD_PROJECT_DEPENDENCY(hpp-fcl 1.4.0 REQUIRED PKG_CONFIG_REQUIRES "hpp-fcl >= 1.4.0")
  # Check whether hpp-fcl python bindings are available.
  SET(BUILD_WITH_HPP_FCL_PYTHON_BINDINGS FALSE)
  IF(BUILD_PYTHON_INTERFACE)
    EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import hppfcl"
      RESULT_VARIABLE _hpp_fcl_python_bindings_not_found
      OUTPUT_QUIET
      ERROR_QUIET)
    IF(_hpp_fcl_python_bindings_not_found EQUAL 0)
      SET(BUILD_WITH_HPP_FCL_PYTHON_BINDINGS TRUE)
      MESSAGE(STATUS "Found hpp-fcl Python bindings.")
    ELSE()
      MESSAGE(STATUS "hpp-fcl Python bindings NOT found.")
    ENDIF()
    UNSET(_hpp_fcl_python_bindings_not_found)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDIF(BUILD_WITH_HPP_FCL_SUPPORT)

# Enforce the preprocessed version of boost::list and boost::vector
# This information is redundant with the content of include/pinocchio/container/boost-container-limits.hpp
# but it avoids any compilation issue.
ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_LIST_SIZE=30)
ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_VECTOR_SIZE=30)

LIST(APPEND CFLAGS_DEPENDENCIES "-DBOOST_MPL_LIMIT_LIST_SIZE=30")

# Path to boost headers
INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIRS})

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
FILE(GLOB_RECURSE HEADERS 
     ${PROJECT_SOURCE_DIR}/src/*.hpp 
     ${PROJECT_SOURCE_DIR}/src/*.hxx 
     ) 

IF(NOT BUILD_WITH_URDF_SUPPORT)
  LIST(REMOVE_ITEM HEADERS
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf.hpp
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/model.hxx
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/geometry.hxx
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/utils.hpp
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/types.hpp
    )
ENDIF(NOT BUILD_WITH_URDF_SUPPORT)

IF(NOT BUILD_WITH_HPP_FCL_SUPPORT)
  LIST(REMOVE_ITEM HEADERS 
    ${PROJECT_SOURCE_DIR}/src/spatial/fcl-pinocchio-conversions.hpp
    )
ENDIF(NOT BUILD_WITH_HPP_FCL_SUPPORT)

LIST(APPEND HEADERS macros.hpp)

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/core")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/math")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/joint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/liegroup")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/visitor")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/urdf")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/utils")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/serialization")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/algorithm")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/container")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/codegen")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/casadi")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/casadi/math")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/casadi/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/casadi/utils")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/cppad")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/cppad/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/cppad/math")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/cppad/algorithm")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/autodiff/cppad/utils")

SET(HEADERS_)
FOREACH(header ${HEADERS})
  STRING(REGEX REPLACE "${PROJECT_SOURCE_DIR}/src/" "" header ${header})
  LIST(APPEND HEADERS_ ${header})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
    ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${header})
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header)
SET(HEADERS ${HEADERS_})

LIST(APPEND HEADERS
     ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/config.hpp
     ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/deprecated.hpp
     ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/warning.hpp)

# --- MAIN LIBRARY -------------------------------------------------------------
ADD_SUBDIRECTORY(src)

# --- BINDINGS -----------------------------------------------------------------
ADD_SUBDIRECTORY(bindings)

# --- EXECUTABLES --------------------------------------------------------------
ADD_SUBDIRECTORY(utils)

# --- UNIT TESTS ---------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)

# --- CHECK EXAMPLES -----------------------------------------------------------
ADD_SUBDIRECTORY(examples)

# --- BENCHMARKS ---------------------------------------------------------------
ADD_SUBDIRECTORY(benchmark)

# --- PACKAGING ----------------------------------------------------------------
MACRO(EXPORT_VARIABLE var_name var_value)
  GET_DIRECTORY_PROPERTY(has_parent PARENT_DIRECTORY)
  IF(has_parent)
    SET(${var_name} ${var_value} PARENT_SCOPE)
  ELSE()
    SET(${var_name} ${var_value})
  ENDIF()
ENDMACRO(EXPORT_VARIABLE var_name var_value)

IF(BUILD_WITH_URDF_SUPPORT)
  EXPORT_VARIABLE(PINOCCHIO_USE_URDFDOM ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_URDFDOM \"\")")
ENDIF()
IF(BUILD_WITH_HPP_FCL_SUPPORT)
  EXPORT_VARIABLE(PINOCCHIO_USE_HPP_FCL ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_HPP_FCL \"\")")
ENDIF()
IF(BUILD_WITH_CPPAD_SUPPORT)
  EXPORT_VARIABLE(PINOCCHIO_USE_CPPAD ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CPPAD \"\")")
ENDIF()
IF(BUILD_WITH_CPPAD_CODEGEN_SUPPORT)
  EXPORT_VARIABLE(PINOCCHIO_USE_CPPAD_CODEGEN ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CPPAD_CODEGEN \"\")")
ENDIF()
IF(BUILD_WITH_CASADI_SUPPORT)
  EXPORT_VARIABLE(PINOCCHIO_USE_CASADI ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_USE_CASADI \"\")")
ENDIF()
IF(BUILD_PYTHON_INTERFACE)
  EXPORT_VARIABLE(PINOCCHIO_WITH_PYTHON_INTERFACE ON)
  SET(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}\nset(PINOCCHIO_WITH_PYTHON_INTERFACE \"\")")
ENDIF()

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
PKG_CONFIG_APPEND_BOOST_LIBS(${BOOST_REQUIRED_COMPONENTS})

FOREACH(cflags ${CFLAGS_DEPENDENCIES})
  PKG_CONFIG_APPEND_CFLAGS(${cflags})
ENDFOREACH(cflags ${CFLAGS_DEPENDENCIES})

# Install catkin package.xml
INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
