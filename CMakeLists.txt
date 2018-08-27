#
# Copyright (c) 2015-2018 CNRS
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#
# This file is part of Pinocchio
# pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# pinocchio If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME pinocchio)
SET(PROJECT_DESCRIPTION "Rigid multi body dynamics algorithms")
SET(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" ON)

SET(DOXYGEN_USE_MATHJAX YES)

IF(APPLE)
  SET(CMAKE_MACOSX_RPATH TRUE)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
     set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif("${isSystemDir}" STREQUAL "-1")
ENDIF(APPLE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

SETUP_PROJECT()

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

# --- OPTIONS ----------------------------------------
OPTION (BUILD_BENCHMARK "Build the benchmarks" OFF)
OPTION (BUILD_UNIT_TESTS "Build the unitary tests" ON)
OPTION (BUILD_UTILS "Build the utils" OFF)
OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION (BUILD_WITH_LUA_SUPPORT "Build the lua parser" OFF)
OPTION (BUILD_WITH_COMMIT_VERSION "Build libraries by setting specific commit version" OFF)

IF (INITIALIZE_WITH_NAN)
  MESSAGE (STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF (INITIALIZE_WITH_NAN)

IF(BUILD_UNIT_TESTS)
  SET(DISABLE_TESTS OFF)
ELSE(BUILD_UNIT_TESTS)
  SET(DISABLE_TESTS ON)
ENDIF(BUILD_UNIT_TESTS)

MACRO(TAG_LIBRARY_VERSION target)
  SET_TARGET_PROPERTIES(${target} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDMACRO(TAG_LIBRARY_VERSION)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
ADD_OPTIONAL_DEPENDENCY("metapod >= 1.0.7")
ADD_OPTIONAL_DEPENDENCY("urdfdom >= 0.2.0")
ADD_OPTIONAL_DEPENDENCY("hpp-fcl >= 0.5.1")

# Special care of urdfdom version
IF(URDFDOM_FOUND)
  IF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")
    ADD_DEFINITIONS(-DURDFDOM_COLLISION_WITH_GROUP_NAME)
    PKG_CONFIG_APPEND_CFLAGS("-DURDFDOM_COLLISION_WITH_GROUP_NAME")
  ENDIF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")

  # defines types from version 0.4.0
  IF(NOT ${URDFDOM_VERSION} VERSION_LESS "0.4.0")
    ADD_DEFINITIONS(-DURDFDOM_TYPEDEF_SHARED_PTR)
    PKG_CONFIG_APPEND_CFLAGS("-DURDFDOM_TYPEDEF_SHARED_PTR")
  ENDIF(NOT ${URDFDOM_VERSION} VERSION_LESS "0.4.0")
  
  # std::shared_ptr appears from version 1.0.0
  IF(${URDFDOM_VERSION} VERSION_GREATER "0.4.2")
    ADD_DEFINITIONS(-DURDFDOM_USE_STD_SHARED_PTR)
    PKG_CONFIG_APPEND_CFLAGS("-DURDFDOM_USE_STD_SHARED_PTR")
  ENDIF(${URDFDOM_VERSION} VERSION_GREATER "0.4.2")
ENDIF(URDFDOM_FOUND)

# Special care of lua which can be of versions 5.1 or 5.2
# Priority is set to lua5.2
IF(BUILD_WITH_LUA_SUPPORT)
  ADD_OPTIONAL_DEPENDENCY("lua5.2")
  IF(LUA5_2_FOUND)
    SET(LUA5_FOUND TRUE)
    SET(LUA5_PACKAGE "lua5.2")
    ADD_DEFINITIONS(-DLUA_VERSION_GREATER_5_2)
    PKG_CONFIG_APPEND_CFLAGS("-DLUA_VERSION_GREATER_5_2")
  ELSE(LUA5_2_FOUND)
    ADD_OPTIONAL_DEPENDENCY("lua5.1")
    IF(LUA5_1_FOUND)
      SET(LUA5_FOUND TRUE)
      SET(LUA5_PACKAGE "lua5.1")
    ELSE(LUA5_1_FOUND)
      MESSAGE(FATAL_ERROR "Cannot find a suitable version of Lua 5.1 or 5.2")
    ENDIF(LUA5_1_FOUND)
  ENDIF(LUA5_2_FOUND)
ENDIF(BUILD_WITH_LUA_SUPPORT)

SET(BOOST_REQUIERED_COMPONENTS filesystem system)
SET(BOOST_BUILD_COMPONENTS unit_test_framework)
SET(BOOST_OPTIONAL_COMPONENTS "")

IF(BUILD_PYTHON_INTERFACE)
  SET(BOOST_OPTIONAL_COMPONENTS ${BOOST_OPTIONAL_COMPONENTS} python)
  FINDPYTHON()
  INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_DIRS}) 
ENDIF(BUILD_PYTHON_INTERFACE)

SET(BOOST_COMPONENTS ${BOOST_REQUIERED_COMPONENTS} ${BOOST_OPTIONAL_COMPONENTS} ${BOOST_BUILD_COMPONENTS})
SEARCH_FOR_BOOST()

# Variable containing all the cflags definition relative to optional dependencies
SET(CFLAGS_DEPENDENCIES)

# Path to boost headers
INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIRS})

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MATH_HEADERS
  math/fwd.hpp
  math/sincos.hpp
  math/quaternion.hpp
  math/matrix.hpp
  )

SET(${PROJECT_NAME}_UTILS_HEADERS
  utils/axis-label.hpp
  utils/timer.hpp
  utils/string-generator.hpp
  utils/file-explorer.hpp
  utils/version.hpp
  )

SET(${PROJECT_NAME}_SPATIAL_HEADERS
  spatial/symmetric3.hpp
  spatial/se3.hpp
  spatial/motion.hpp
  spatial/motion-base.hpp
  spatial/motion-dense.hpp
  spatial/motion-tpl.hpp
  spatial/motion-ref.hpp
  spatial/motion-zero.hpp
  spatial/force.hpp
  spatial/force-base.hpp
  spatial/force-dense.hpp
  spatial/force-tpl.hpp
  spatial/force-ref.hpp
  spatial/inertia.hpp
  spatial/fwd.hpp
  spatial/skew.hpp
  spatial/act-on-set.hpp
  spatial/act-on-set.hxx
  spatial/explog.hpp
  spatial/cartesian-axis.hpp
  spatial/spatial-axis.hpp
  )

SET(${PROJECT_NAME}_MULTIBODY_JOINT_HEADERS
  multibody/joint/fwd.hpp
  multibody/joint/joint-base.hpp
  multibody/joint/joint-revolute.hpp
  multibody/joint/joint-revolute-unaligned.hpp
  multibody/joint/joint-revolute-unbounded.hpp
  multibody/joint/joint-spherical.hpp
  multibody/joint/joint-spherical-ZYX.hpp
  multibody/joint/joint-prismatic.hpp
  multibody/joint/joint-prismatic-unaligned.hpp
  multibody/joint/joint-planar.hpp
  multibody/joint/joint-translation.hpp
  multibody/joint/joint-free-flyer.hpp
  multibody/joint/joint-variant.hpp
  multibody/joint/joint.hpp
  multibody/joint/joint-basic-visitors.hpp
  multibody/joint/joint-basic-visitors.hxx
  multibody/joint/joint-composite.hpp
  multibody/joint/joint-composite.hxx
  ) 

SET(${PROJECT_NAME}_MULTIBODY_LIEGROUP_HEADERS
  multibody/liegroup/liegroup.hpp
  multibody/liegroup/liegroup-algo.hpp
  multibody/liegroup/liegroup-algo.hxx
  multibody/liegroup/liegroup-variant.hpp
  multibody/liegroup/liegroup-variant-visitors.hpp
  multibody/liegroup/liegroup-variant-visitors.hxx
  multibody/liegroup/operation-base.hpp
  multibody/liegroup/operation-base.hxx
  multibody/liegroup/vector-space.hpp
  multibody/liegroup/cartesian-product.hpp
  multibody/liegroup/special-orthogonal.hpp
  multibody/liegroup/special-euclidean.hpp
  ) 

SET(${PROJECT_NAME}_MULTIBODY_HEADERS
  multibody/fwd.hpp
  multibody/constraint.hpp
  multibody/force-set.hpp
  multibody/frame.hpp
  multibody/model.hpp
  multibody/model.hxx
  multibody/data.hpp
  multibody/data.hxx
  multibody/visitor.hpp
  )

SET(${PROJECT_NAME}_ALGORITHM_HEADERS
  algorithm/aba.hpp
  algorithm/aba.hxx
  algorithm/aba-derivatives.hpp
  algorithm/aba-derivatives.hxx
  algorithm/centroidal.hpp
  algorithm/centroidal.hxx
  algorithm/rnea.hpp
  algorithm/rnea.hxx
  algorithm/rnea-derivatives.hpp
  algorithm/rnea-derivatives.hxx
  algorithm/crba.hpp
  algorithm/crba.hxx
  algorithm/finite-differences.hpp
  algorithm/finite-differences.hxx
  algorithm/jacobian.hpp
  algorithm/jacobian.hxx
  algorithm/cholesky.hpp
  algorithm/cholesky.hxx
  algorithm/kinematics.hpp
  algorithm/kinematics.hxx
  algorithm/kinematics-derivatives.hpp
  algorithm/kinematics-derivatives.hxx
  algorithm/dynamics.hpp
  algorithm/center-of-mass.hpp
  algorithm/center-of-mass.hxx
  algorithm/joint-configuration.hpp
  algorithm/joint-configuration.hxx
  algorithm/energy.hpp
  algorithm/frames.hpp
  algorithm/frames.hxx
  algorithm/compute-all-terms.hpp
  algorithm/copy.hpp
  algorithm/check.hpp
  algorithm/check.hxx
  algorithm/default-check.hpp
  algorithm/regressor.hpp
  algorithm/regressor.hxx
  )

SET(${PROJECT_NAME}_CONTAINER_HEADERS
  container/aligned-vector.hpp
  )

SET(${PROJECT_NAME}_PARSERS_HEADERS
  parsers/sample-models.hpp
  parsers/utils.hpp
  parsers/srdf.hpp
  ) 

IF(URDFDOM_FOUND)
  LIST(APPEND ${PROJECT_NAME}_PARSERS_HEADERS
    parsers/urdf.hpp
    parsers/urdf/utils.hpp
    parsers/urdf/types.hpp
    )

  ADD_DEFINITIONS(-DWITH_URDFDOM)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DWITH_URDFDOM")
ENDIF(URDFDOM_FOUND)

LIST(APPEND ${PROJECT_NAME}_MULTIBODY_HEADERS
  multibody/fcl.hpp
  multibody/fcl.hxx
  multibody/geometry.hpp
  multibody/geometry.hxx
  )
LIST(APPEND ${PROJECT_NAME}_ALGORITHM_HEADERS
  algorithm/geometry.hpp
  algorithm/geometry.hxx
  )
IF(HPP_FCL_FOUND)
  LIST(APPEND ${PROJECT_NAME}_SPATIAL_HEADERS
    spatial/fcl-pinocchio-conversions.hpp
    )

  ADD_DEFINITIONS(-DWITH_HPP_FCL)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DWITH_HPP_FCL")
ENDIF(HPP_FCL_FOUND)

IF(LUA5_FOUND)
  LIST(APPEND ${PROJECT_NAME}_PARSERS_HEADERS
    parsers/lua.hpp
    parsers/lua/lua_tables.hpp
    )

  ADD_DEFINITIONS(-DWITH_LUA5)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DWITH_LUA5")
ENDIF(LUA5_FOUND)

SET(HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_UTILS_HEADERS}
  ${${PROJECT_NAME}_SPATIAL_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_JOINT_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_LIEGROUP_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_HEADERS}
  ${${PROJECT_NAME}_PARSERS_HEADERS}
  ${${PROJECT_NAME}_ALGORITHM_HEADERS}
  ${${PROJECT_NAME}_CONTAINER_HEADERS}
  exception.hpp
  assert.hpp
  macros.hpp
 )
LIST(REMOVE_DUPLICATES HEADERS)

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/math")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/joint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/liegroup")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/lua")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/urdf")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/utils")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/algorithm")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/container")

FOREACH(header ${HEADERS})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
    ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${header})
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header)

# --- MAIN LIBRARY -------------------------------------------------------------
ADD_SUBDIRECTORY(src)

# --- BINDINGS -----------------------------------------------------------------
SET(PYWRAP ${PROJECT_NAME}_pywrap)
ADD_SUBDIRECTORY(bindings)

# --- EXECUTABLES --------------------------------------------------------------
ADD_SUBDIRECTORY(utils)

# --- UNIT TESTS ---------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)

# --- BENCHMARKS ---------------------------------------------------------------
ADD_SUBDIRECTORY(benchmark)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS (${PROJECT_NAME})
PKG_CONFIG_APPEND_BOOST_LIBS(${BOOST_REQUIERED_COMPONENTS})

FOREACH(cflags ${CFLAGS_DEPENDENCIES})
  PKG_CONFIG_APPEND_CFLAGS(${cflags})
ENDFOREACH(cflags ${CFLAGS_DEPENDENCIES})

SETUP_PROJECT_FINALIZE()
