#
# Copyright (c) 2015-2016 CNRS
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
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

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
OPTION (BUILD_TESTS_WITH_HPP "Build geom tests and benchmarks with hpp to do comparisons" OFF)
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)

IF (INITIALIZE_WITH_NAN)
  MESSAGE (STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF (INITIALIZE_WITH_NAN)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
ADD_OPTIONAL_DEPENDENCY("metapod >= 1.0.7")
ADD_OPTIONAL_DEPENDENCY("urdfdom >= 0.2.0")
ADD_OPTIONAL_DEPENDENCY("lua5.1")
ADD_OPTIONAL_DEPENDENCY("hpp-fcl >= 0.4.2")

# Spetcial care of Eigen3 greater than 3.2.10
IF(EIGEN3_FOUND)
  IF(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
    ADD_DEFINITIONS(-DEIGEN3_FUTURE)
    SET(EIGEN3_FUTURE TRUE)
    PKG_CONFIG_APPEND_CFLAGS("-DEIGEN3_FUTURE")
  ELSE(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
    SET(EIGEN3_FUTURE FALSE)
  ENDIF(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
ENDIF(EIGEN3_FOUND)

# Special care of urdfdom less than 0.3.0
IF(URDFDOM_FOUND)
  IF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")
    ADD_DEFINITIONS(-DURDFDOM_COLLISION_WITH_GROUP_NAME)
    SET(URDFDOM_COLLISION_WITH_GROUP_NAME TRUE)
    PKG_CONFIG_APPEND_CFLAGS("-DURDFDOM_COLLISION_WITH_GROUP_NAME")
  ENDIF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")
ENDIF(URDFDOM_FOUND)

SET(BOOST_REQUIERED_COMPONENTS filesystem system)
SET(BOOST_BUILD_COMPONENTS unit_test_framework)
SET(BOOST_OPTIONAL_COMPONENTS "")

IF(BUILD_PYTHON_INTERFACE)
  SET(BOOST_OPTIONAL_COMPONENTS ${BOOST_OPTIONAL_COMPONENTS} python)
  FINDPYTHON(2.7 EXACT REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS}) 
ENDIF(BUILD_PYTHON_INTERFACE)

SET(BOOST_COMPONENTS ${BOOST_REQUIERED_COMPONENTS} ${BOOST_OPTIONAL_COMPONENTS} ${BOOST_BUILD_COMPONENTS})
SEARCH_FOR_BOOST()

# Variable containing all the cflags definition relative to optional dependencies
SET(CFLAGS_DEPENDENCIES)

# Path to boost headers
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MATH_HEADERS
  math/fwd.hpp
  math/sincos.hpp
  math/quaternion.hpp
  math/matrix.hpp
  )

SET(${PROJECT_NAME}_TOOLS_HEADERS
  tools/timer.hpp
  tools/string-generator.hpp
  tools/file-explorer.hpp
  )

SET(${PROJECT_NAME}_SPATIAL_HEADERS
  spatial/symmetric3.hpp
  spatial/se3.hpp
  spatial/motion.hpp
  spatial/force.hpp
  spatial/inertia.hpp
  spatial/fwd.hpp
  spatial/skew.hpp
  spatial/act-on-set.hpp
  spatial/explog.hpp
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
  ) 

SET(${PROJECT_NAME}_MULTIBODY_HEADERS
  multibody/fwd.hpp
  multibody/constraint.hpp
  multibody/force-set.hpp
  multibody/frame.hpp
  multibody/model.hpp
  multibody/model.hxx
  multibody/visitor.hpp
  )

SET(${PROJECT_NAME}_ALGORITHM_HEADERS
  algorithm/aba.hpp
  algorithm/aba.hxx
  algorithm/rnea.hpp
  algorithm/rnea.hxx
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
  algorithm/dynamics.hpp
  algorithm/center-of-mass.hpp
  algorithm/center-of-mass.hxx
  algorithm/joint-configuration.hpp
  algorithm/energy.hpp
  algorithm/frames.hpp
  algorithm/compute-all-terms.hpp
  algorithm/copy.hpp
  algorithm/check.hpp
  algorithm/check.hxx
  algorithm/default-check.hpp
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

IF(LUA5_1_FOUND)
  LIST(APPEND ${PROJECT_NAME}_PARSERS_HEADERS
    parsers/lua.hpp
    parsers/lua/lua_tables.hpp
    )

  ADD_DEFINITIONS(-DWITH_LUA)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DWITH_LUA")
ENDIF(LUA5_1_FOUND)

SET(HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TOOLS_HEADERS}
  ${${PROJECT_NAME}_SPATIAL_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_JOINT_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_HEADERS}
  ${${PROJECT_NAME}_PARSERS_HEADERS}
  ${${PROJECT_NAME}_ALGORITHM_HEADERS}
  ${${PROJECT_NAME}_CONTAINER_HEADERS}
  exception.hpp
  assert.hpp
 )
LIST(REMOVE_DUPLICATES HEADERS)

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/math")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/joint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/lua")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/urdf")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/tools")
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
