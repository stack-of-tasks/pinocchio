#
#  Copyright 2014 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME pinocchio)
SET(PROJECT_DESCRIPTION "Rigid multi body dynamics algorithms")
SET(PROJECT_URL "http://github.com/stac-of-tasks/pinocchio")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

project(${PROJECT_NAME})
SETUP_PROJECT()

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigenpy >= v1.0.1")
ADD_REQUIRED_DEPENDENCY("urdfdom >= v0.3.0")

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
SET(${PROJECT_NAME}_HEADERS
  exception.hpp
  spatial/se3.hpp
  spatial/motion.hpp
  spatial/force.hpp
  spatial/inertia.hpp
  spatial/fwd.hpp
  spatial/skew.hpp
  multibody/constraint.hpp
  multibody/force-set.hpp
  multibody/joint.hpp
  multibody/joint/joint-base.hpp
  multibody/joint/joint-revolute.hpp
  multibody/joint/joint-free-flyer.hpp
  multibody/joint/joint-variant.hpp
  multibody/joint/joint-generic.hpp
  multibody/model.hpp
  multibody/visitor.hpp
  multibody/parser/urdf.hpp
  tools/timer.hpp
  algorithm/rnea.hpp
  algorithm/crba.hpp
  algorithm/kinematics.hpp
)

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/joint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/parser")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/tools")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/algorithm")

FOREACH(header ${${PROJECT_NAME}_HEADERS})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  IF(WIN32)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
                    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${header})
  ELSE(WIN32)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                    ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
                    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${header})
  ENDIF(WIN32)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
	  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header)

# ----------------------------------------------------
# --- TARGETS ----------------------------------------
# ----------------------------------------------------
ADD_EXECUTABLE(tspatial EXCLUDE_FROM_ALL unittest/tspatial.cpp)
PKG_CONFIG_USE_DEPENDENCY(tspatial eigenpy)

ADD_EXECUTABLE(constraint EXCLUDE_FROM_ALL unittest/constraint.cpp)
PKG_CONFIG_USE_DEPENDENCY(constraint eigenpy)

ADD_EXECUTABLE(rnea EXCLUDE_FROM_ALL unittest/rnea.cpp)
PKG_CONFIG_USE_DEPENDENCY(rnea eigenpy)

ADD_EXECUTABLE(crba EXCLUDE_FROM_ALL unittest/crba.cpp)
PKG_CONFIG_USE_DEPENDENCY(crba eigenpy)
PKG_CONFIG_USE_DEPENDENCY(crba urdfdom)

ADD_EXECUTABLE(dg EXCLUDE_FROM_ALL unittest/dg.cpp)
PKG_CONFIG_USE_DEPENDENCY(dg eigenpy)

ADD_EXECUTABLE(urdf EXCLUDE_FROM_ALL unittest/urdf.cpp)
PKG_CONFIG_USE_DEPENDENCY(urdf eigenpy)
PKG_CONFIG_USE_DEPENDENCY(urdf urdfdom)

ADD_EXECUTABLE(value EXCLUDE_FROM_ALL unittest/value.cpp)
PKG_CONFIG_USE_DEPENDENCY(value eigenpy)
PKG_CONFIG_USE_DEPENDENCY(value urdfdom)

ADD_EXECUTABLE(variant EXCLUDE_FROM_ALL unittest/variant.cpp)
PKG_CONFIG_USE_DEPENDENCY(variant eigenpy)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

