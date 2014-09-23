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

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)


# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigenpy >= v1.0.1")
ADD_REQUIRED_DEPENDENCY("urdfdom >= v0.3.0")

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
SET(HEADERS
  exception.hpp
  assert.hpp
  spatial/symmetric3.hpp
  spatial/se3.hpp
  spatial/motion.hpp
  spatial/force.hpp
  spatial/inertia.hpp
  spatial/fwd.hpp
  spatial/skew.hpp
  spatial/act-on-set.hpp
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
  multibody/parser/sample-models.hpp
  tools/timer.hpp
  algorithm/rnea.hpp
  algorithm/crba.hpp
  algorithm/jacobian.hpp
  algorithm/cholesky.hpp
  algorithm/kinematics.hpp
  algorithm/center-of-mass.hpp
  python/python.hpp
  python/se3.hpp
  python/force.hpp
  python/motion.hpp
  python/inertia.hpp
  python/model.hpp
  python/data.hpp
)

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/joint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/multibody/parser")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/tools")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/algorithm")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/python")

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

# ----------------------------------------------------
# --- PYTHON -----------------------------------------
# ----------------------------------------------------
FINDPYTHON()
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")

# --- COMPILE WRAPPER
SET(PYWRAP ${PROJECT_NAME}_pywrap)
ADD_LIBRARY(${PYWRAP} SHARED src/python/module.cpp src/python/python.cpp)
PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_LIBRARIES} eigenpy)
SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}")

# --- INSTALL SCRIPTS 
SET(PYTHON_FILES
  python/__init__.py
  python/utils.py
)
FOREACH(python ${PYTHON_FILES})
  GET_FILENAME_COMPONENT(pythonFile ${python} NAME)
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
    ${${PROJECT_NAME}_SOURCE_DIR}/src/${python}
    ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})

  # Tag pyc file as generated.
  #SET_SOURCE_FILES_PROPERTIES(
  #  "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${pythonFile}c"
  #  PROPERTIES GENERATED TRUE)

  # Clean generated files.
  #SET_PROPERTY(
  #  DIRECTORY APPEND PROPERTY
  #  ADDITIONAL_MAKE_CLEAN_FILES
  #  "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${pythonFile}c")

  #INSTALL(CODE
  #  "EXECUTE_PROCESS(COMMAND
  #  \"${PYTHON_EXECUTABLE}\"
  #  \"-m py_compile\"
  #  \"${CMAKE_CURRENT_BINARY_DIR}/lib/python/${pythonFile}\")    ")

  INSTALL(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}"
  #  "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${pythonFile}c"
    DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
ENDFOREACH(python)

# ----------------------------------------------------
# --- UNITTESTS --------------------------------------
# ----------------------------------------------------
ADD_EXECUTABLE(tspatial EXCLUDE_FROM_ALL unittest/tspatial.cpp)
PKG_CONFIG_USE_DEPENDENCY(tspatial eigenpy)

ADD_EXECUTABLE(symmetric EXCLUDE_FROM_ALL unittest/symmetric.cpp)
PKG_CONFIG_USE_DEPENDENCY(symmetric eigenpy)

ADD_EXECUTABLE(constraint EXCLUDE_FROM_ALL unittest/constraint.cpp)
PKG_CONFIG_USE_DEPENDENCY(constraint eigenpy)

ADD_EXECUTABLE(rnea EXCLUDE_FROM_ALL unittest/rnea.cpp)
PKG_CONFIG_USE_DEPENDENCY(rnea eigenpy)

ADD_EXECUTABLE(crba EXCLUDE_FROM_ALL unittest/crba.cpp)
PKG_CONFIG_USE_DEPENDENCY(crba eigenpy)
PKG_CONFIG_USE_DEPENDENCY(crba urdfdom)

ADD_EXECUTABLE(timings EXCLUDE_FROM_ALL unittest/timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigenpy)
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

ADD_EXECUTABLE(jacobian EXCLUDE_FROM_ALL unittest/jacobian.cpp)
PKG_CONFIG_USE_DEPENDENCY(jacobian eigenpy)

ADD_EXECUTABLE(com EXCLUDE_FROM_ALL unittest/com.cpp)
PKG_CONFIG_USE_DEPENDENCY(com eigenpy)

ADD_EXECUTABLE(cholesky EXCLUDE_FROM_ALL unittest/cholesky.cpp)
PKG_CONFIG_USE_DEPENDENCY(cholesky eigenpy)
PKG_CONFIG_USE_DEPENDENCY(cholesky urdfdom)

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

