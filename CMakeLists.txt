#
#  Copyright 2014 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME pinocchio)
SET(PROJECT_DESCRIPTION "Rigid multi body dynamics algorithms")
SET(PROJECT_URL "http://github.com/stac-of-tasks/pinocchio")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

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
ADD_REQUIRED_DEPENDENCY("eigen3 >= v3.0.5")

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
SET(${PROJECT_NAME}_HEADERS
  include/pinocchio/exception.hpp
  include/pinocchio/assert.hpp
  include/pinocchio/math/sincos.hpp
  include/pinocchio/spatial/symmetric3.hpp
  include/pinocchio/spatial/se3.hpp
  include/pinocchio/spatial/motion.hpp
  include/pinocchio/spatial/force.hpp
  include/pinocchio/spatial/inertia.hpp
  include/pinocchio/spatial/fwd.hpp
  include/pinocchio/spatial/skew.hpp
  include/pinocchio/spatial/act-on-set.hpp
  include/pinocchio/multibody/constraint.hpp
  include/pinocchio/multibody/force-set.hpp
  include/pinocchio/multibody/joint.hpp
  include/pinocchio/multibody/joint/joint-base.hpp
  include/pinocchio/multibody/joint/joint-revolute.hpp
  include/pinocchio/multibody/joint/joint-free-flyer.hpp
  include/pinocchio/multibody/joint/joint-variant.hpp
  include/pinocchio/multibody/joint/joint-generic.hpp
  include/pinocchio/multibody/model.hpp
  include/pinocchio/multibody/visitor.hpp
  include/pinocchio/multibody/parser/urdf.hpp
  include/pinocchio/multibody/parser/sample-models.hpp
  include/pinocchio/tools/timer.hpp
  include/pinocchio/algorithm/rnea.hpp
  include/pinocchio/algorithm/crba.hpp
  include/pinocchio/algorithm/jacobian.hpp
  include/pinocchio/algorithm/cholesky.hpp
  include/pinocchio/algorithm/kinematics.hpp
  include/pinocchio/algorithm/center-of-mass.hpp
  include/pinocchio/python/eigen_container.hpp
  include/pinocchio/python/handler.hpp
  include/pinocchio/python/python.hpp
  include/pinocchio/python/se3.hpp
  include/pinocchio/python/force.hpp
  include/pinocchio/python/motion.hpp
  include/pinocchio/python/inertia.hpp
  include/pinocchio/python/model.hpp
  include/pinocchio/python/data.hpp
  include/pinocchio/python/algorithms.hpp
)


# ----------------------------------------------------
# --- PYTHON -----------------------------------------
# ----------------------------------------------------
FINDPYTHON()
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")

# --- SET UP PYTHON WRAPPER LIBRARY
SET(PYWRAP ${PROJECT_NAME}_pywrap)
SET(${PYWRAP}_SOURCES src/python/module.cpp src/python/python.cpp) 

ADD_LIBRARY(${PYWRAP} SHARED ${${PYWRAP}_SOURCES})
PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_LIBRARIES})
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
PKG_CONFIG_USE_DEPENDENCY(tspatial eigen3)

ADD_EXECUTABLE(symmetric EXCLUDE_FROM_ALL unittest/symmetric.cpp)
PKG_CONFIG_USE_DEPENDENCY(symmetric eigen3)

ADD_EXECUTABLE(constraint EXCLUDE_FROM_ALL unittest/constraint.cpp)
PKG_CONFIG_USE_DEPENDENCY(constraint eigen3)

ADD_EXECUTABLE(rnea EXCLUDE_FROM_ALL unittest/rnea.cpp)
PKG_CONFIG_USE_DEPENDENCY(rnea eigen3)

ADD_EXECUTABLE(crba EXCLUDE_FROM_ALL unittest/crba.cpp)
PKG_CONFIG_USE_DEPENDENCY(crba eigen3)
PKG_CONFIG_USE_DEPENDENCY(crba urdfdom)

ADD_EXECUTABLE(timings EXCLUDE_FROM_ALL unittest/timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigen3)
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

ADD_EXECUTABLE(jacobian EXCLUDE_FROM_ALL unittest/jacobian.cpp)
PKG_CONFIG_USE_DEPENDENCY(jacobian eigen3)

ADD_EXECUTABLE(com EXCLUDE_FROM_ALL unittest/com.cpp)
PKG_CONFIG_USE_DEPENDENCY(com eigen3)

ADD_EXECUTABLE(cholesky EXCLUDE_FROM_ALL unittest/cholesky.cpp)
PKG_CONFIG_USE_DEPENDENCY(cholesky eigen3)
PKG_CONFIG_USE_DEPENDENCY(cholesky urdfdom)

#ADD_EXECUTABLE(dg EXCLUDE_FROM_ALL unittest/dg.cpp)
#PKG_CONFIG_USE_DEPENDENCY(dg eigenpy)

ADD_EXECUTABLE(urdf EXCLUDE_FROM_ALL unittest/urdf.cpp)
PKG_CONFIG_USE_DEPENDENCY(urdf eigen3)
PKG_CONFIG_USE_DEPENDENCY(urdf urdfdom)

ADD_EXECUTABLE(value EXCLUDE_FROM_ALL unittest/value.cpp)
PKG_CONFIG_USE_DEPENDENCY(value eigen3)
PKG_CONFIG_USE_DEPENDENCY(value urdfdom)

ADD_EXECUTABLE(variant EXCLUDE_FROM_ALL unittest/variant.cpp)
PKG_CONFIG_USE_DEPENDENCY(variant eigen3)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

