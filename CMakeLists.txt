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

# --- OPTIONS ----------------------------------------
OPTION (BUILD_BENCHMARK "Build the benchmarks" OFF)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
ADD_OPTIONAL_DEPENDENCY("eigenpy >= 1.2.0")
ADD_OPTIONAL_DEPENDENCY("metapod >= 1.0.7")
ADD_OPTIONAL_DEPENDENCY("urdfdom >= 0.2.10")

SET(BOOST_COMPONENTS filesystem log log_setup unit_test_framework)
SEARCH_FOR_BOOST()
# Path to boost headers
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------


SET(${PROJECT_NAME}_MATH_HEADERS
  math/sincos.hpp
  )

SET(${PROJECT_NAME}_TOOLS_HEADERS
  tools/timer.hpp
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
  )

SET(${PROJECT_NAME}_MULTIBODY_HEADERS
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
  multibody/parser/sample-models.hpp
  )

SET(${PROJECT_NAME}_ALGORITHM_HEADERS
  algorithm/rnea.hpp
  algorithm/crba.hpp
  algorithm/jacobian.hpp
  algorithm/cholesky.hpp
  algorithm/kinematics.hpp
  algorithm/center-of-mass.hpp
  )


SET(${PROJECT_NAME}_PYTHON_HEADERS
  python/eigen_container.hpp
  python/handler.hpp
  python/python.hpp
  python/se3.hpp
  python/force.hpp
  python/motion.hpp
  python/inertia.hpp
  python/model.hpp
  python/data.hpp
  python/algorithms.hpp
  python/parsers.hpp
  )

SET(HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TOOLS_HEADERS}
  ${${PROJECT_NAME}_SPATIAL_HEADERS}
  ${${PROJECT_NAME}_MULTIBODY_HEADERS}
  ${${PROJECT_NAME}_ALGORITHM_HEADERS}
  ${${PROJECT_NAME}_PYTHON_HEADERS}
  exception.hpp
  assert.hpp
  )

IF(URDFDOM_FOUND)
  LIST(APPEND HEADERS multibody/parser/urdf.hpp)
ENDIF(URDFDOM_FOUND)

PKG_CONFIG_APPEND_LIBS (${PROJECT_NAME})

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/math")
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

ADD_SUBDIRECTORY(src)
# --- EXECUTABLES --------------------------------------------------------------
# --- EXECUTABLES --------------------------------------------------------------
# --- EXECUTABLES --------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)
IF (BUILD_BENCHMARK)
  ADD_SUBDIRECTORY (benchmark)
ENDIF (BUILD_BENCHMARK)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
