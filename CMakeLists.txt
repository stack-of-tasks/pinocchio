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

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
SET(HEADERS
  exception.hpp
  assert.hpp
  math/sincos.hpp
  tools/timer.hpp
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
  multibody/parser/sample-models.hpp
  algorithm/rnea.hpp
  algorithm/crba.hpp
  algorithm/jacobian.hpp
  algorithm/cholesky.hpp
  algorithm/kinematics.hpp
  algorithm/center-of-mass.hpp
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

IF(URDFDOM_FOUND)
  LIST(APPEND HEADERS multibody/parser/urdf.hpp)
ENDIF(URDFDOM_FOUND)

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

# ----------------------------------------------------
# --- PYTHON -----------------------------------------
# ----------------------------------------------------

IF(EIGENPY_FOUND)
  FINDPYTHON()
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")
  
  # --- COMPILE WRAPPER
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  ADD_LIBRARY(${PYWRAP} SHARED src/python/module.cpp src/python/python.cpp)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
  IF(URDFDOM_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} urdfdom)
    
  ENDIF(URDFDOM_FOUND)
  TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_LIBRARIES} eigenpy)
  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}")
  INSTALL(FILES
    "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}/lib${PYWRAP}.so"
    DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  
  # --- INSTALL SCRIPTS 
  SET(PYTHON_FILES
    python/__init__.py
    python/utils.py
    python/robot_wrapper.py
    python/rpy.py
    python/explog.py
    )
  FOREACH(python ${PYTHON_FILES})
    GET_FILENAME_COMPONENT(pythonFile ${python} NAME)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/src/${python}
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})
    
    # Tag pyc file as generated.
    SET_SOURCE_FILES_PROPERTIES(
      "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      PROPERTIES GENERATED TRUE)
    
    EXECUTE_PROCESS(COMMAND
      ${PYTHON_EXECUTABLE} -m py_compile
      ${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile} )
    
    # Clean generated files.
    SET_PROPERTY(
      DIRECTORY APPEND PROPERTY
      ADDITIONAL_MAKE_CLEAN_FILES
      "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c")
    
    INSTALL(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}"
      "${CMAKE_CURRENT_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  ENDFOREACH(python)
ENDIF(EIGENPY_FOUND)

# --- EXECUTABLES --------------------------------------------------------------
# --- EXECUTABLES --------------------------------------------------------------
# --- EXECUTABLES --------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)
IF (BUILD_BENCHMARK)
  ADD_SUBDIRECTORY (benchmark)
ENDIF (BUILD_BENCHMARK)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

