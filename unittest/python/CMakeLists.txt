#
# Copyright (c) 2015-2023 CNRS INRIA
#

macro(ADD_PYTHON_MEMORYCHECK_UNIT_TEST NAME SOURCE)
  set(TEST_FILE_NAME memorycheck_unit_test_${NAME}.cmake)
  set(PYTHON_TEST_SCRIPT "${PROJECT_SOURCE_DIR}/${SOURCE}")
  configure_file(memorycheck_unit_test.cmake.in ${TEST_FILE_NAME} @ONLY)

  add_test(NAME ${NAME}
    COMMAND ${CMAKE_COMMAND} -P ${TEST_FILE_NAME})

  set(MODULES "${ARGN}") # ARGN is not a variable
  foreach(MODULE_PATH IN LISTS MODULES)
    list(APPEND PYTHONPATH "${CMAKE_BINARY_DIR}/${MODULE_PATH}")
  endforeach()

  if(DEFINED ENV{PYTHONPATH})
    list(APPEND PYTHONPATH "$ENV{PYTHONPATH}")
  endif()

  # get path separator to join those paths
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" "-c" "import os; print(os.pathsep)"
    OUTPUT_VARIABLE PATHSEP
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  list(REMOVE_DUPLICATES PYTHONPATH)
  if(WIN32)
    # ensure that severals paths stay together as ENV variable PYTHONPATH when
    # passed to python test via PROPERTIES
    string(REPLACE ";" "\;" PYTHONPATH_STR "${PYTHONPATH}")
  else(WIN32)
    string(REPLACE ";" "${PATHSEP}" PYTHONPATH_STR "${PYTHONPATH}")
  endif(WIN32)
  set(ENV_VARIABLES "PYTHONPATH=${PYTHONPATH_STR}")
  if(APPLE)
    list(APPEND ENV_VARIABLES "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}")
    list(APPEND ENV_VARIABLES "DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
  endif(APPLE)
  set_tests_properties(${NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
endmacro()

SET(${PROJECT_NAME}_PYTHON_TESTS
  bindings

  # Multibody
  bindings_joint_composite
  bindings_joints
  bindings_model
  bindings_data
  bindings_geometry_model
  bindings_liegroups

  # Spatial
  bindings_force
  bindings_frame
  bindings_inertia
  bindings_spatial
  bindings_motion
  bindings_SE3
  explog
  rpy

  # Algo
  bindings_com
  bindings_kinematic_regressor
  bindings_regressor
  bindings_dynamics
  bindings_kinematics
  bindings_rnea
  bindings_aba
  bindings_joint_algorithms
  
  # Algo derivatives
  bindings_kinematics_derivatives
  bindings_frame_derivatives
  bindings_forward_dynamics_derivatives
  bindings_inverse_dynamics_derivatives
  bindings_centroidal_dynamics_derivatives
  bindings_com_velocity_derivatives

  # Parsers
  bindings_sample_models

  # Others
  utils
  serialization
  version
  bindings_std_vector
  bindings_std_map
  )

IF(hpp-fcl_FOUND)
  SET(${PROJECT_NAME}_PYTHON_TESTS
    ${${PROJECT_NAME}_PYTHON_TESTS}
    bindings_geometry_object
    )
  IF(BUILD_WITH_HPP_FCL_PYTHON_BINDINGS)
    SET(${PROJECT_NAME}_PYTHON_TESTS
      ${${PROJECT_NAME}_PYTHON_TESTS}
      bindings_fcl_transform
      )
  ENDIF(BUILD_WITH_HPP_FCL_PYTHON_BINDINGS)
ENDIF(hpp-fcl_FOUND)

IF(urdfdom_FOUND)
  SET(${PROJECT_NAME}_PYTHON_TESTS
    ${${PROJECT_NAME}_PYTHON_TESTS}
    bindings_urdf
    bindings_geometry_model_urdf
    )
ENDIF(urdfdom_FOUND)

SET(${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS
  bindings_build_geom_from_urdf_memorycheck
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("test-py-${TEST}" "unittest/python/${TEST}.py" "bindings/python")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")

IF(MEMORYCHECK_COMMAND AND MEMORYCHECK_COMMAND MATCHES ".*valgrind$")
  FOREACH(TEST ${${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS})
    ADD_PYTHON_MEMORYCHECK_UNIT_TEST("test-py-memory-${TEST}"
      "unittest/python/${TEST}.py"
      "bindings/python")
  ENDFOREACH()
ELSE()
  MESSAGE(STATUS "Valgrind not found, memory checks are disabled")
ENDIF()

ADD_SUBDIRECTORY(pybind11)
