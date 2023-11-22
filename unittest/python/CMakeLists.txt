#
# Copyright (c) 2015-2023 CNRS INRIA
#

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
