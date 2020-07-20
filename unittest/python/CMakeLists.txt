#
# Copyright (c) 2015-2020 CNRS INRIA
#

SET(${PROJECT_NAME}_PYTHON_TESTS
  bindings
  bindings_model
  bindings_data
  bindings_com
  bindings_com_velocity_derivatives
  bindings_regressor
  bindings_dynamics
  bindings_force
  bindings_frame
  bindings_kinematics
  bindings_inertia
  bindings_spatial
  bindings_joint_composite
  bindings_motion
  bindings_SE3
  bindings_geometry_model
  bindings_rnea
  bindings_liegroups
  explog
  rpy
  utils
  serialization
  version
  )

IF(HPP_FCL_FOUND)
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
ENDIF(HPP_FCL_FOUND)

IF(URDFDOM_FOUND)
  SET(${PROJECT_NAME}_PYTHON_TESTS
    ${${PROJECT_NAME}_PYTHON_TESTS}
    bindings_urdf
    bindings_geometry_model_urdf
    )
ENDIF(URDFDOM_FOUND)

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("test-py-${TEST}" "unittest/python/${TEST}.py" "bindings/python")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")
