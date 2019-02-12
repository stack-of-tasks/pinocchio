SET(${PROJECT_NAME}_PYTHON_TESTS
  bindings
  #bindings_com
  #bindings_dynamics
  bindings_force
  bindings_frame
  bindings_geometry_object
  bindings_geometry_object_urdf
  bindings_inertia
  bindings_joint_composite
  bindings_motion
  bindings_SE3
  #explog
  model
  rpy
  utils
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "unittest/python/${TEST}.py" "bindings/python")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
