#
# Copyright (c) 2015-2023 CNRS INRIA
#

set(${PROJECT_NAME}_PYTHON_TESTS
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
    robot_wrapper
    utils
    serialization
    version
    bindings_std_vector
    bindings_std_map)

if(BUILD_WITH_HPP_FCL_SUPPORT)
  set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS} bindings_geometry_object)
  set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS} bindings_fcl_transform)
endif()

if(urdfdom_FOUND)
  set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS} bindings_urdf
                                   bindings_geometry_model_urdf bindings_contact_inverse_dynamics)
endif(urdfdom_FOUND)

foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  set(TEST_NAME "${PROJECT_NAME}-test-py-${TEST}")
  add_python_unit_test(${TEST_NAME} "unittest/python/${TEST}.py" "bindings/python")
  add_windows_dll_path_to_test(${TEST_NAME})
endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

make_directory("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")

if(BUILD_WITH_CASADI_SUPPORT)
  add_subdirectory(casadi)
endif(BUILD_WITH_CASADI_SUPPORT)

# TODO BUILD_ADVANCED_TESTING
if(MEMORYCHECK_COMMAND AND MEMORYCHECK_COMMAND MATCHES ".*valgrind$")
  set(${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS bindings_build_geom_from_urdf_memorycheck)

  foreach(TEST ${${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS})
    set(TEST_NAME "${PROJECT_NAME}-test-py-memory-${TEST}")
    add_python_memorycheck_unit_test_v2(
      NAME
      ${TEST_NAME}
      SOURCE
      "unittest/python/${TEST}.py"
      SUPP
      "unittest/python/${TEST}.supp"
      MODULES
      "bindings/python")
    add_windows_dll_path_to_test(${TEST_NAME})
  endforeach()
else()
  message(STATUS "Valgrind not found, memory checks are disabled")
endif()

add_subdirectory(pybind11)
