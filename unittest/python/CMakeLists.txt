# #
# # Copyright (c) 2015-2023 CNRS INRIA, 2024-2025 INRIA
# #
# include(${JRL_CMAKE_MODULES}/python-helpers.cmake)

# set(${PROJECT_NAME}_PYTHON_TESTS bindings # Multibody bindings_joint_composite bindings_joints
# bindings_model bindings_data bindings_geometry_model bindings_liegroups # Spatial bindings_force
# bindings_frame bindings_inertia bindings_spatial bindings_motion bindings_SE3 explog rpy # Algo
# bindings_com bindings_kinematic_regressor bindings_regressor bindings_dynamics bindings_kinematics
# bindings_rnea bindings_aba bindings_joint_algorithms # Algo derivatives
# bindings_kinematics_derivatives bindings_frame_derivatives bindings_forward_dynamics_derivatives
# bindings_inverse_dynamics_derivatives bindings_centroidal_dynamics_derivatives
# bindings_com_velocity_derivatives # Parsers bindings_sample_models # Others robot_wrapper utils
# serialization version bindings_std_vector bindings_std_map)

# function(pinocchio_add_python_cpp_module name) set(target_name "test-ext-${name}") string(REPLACE
# "_" "-" target_name ${target_name}) string(REPLACE "-" "_" module_name ${target_name})
# set(source_file "${name}.cpp") set(options) set(oneValueArgs) set(multiValueArgs "PIN_TARGETS")
# cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
# if(BUILD_TESTING) set(_exclude) else() set(_exclude EXCLUDE_FROM_ALL) endif()

# add_library(${target_name} MODULE ${_exclude} ${source_file})
# target_compile_definitions(${target_name} PRIVATE EXT_MODULE_NAME=${module_name})
# target_link_libraries(${target_name} PRIVATE pinocchio_default eigenpy::eigenpy) foreach(_dep
# ${ARGS_PIN_TARGETS}) target_link_libraries(${target_name} PRIVATE ${_dep}) endforeach()
# set_target_properties( ${target_name} PROPERTIES PREFIX "" SUFFIX "${PYTHON_EXT_SUFFIX}"
# LIBRARY_OUTPUT_NAME ${module_name} RUNTIME_OUTPUT_NAME ${module_name})

# add_test( NAME ${target_name} COMMAND ${PYTHON_EXECUTABLE} -c "import ${module_name}"
# WORKING_DIRECTORY $<TARGET_FILE_DIR:${target_name}>) set(PYTHONPATH)
# compute_pythonpath(ENV_VARIABLES bindings/python) set_tests_properties(${target_name} PROPERTIES
# ENVIRONMENT "${ENV_VARIABLES}")

# add_dependencies(build_tests ${target_name}) if(NOT BUILD_TESTING)
# set_tests_properties(${target_name} PROPERTIES DEPENDS ctest_build_tests) endif() endfunction()

# function(pinocchio_add_lib_unit_test name) set(TEST_NAME "${PROJECT_NAME}-test-py-${name}")
# add_python_unit_test(${TEST_NAME} "unittest/python/${name}.py" "bindings/python"
# "unittest/python") add_windows_dll_path_to_test(${TEST_NAME}) endfunction()

# if(BUILD_WITH_HPP_FCL_SUPPORT) set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS}
# bindings_geometry_object) set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS}
# bindings_fcl_transform) endif()

# if(urdfdom_FOUND) set(${PROJECT_NAME}_PYTHON_TESTS ${${PROJECT_NAME}_PYTHON_TESTS} bindings_urdf
# bindings_geometry_model_urdf bindings_contact_inverse_dynamics) endif(urdfdom_FOUND)

# if(TARGET build_tests) add_dependencies(build_tests ${PROJECT_NAME}-python) endif()

# foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS}) set(TEST_NAME "${PROJECT_NAME}-test-py-${TEST}")
# add_python_unit_test(${TEST_NAME} "unittest/python/${TEST}.py" "bindings/python")
# add_windows_dll_path_to_test(${TEST_NAME}) endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

# make_directory("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")

# if(BUILD_WITH_CASADI_SUPPORT) add_subdirectory(casadi) endif(BUILD_WITH_CASADI_SUPPORT)

# # TODO BUILD_ADVANCED_TESTING if(MEMORYCHECK_COMMAND AND MEMORYCHECK_COMMAND MATCHES
# ".*valgrind$") set(${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS
# bindings_build_geom_from_urdf_memorycheck)

# foreach(TEST ${${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS}) set(TEST_NAME
# "${PROJECT_NAME}-test-py-memory-${TEST}") add_python_memorycheck_unit_test_v2( NAME ${TEST_NAME}
# SOURCE "unittest/python/${TEST}.py" SUPP "unittest/python/${TEST}.supp" MODULES "bindings/python")
# add_windows_dll_path_to_test(${TEST_NAME}) endforeach() else() message(STATUS "Valgrind not found,
# memory checks are disabled") endif()

# # TODO @jorisv: reenable these on windows when we normalize paths to lib/bin like on eigenpy
# if(NOT WIN32) pinocchio_add_python_cpp_module(bindings_visualizer PIN_TARGETS
# pinocchio_visualizers) pinocchio_add_lib_unit_test(bindings_visualizer) endif()

# add_subdirectory(pybind11)
