#
# Copyright (c) 2015-2025 CNRS INRIA
#

function(pinocchio_add_python_test_or_example target_name file_name)
    add_test(
        NAME ${target_name}
        COMMAND
            $<TARGET_FILE:Python::Interpreter>
            ${CMAKE_CURRENT_SOURCE_DIR}/${file_name}.py
    )
    set_tests_properties(
        ${target_name}
        PROPERTIES
            ENVIRONMENT
                "PYTHONPATH=$<TARGET_FILE_DIR:pinocchio_pywrap_default>/.."
            LABELS "python"
    )
    # TODO: Remove this once coal and example-robot-data also use jrl-v2.
    # (Only meaningful when Pinocchio is built in-tree inside the
    #  simple-workspace superbuild; empty/no-op in a standalone build.)
    string(
        REPLACE ";"
        ":"
        _pythonpath
        "${PROJECT_PYTHON_PACKAGES_IN_WORKSPACE}"
    )
    set_tests_properties(
        ${target_name}
        PROPERTIES
            ENVIRONMENT_MODIFICATION
                "EXAMPLE_ROBOT_DATA_MODEL_DIR=set:${EXAMPLE_ROBOT_DATA_MODEL_DIR};PYTHONPATH=path_list_prepend:${_pythonpath}"
    )
endfunction()

function(pinocchio_add_python_test name)
    pinocchio_add_python_test_or_example("${PROJECT_NAME}-test-py-${name}" "${name}")
endfunction()

pinocchio_add_python_test(bindings)
# Multibody
pinocchio_add_python_test(bindings_joint_composite)
pinocchio_add_python_test(bindings_joints)
pinocchio_add_python_test(bindings_model)
pinocchio_add_python_test(bindings_data)
pinocchio_add_python_test(bindings_geometry_model)
pinocchio_add_python_test(bindings_liegroups)
# Spatial
pinocchio_add_python_test(bindings_force)
pinocchio_add_python_test(bindings_frame)
pinocchio_add_python_test(bindings_inertia)
pinocchio_add_python_test(bindings_spatial)
pinocchio_add_python_test(bindings_motion)
pinocchio_add_python_test(bindings_SE3)
pinocchio_add_python_test(explog)
pinocchio_add_python_test(rpy)
# Algo
pinocchio_add_python_test(bindings_com)
pinocchio_add_python_test(bindings_kinematic_regressor)
pinocchio_add_python_test(bindings_regressor)
pinocchio_add_python_test(bindings_dynamics)
pinocchio_add_python_test(bindings_kinematics)
pinocchio_add_python_test(bindings_rnea)
pinocchio_add_python_test(bindings_aba)
pinocchio_add_python_test(bindings_joint_algorithms)
# Solvers
pinocchio_add_python_test(bindings_pgs)
pinocchio_add_python_test(bindings_admm)
# Algo derivatives
pinocchio_add_python_test(bindings_kinematics_derivatives)
pinocchio_add_python_test(bindings_frame_derivatives)
pinocchio_add_python_test(bindings_forward_dynamics_derivatives)
pinocchio_add_python_test(bindings_inverse_dynamics_derivatives)
pinocchio_add_python_test(bindings_centroidal_dynamics_derivatives)
pinocchio_add_python_test(bindings_com_velocity_derivatives)
# Parsers
pinocchio_add_python_test(bindings_sample_models)
pinocchio_add_python_test(bindings_mjcf)
pinocchio_add_python_test(bindings_model_graph)
# Others
pinocchio_add_python_test(robot_wrapper)
pinocchio_add_python_test(utils)
pinocchio_add_python_test(serialization)
pinocchio_add_python_test(version)
pinocchio_add_python_test(bindings_std_vector)
pinocchio_add_python_test(bindings_std_map)

if(BUILD_WITH_COLLISION_SUPPORT)
    pinocchio_add_python_test(bindings_geometry_object)
    pinocchio_add_python_test(bindings_coal_transform)
    pinocchio_add_python_test(bindings_constraints)
    pinocchio_add_python_test(bindings_lcaba)
endif()

if(urdfdom_FOUND)
    pinocchio_add_python_test(bindings_urdf)
    pinocchio_add_python_test(bindings_geometry_model_urdf)
    pinocchio_add_python_test(bindings_contact_inverse_dynamics)
    pinocchio_add_python_test(bindings_viser_visualizer)
endif(urdfdom_FOUND)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/serialization-data")

# handle casadi
if(BUILD_WITH_CASADI_SUPPORT)
    pinocchio_add_python_test(bindings_casadi_main_algo)
    pinocchio_add_python_test(bindings_casadi_explog)
    add_dependencies(pinocchio-tests pinocchio_pywrap_casadi)
endif()

# TODO BUILD_ADVANCED_TESTING

# TODO(jrl-v2 migration):
# add
# - add_python_memorycheck_unit_test_v2
# - add_windows_dll_path_to_test
if(MEMORYCHECK_COMMAND AND MEMORYCHECK_COMMAND MATCHES ".*valgrind$")
    set(pinocchio_PYTHON_MEMORYCHECK_TESTS
        bindings_build_geom_from_urdf_memorycheck
    )

    # foreach(TEST ${${PROJECT_NAME}_PYTHON_MEMORYCHECK_TESTS})
    #     set(TEST_NAME "${PROJECT_NAME}-test-py-memory-${TEST}")
    #     add_python_memorycheck_unit_test_v2(
    #       NAME
    #       ${TEST_NAME}
    #       SOURCE
    #       "unittest/python/${TEST}.py"
    #       SUPP
    #       "unittest/python/${TEST}.supp"
    #       MODULES
    #       "bindings/python"
    #     )
    #     get_test_property(${TEST_NAME} ENVIRONMENT _ENVIRONMENT)
    #     set_tests_properties(
    #         "${TEST_NAME}"
    #         PROPERTIES
    #             ENVIRONMENT
    #                 "${_ENVIRONMENT};EXAMPLE_ROBOT_DATA_MODEL_DIR=${EXAMPLE_ROBOT_DATA_MODEL_DIR}"
    #     )
    #     add_windows_dll_path_to_test(${TEST_NAME})
    # endforeach()
else()
    message(STATUS "Valgrind not found, memory checks are disabled")
endif()

function(pinocchio_add_python_cpp_module name)
    set(target_name "test-ext-${name}")
    string(REPLACE "_" "-" target_name ${target_name})
    string(REPLACE "-" "_" module_name ${target_name})
    set(options)
    set(oneValueArgs)
    set(multiValueArgs "PIN_TARGETS")
    cmake_parse_arguments(
        ARGS
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )

    if(BUILD_TESTING)
        set(_exclude)
    else()
        set(_exclude EXCLUDE_FROM_ALL)
    endif()

    python_add_library(
        ${target_name}
        MODULE
        ${_exclude}
        WITH_SOABI
        "${name}.cpp"
    )
    target_compile_definitions(
        ${target_name}
        PRIVATE EXT_MODULE_NAME=${module_name}
    )
    target_link_libraries(${target_name} PRIVATE pinocchio eigenpy::eigenpy)
    foreach(_dep ${ARGS_PIN_TARGETS})
        target_link_libraries(${target_name} PRIVATE ${_dep})
    endforeach()
    set_target_properties(
        ${target_name}
        PROPERTIES
            LIBRARY_OUTPUT_NAME ${module_name}
            RUNTIME_OUTPUT_NAME ${module_name}
    )

    add_test(
        NAME ${target_name}
        COMMAND $<TARGET_FILE:Python::Interpreter> -c "import ${module_name}"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${target_name}>
    )
    add_dependencies(pinocchio-tests ${target_name})
endfunction()

# TODO @jorisv: reenable these on windows when we normalize paths to lib/bin like on eigenpy
if(NOT WIN32 AND PINOCCHIO_BUILD_VISUALIZERS)
    pinocchio_add_python_cpp_module(bindings_visualizer PIN_TARGETS pinocchio_visualizers)
    pinocchio_add_python_test(bindings_visualizer)
    set_tests_properties(
        "pinocchio-test-py-bindings_visualizer"
        PROPERTIES
            ENVIRONMENT_MODIFICATION
                "PYTHONPATH=path_list_prepend:$<TARGET_FILE_DIR:test-ext-bindings-visualizer>"
    )
    add_subdirectory(pybind11)
endif()
