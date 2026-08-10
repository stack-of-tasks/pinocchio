#
# Copyright (c) 2015-2024 CNRS INRIA
#

add_custom_target(pinocchio-examples)


function(pinocchio_add_cpp_example example_name)
    set(target_name "pinocchio-example-cpp-${example_name}")
    pinocchio_add_cpp_test_or_example(${target_name} ${example_name})
    add_dependencies(pinocchio-examples ${target_name})

    if(MSVC)
        target_compile_definitions(${target_name} PRIVATE _USE_MATH_DEFINES)
    endif()
    if(WIN32)
        string(REPLACE ";" "\\\;" _PATH "$ENV{PATH}")
        set(ENV_VARIABLES
            "PATH=${_PATH}\\\;${PROJECT_BINARY_DIR}/src\\\;${PROJECT_BINARY_DIR}/bindings/python/pinocchio"
        )
        set_tests_properties(
            ${target_name}
            PROPERTIES ENVIRONMENT "${ENV_VARIABLES}"
        )
    endif()
    # modernize_target_link_libraries(
    #   ${EXAMPLE_NAME}
    #   SCOPE PRIVATE
    #   TARGETS Boost::filesystem
    #   LIBRARIES ${Boost_FILESYSTEM_LIBRARY}
    #   INCLUDE_DIRS ${Boost_INCLUDE_DIRS}
    # )
endfunction()

function(pinocchio_add_python_example example_name)
    pinocchio_add_python_test_or_example("${PROJECT_NAME}-example-py-${example_name}" "${example_name}")
    # TODO handle add_windows_dll_path_to_test
endfunction()

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
    pinocchio_add_cpp_example(inverse-kinematics)
    pinocchio_add_cpp_example(inverse-kinematics-3d)
    pinocchio_add_cpp_example(overview-simple)
    pinocchio_add_cpp_example(overview-lie)
    pinocchio_add_cpp_example(overview-SE3)
    pinocchio_add_cpp_example(interpolation-SE3)

    if(BUILD_WITH_PARSERS_SUPPORT)
        pinocchio_add_cpp_example(model-graph)
        pinocchio_add_cpp_example(model-configuration-converter)
    endif()

    if(BUILD_WITH_URDF_SUPPORT)
        pinocchio_add_cpp_example(overview-urdf)
        pinocchio_add_cpp_example(build-reduced-model)
        pinocchio_add_cpp_example(geometry-models)
        pinocchio_add_cpp_example(kinematics-derivatives)
        pinocchio_add_cpp_example(forward-dynamics-derivatives)
        pinocchio_add_cpp_example(inverse-dynamics-derivatives)
        if(BUILD_ADVANCED_TESTING)
            pinocchio_add_cpp_example(multiprecision)
        endif()
    endif()

    if(BUILD_WITH_COLLISION_SUPPORT AND BUILD_WITH_URDF_SUPPORT)
        pinocchio_add_cpp_example(collisions)
    endif()

    if(BUILD_WITH_CASADI_SUPPORT)
        function(pinocchio_add_casadi_example name)
            pinocchio_add_cpp_example(${name})
            add_dependencies(pinocchio-example-cpp-casadi pinocchio-example-cpp-${name})
            target_compile_definitions(
                pinocchio-example-cpp-${name}
                PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
            )
            target_link_libraries(
                pinocchio-example-cpp-${name}
                PRIVATE pinocchio_casadi_headers
            )
        endfunction()

        add_custom_target(pinocchio-example-cpp-casadi)
        set_target_properties(
            pinocchio-example-cpp-casadi
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )
        pinocchio_add_casadi_example(casadi-aba)
        pinocchio_add_casadi_example(casadi-crba)
        pinocchio_add_casadi_example(casadi-rnea)
    endif()

    if(BUILD_WITH_AUTODIFF_SUPPORT)
        function(pinocchio_add_cppad_example name)
            pinocchio_add_cpp_example(${name})
            add_dependencies(pinocchio-example-cpp-cppad pinocchio-example-cpp-${name})
            target_link_libraries(
                pinocchio-example-cpp-${name}
                PRIVATE pinocchio_cppad_headers
            )
        endfunction()
        add_custom_target(pinocchio-example-cpp-cppad)
        set_target_properties(
            pinocchio-example-cpp-cppad
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )
        pinocchio_add_cppad_example(autodiff-rnea)
    endif()

    if(BUILD_WITH_CODEGEN_SUPPORT AND BUILD_WITH_URDF_SUPPORT)
        function(pinocchio_add_cppadcg_example name)
            pinocchio_add_cpp_example(${name})
            add_dependencies(pinocchio-example-cpp-cppadcg pinocchio-example-cpp-${name})
            target_compile_definitions(
                pinocchio-example-cpp-${name}
                PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
            )
            
            target_link_libraries(
                pinocchio-example-cpp-${name}
                PRIVATE pinocchio_cppadcg_headers
            )

        endfunction()
        add_custom_target(pinocchio-example-cpp-cppadcg)
        set_target_properties(
            pinocchio-example-cpp-cppadcg
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )
        pinocchio_add_cppadcg_example(codegen-rnea)
        pinocchio_add_cppadcg_example(codegen-crba)        
    endif()
endif()

if(BUILD_PYTHON_INTERFACE)
        pinocchio_add_python_example(inverse-kinematics)
        pinocchio_add_python_example(inverse-kinematics-3d)
        pinocchio_add_python_example(overview-simple)
        pinocchio_add_python_example(kinematics-derivatives)
        pinocchio_add_python_example(forward-dynamics-derivatives)
        pinocchio_add_python_example(inverse-dynamics-derivatives)
        pinocchio_add_python_example(model-graph)
        pinocchio_add_python_example(model-configuration-converter)
        pinocchio_add_python_example(ellipsoid-joint-kinematics)
        pinocchio_add_python_example(admm-constraint-solver)
    

    if(BUILD_WITH_URDF_SUPPORT)
            pinocchio_add_python_example(overview-urdf)
            pinocchio_add_python_example(gepetto-viewer)
            pinocchio_add_python_example(build-reduced-model)
            pinocchio_add_python_example(robot-wrapper-viewer)
            pinocchio_add_python_example(geometry-models)
            pinocchio_add_python_example(constraint-cholesky)
        
        if(BUILD_ADVANCED_TESTING)
            # Set as advanced testing because it need a new
            # example_robot_data release (> 4.4)
            # and because computation is long in Debug

                pinocchio_add_python_example(g1-constraint-simulation)
        endif()
    endif()

    if( BUILD_WITH_CASADI_SUPPORT)
        pinocchio_add_python_example(quadrotor-ocp)
    endif()

    if(BUILD_WITH_AUTODIFF_SUPPORT)
        pinocchio_add_python_example(autodiff-rnea)
    endif()

    if(BUILD_WITH_CODEGEN_SUPPORT)
        pinocchio_add_python_example(codegen-rnea)
    endif()

    if(BUILD_WITH_COLLISION_SUPPORT)
        pinocchio_add_python_example(sample-model-viewer)
        pinocchio_add_python_example(display-shapes)
        pinocchio_add_python_example(simulation-pendulum)
        pinocchio_add_python_example(simulation-closed-kinematic-chains)
        pinocchio_add_python_example(simulation-lcaba)
        pinocchio_add_python_example(model-graph-geometry)
        
        if(BUILD_WITH_URDF_SUPPORT)
            pinocchio_add_python_example(meshcat-viewer)
            pinocchio_add_python_example(meshcat-viewer-dae)
            pinocchio_add_python_example(viser-viewer)
            pinocchio_add_python_example(collisions)
            pinocchio_add_python_example(collision-with-point-clouds)
            pinocchio_add_python_example(append-urdf-model-with-another-model)
            pinocchio_add_python_example(simulation-contact-dynamics)
            #pinocchio_add_python_example(talos-simulation)
            pinocchio_add_python_example(static-contact-dynamics)
        endif()

        if( BUILD_WITH_CASADI_SUPPORT)
            pinocchio_add_python_example(cartpole)
        endif()
    endif()

    if(BUILD_WITH_OPENMP_SUPPORT)
        pinocchio_add_python_example(run-algo-in-parallel)
    endif()
endif()
