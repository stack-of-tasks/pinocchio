# CPP tests
# Header only
pinocchio_add_cpp_test(alloca)
pinocchio_add_cpp_test(double-entry-container)
pinocchio_add_cpp_test(macros)
pinocchio_add_cpp_test(reference)
pinocchio_add_cpp_test(eigen-storage)
pinocchio_add_cpp_test(matrix-stack)
pinocchio_add_cpp_test(size-in-bytes)

# Math components
pinocchio_add_cpp_test(eigen-basic-op)
pinocchio_add_cpp_test(matrix)
pinocchio_add_cpp_test(block-diagonal-matrix)
pinocchio_add_cpp_test(matrix-block-element)
pinocchio_add_cpp_test(eigen-tensor)
pinocchio_add_cpp_test(sincos)
pinocchio_add_cpp_test(quaternion)
pinocchio_add_cpp_test(rpy)
pinocchio_add_cpp_test(rotation)
pinocchio_add_cpp_test(vector)
pinocchio_add_cpp_test(matrix-inverse)
pinocchio_add_cpp_test(matrix-product)
pinocchio_add_cpp_test(eigenvalues)
pinocchio_add_cpp_test(tridiagonal-matrix)
pinocchio_add_cpp_test(lanczos-decomposition)
pinocchio_add_cpp_test(gram-schmidt-orthonormalisation)
pinocchio_add_cpp_test(promote-static-eval)

# Derivatives algo
pinocchio_add_cpp_test(kinematics-derivatives)
pinocchio_add_cpp_test(frames-derivatives)
pinocchio_add_cpp_test(rnea-derivatives)
pinocchio_add_cpp_test(aba-derivatives)
pinocchio_add_cpp_test(centroidal-derivatives)
pinocchio_add_cpp_test(center-of-mass-derivatives)
pinocchio_add_cpp_test(constrained-dynamics-derivatives)
pinocchio_add_cpp_test(contact-dynamics-derivatives)
pinocchio_add_cpp_test(impulse-dynamics-derivatives)
pinocchio_add_cpp_test(rnea-second-order-derivatives)

# # Pinocchio features
pinocchio_add_cpp_test(spatial)
pinocchio_add_cpp_test(symmetric)
pinocchio_add_cpp_test(aba)
pinocchio_add_cpp_test(rnea)
pinocchio_add_cpp_test(crba)
pinocchio_add_cpp_test(centroidal)
pinocchio_add_cpp_test(com)
pinocchio_add_cpp_test(joint-jacobian)
pinocchio_add_cpp_test(cholesky)
pinocchio_add_cpp_test(constrained-dynamics)
pinocchio_add_cpp_test(constraint-variants)
pinocchio_add_cpp_test(contact-models)
pinocchio_add_cpp_test(point-anchor-constraint)
pinocchio_add_cpp_test(point-contact-constraint)
pinocchio_add_cpp_test(frame-anchor-constraint)
pinocchio_add_cpp_test(rigid-constraint-conversion)
pinocchio_add_cpp_test(joint-friction-constraint)
pinocchio_add_cpp_test(joint-limit-constraint)
pinocchio_add_cpp_test(constraint-jacobian)
pinocchio_add_cpp_test(contact-dynamics)
pinocchio_add_cpp_test(contact-inverse-dynamics)
pinocchio_add_cpp_test(closed-loop-dynamics)
pinocchio_add_cpp_test(loop-constrained-aba)
pinocchio_add_cpp_test(impulse-dynamics)
pinocchio_add_cpp_test(sample-models)
pinocchio_add_cpp_test(kinematics)
pinocchio_add_cpp_test(delassus)
pinocchio_add_cpp_test(delassus-operations)
pinocchio_add_cpp_test(delassus-operator-dense)
# delassus-operator-preconditioned is broken and unused
#pinocchio_add_cpp_test(delassus-operator-preconditioned)
pinocchio_add_cpp_test(delassus-operator-rigid-body)
pinocchio_add_cpp_test(preconditioner)
pinocchio_add_cpp_test(pv-solver)

# parsers
if(BUILD_WITH_URDF_SUPPORT)
    pinocchio_add_cpp_test(urdf)
    pinocchio_add_cpp_test(value)
    if(BUILD_WITH_COLLISION_SUPPORT)
        pinocchio_add_cpp_test(geometry-object)
        pinocchio_add_cpp_test(
          geometry-model
        )
        pinocchio_add_cpp_test(
          geometry-algorithms
        )
        pinocchio_add_cpp_test(
          broadphase
        )
        pinocchio_add_cpp_test(
          tree-broadphase
        )
        pinocchio_add_cpp_test(
          srdf
        )
    endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
    pinocchio_add_cpp_test(sdf)
endif()

if(BUILD_WITH_PARSERS_SUPPORT)
    pinocchio_add_cpp_test(mjcf)
    pinocchio_add_cpp_test(model-graph)
    pinocchio_add_cpp_test(model-configuration-converter)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
    pinocchio_add_cpp_test(
      reachable-workspace
    )
endif()

if(PINOCCHIO_BUILD_VISUALIZERS)
    pinocchio_add_cpp_test(visualizer)
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
    pinocchio_add_cpp_test(python_parser PYTHON_PARSER)
endif()

# Test over the joints
pinocchio_add_cpp_test(all-joints)
pinocchio_add_cpp_test(joint-revolute)
pinocchio_add_cpp_test(joint-prismatic)
pinocchio_add_cpp_test(joint-planar)
pinocchio_add_cpp_test(joint-free-flyer)
pinocchio_add_cpp_test(joint-spherical)
pinocchio_add_cpp_test(joint-ellipsoid)
pinocchio_add_cpp_test(joint-translation)
pinocchio_add_cpp_test(joint-generic)
pinocchio_add_cpp_test(joint-composite)
pinocchio_add_cpp_test(joint-mimic)
pinocchio_add_cpp_test(joint-helical)
pinocchio_add_cpp_test(joint-universal)
pinocchio_add_cpp_test(joint-visitors)

# Main corpus
pinocchio_add_cpp_test(model)
pinocchio_add_cpp_test(data)
pinocchio_add_cpp_test(joint-motion-subspace)
pinocchio_add_cpp_test(compute-all-terms)
pinocchio_add_cpp_test(energy)
pinocchio_add_cpp_test(frames)
if(NOT MSVC AND NOT MSVC_VERSION)
    pinocchio_add_cpp_test(joint-configurations)
endif()
pinocchio_add_cpp_test(explog)
pinocchio_add_cpp_test(finite-differences)
pinocchio_add_cpp_test(visitor)
pinocchio_add_cpp_test(algo-check)

# Warning ignore with pragma doesn't work for a part of this test, so we ignore two warning globally
pinocchio_add_cpp_test(liegroups)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(
        pinocchio-test-cpp-liegroups
        PRIVATE -Wno-maybe-uninitialized -Wuse-after-free=0
    )
endif()

pinocchio_add_cpp_test(cartesian-product-liegroups)
pinocchio_add_cpp_test(regressor)
pinocchio_add_cpp_test(version)
pinocchio_add_cpp_test(copy)
pinocchio_add_cpp_test(constraint-cholesky)
pinocchio_add_cpp_test(classic-acceleration)
pinocchio_add_cpp_test(box-set)
pinocchio_add_cpp_test(full-space-cone)
pinocchio_add_cpp_test(zero-cone)
pinocchio_add_cpp_test(orthant-cone)
pinocchio_add_cpp_test(coulomb-friction-cone)

# Solvers
pinocchio_add_cpp_test(pgs-solver)
pinocchio_add_cpp_test(admm-solver)

# Symmetric cones jordan operations
pinocchio_add_cpp_test(second-order-cone-jordan-operation)
pinocchio_add_cpp_test(orthant-cone-jordan-operation)

# Serialization
pinocchio_add_cpp_test(serialization)
pinocchio_add_cpp_test(serialization-math)
pinocchio_add_cpp_test(serialization-multibody)
pinocchio_add_cpp_test(serialization-multibody-joint)
pinocchio_add_cpp_test(serialization-algorithm)
pinocchio_add_cpp_test(serialization-constraints)
pinocchio_add_cpp_test(csv)

# algorithm
pinocchio_add_cpp_test(algorithm-utils-motion)
pinocchio_add_cpp_test(algorithm-utils-force)

# Multiprecision arithmetic
if(BUILD_ADVANCED_TESTING)
    pinocchio_add_cpp_test(multiprecision)
    if(PINOCCHIO_BUILD_MPFR_TESTING)
        pinocchio_add_cpp_test(multiprecision-mpfr)
        target_link_libraries(
            pinocchio-test-cpp-multiprecision-mpfr
            PRIVATE MPFR::MPFR
        )
    endif()
endif()

# special scalars
if(BUILD_WITH_AUTODIFF_SUPPORT)
    macro(ADD_CPPAD_UNIT_TEST name)
        pinocchio_add_cpp_test(${name})
        add_dependencies(pinocchio-test-cpp-cppad pinocchio-test-cpp-${name})
        target_link_libraries(
            pinocchio-test-cpp-${name}
            PRIVATE pinocchio_cppad_headers
        )
    endmacro()

    if(BUILD_WITH_AUTODIFF_SUPPORT AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
        add_custom_target(pinocchio-test-cpp-cppad)
        set_target_properties(
            pinocchio-test-cpp-cppad
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )
        add_cppad_unit_test(cppad-basic)
        add_cppad_unit_test(cppad-spatial)
        add_cppad_unit_test(cppad-joints)
        add_cppad_unit_test(cppad-algorithms)
        add_cppad_unit_test(cppad-derivatives)
        add_cppad_unit_test(cppad-joint-configurations)
    endif()
endif()

if(BUILD_WITH_CODEGEN_SUPPORT)
    macro(ADD_CPPADCG_UNIT_TEST name)
        pinocchio_add_cpp_test(${name})
        add_dependencies(pinocchio-test-cpp-cppadcg pinocchio-test-cpp-${name})
        target_compile_definitions(
            pinocchio-test-cpp-${name}
            PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
        )
        target_link_libraries(
            pinocchio-test-cpp-${name}
            PRIVATE pinocchio_cppadcg_headers
        )
    endmacro()
    if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
        add_custom_target(pinocchio-test-cpp-cppadcg)
        set_target_properties(
            pinocchio-test-cpp-cppadcg
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )
        add_cppadcg_unit_test(cppadcg-basic)
        if(BUILD_ADVANCED_TESTING)
            add_cppadcg_unit_test(cppadcg-algorithms)
            add_cppadcg_unit_test(cppadcg-joint-configurations)
            add_cppadcg_unit_test(cppadcg-contact-dynamics)
        endif()
    endif()
endif()

if(BUILD_WITH_CASADI_SUPPORT)
    macro(ADD_CASADI_UNIT_TEST name)
        pinocchio_add_cpp_test(${name})
        add_dependencies(pinocchio-test-cpp-casadi pinocchio-test-cpp-${name})
        target_compile_definitions(
            pinocchio-test-cpp-${name}
            PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
        )
        target_link_libraries(
            pinocchio-test-cpp-${name}
            PRIVATE pinocchio_casadi_headers
        )
    endmacro()

    if(BUILD_WITH_CASADI_SUPPORT AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
        add_custom_target(pinocchio-test-cpp-casadi)
        set_target_properties(
            pinocchio-test-cpp-casadi
            PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
        )

        add_casadi_unit_test(casadi-basic)
        add_casadi_unit_test(casadi-spatial)
        add_casadi_unit_test(casadi-explog)
        add_casadi_unit_test(casadi-joints)
        add_casadi_unit_test(casadi-algorithms)
        if(BUILD_ADVANCED_TESTING)
            add_casadi_unit_test(casadi-aba)
            add_casadi_unit_test(casadi-constraint-dynamics-derivatives)
            add_casadi_unit_test(casadi-constraint-dynamics)
            add_casadi_unit_test(casadi-aba-derivatives)
        endif()
        add_casadi_unit_test(casadi-integrate-derivatives)
        add_casadi_unit_test(casadi-rnea-derivatives)
    endif()
endif()

# rtsan
if(BUILD_WITH_RTSAN)
    pinocchio_add_cpp_test(dynamic-allocations)
    target_compile_options(
        pinocchio-test-cpp-dynamic-allocations
        PRIVATE -fsanitize=realtime
    )
    target_link_options(
        pinocchio-test-cpp-dynamic-allocations
        PRIVATE -fsanitize=realtime
    )
    # On osx-arm64 mutex are called to initialize some static variables.
    # We use a suppression file to make this test pass.
    if(APPLE AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set_tests_properties(
            pinocchio-test-cpp-dynamic-allocations
            PROPERTIES
                ENVIRONMENT_MODIFICATION
                    "RTSAN_OPTIONS=path_list_append:suppressions=${CMAKE_CURRENT_SOURCE_DIR}/dynamic-allocations-osx-arm64.supp"
        )
    endif()
endif()

if(BUILD_WITH_OPENMP_SUPPORT)
    macro(add_pinocchio_parallel_unit_test name)
        pinocchio_add_cpp_test(${name})
        target_link_libraries(
            pinocchio-test-cpp-${name}
            PUBLIC pinocchio_parallel
        )
    endmacro()

    add_pinocchio_parallel_unit_test(parallel-aba)
    add_pinocchio_parallel_unit_test(openmp-exception)
    add_pinocchio_parallel_unit_test(parallel-rnea)

    if(BUILD_WITH_URDF_SUPPORT AND BUILD_WITH_COLLISION_SUPPORT)
        add_pinocchio_parallel_unit_test(parallel-geometry)
    endif()
endif()
