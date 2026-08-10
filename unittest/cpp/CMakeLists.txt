# CPP tests
# Header only
add_pinocchio_unit_test(alloca)
add_pinocchio_unit_test(double-entry-container)
add_pinocchio_unit_test(macros)
add_pinocchio_unit_test(reference)
add_pinocchio_unit_test(eigen-storage)
add_pinocchio_unit_test(matrix-stack)
add_pinocchio_unit_test(size-in-bytes)

# Math components
add_pinocchio_unit_test(eigen-basic-op)
add_pinocchio_unit_test(matrix)
add_pinocchio_unit_test(block-diagonal-matrix)
add_pinocchio_unit_test(matrix-block-element)
add_pinocchio_unit_test(eigen-tensor)
add_pinocchio_unit_test(sincos)
add_pinocchio_unit_test(quaternion)
add_pinocchio_unit_test(rpy)
add_pinocchio_unit_test(rotation)
add_pinocchio_unit_test(vector)
add_pinocchio_unit_test(matrix-inverse)
add_pinocchio_unit_test(matrix-product)
add_pinocchio_unit_test(eigenvalues)
add_pinocchio_unit_test(tridiagonal-matrix)
add_pinocchio_unit_test(lanczos-decomposition)
add_pinocchio_unit_test(gram-schmidt-orthonormalisation)
add_pinocchio_unit_test(promote-static-eval)

# Derivatives algo
add_pinocchio_unit_test(kinematics-derivatives)
add_pinocchio_unit_test(frames-derivatives)
add_pinocchio_unit_test(rnea-derivatives)
add_pinocchio_unit_test(aba-derivatives)
add_pinocchio_unit_test(centroidal-derivatives)
add_pinocchio_unit_test(center-of-mass-derivatives)
add_pinocchio_unit_test(constrained-dynamics-derivatives)
add_pinocchio_unit_test(contact-dynamics-derivatives)
add_pinocchio_unit_test(impulse-dynamics-derivatives)
add_pinocchio_unit_test(rnea-second-order-derivatives)

# # Pinocchio features
add_pinocchio_unit_test(spatial)
add_pinocchio_unit_test(symmetric)
add_pinocchio_unit_test(aba)
add_pinocchio_unit_test(rnea)
add_pinocchio_unit_test(crba)
add_pinocchio_unit_test(centroidal)
add_pinocchio_unit_test(com)
add_pinocchio_unit_test(joint-jacobian)
add_pinocchio_unit_test(cholesky)
add_pinocchio_unit_test(constrained-dynamics)
add_pinocchio_unit_test(constraint-variants)
add_pinocchio_unit_test(contact-models)
add_pinocchio_unit_test(point-anchor-constraint)
add_pinocchio_unit_test(point-contact-constraint)
add_pinocchio_unit_test(frame-anchor-constraint)
add_pinocchio_unit_test(rigid-constraint-conversion)
add_pinocchio_unit_test(joint-friction-constraint)
add_pinocchio_unit_test(joint-limit-constraint)
add_pinocchio_unit_test(constraint-jacobian)
add_pinocchio_unit_test(contact-dynamics)
add_pinocchio_unit_test(contact-inverse-dynamics)
add_pinocchio_unit_test(closed-loop-dynamics)
add_pinocchio_unit_test(loop-constrained-aba)
add_pinocchio_unit_test(impulse-dynamics)
add_pinocchio_unit_test(sample-models)
add_pinocchio_unit_test(kinematics)
add_pinocchio_unit_test(delassus)
add_pinocchio_unit_test(delassus-operations)
add_pinocchio_unit_test(delassus-operator-dense)
# delassus-operator-preconditioned is broken and unused
#add_pinocchio_unit_test(delassus-operator-preconditioned)
add_pinocchio_unit_test(delassus-operator-rigid-body)
add_pinocchio_unit_test(preconditioner)
add_pinocchio_unit_test(pv-solver)

# parsers
if(BUILD_WITH_URDF_SUPPORT)
    add_pinocchio_unit_test(urdf)
    add_pinocchio_unit_test(value)
    if(BUILD_WITH_COLLISION_SUPPORT)
        add_pinocchio_unit_test(geometry-object)
        add_pinocchio_unit_test(
          geometry-model
        )
        add_pinocchio_unit_test(
          geometry-algorithms
        )
        add_pinocchio_unit_test(
          broadphase
        )
        add_pinocchio_unit_test(
          tree-broadphase
        )
        add_pinocchio_unit_test(
          srdf
        )
    endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
    add_pinocchio_unit_test(sdf)
endif()

if(BUILD_WITH_PARSERS_SUPPORT)
    add_pinocchio_unit_test(mjcf)
    add_pinocchio_unit_test(model-graph)
    add_pinocchio_unit_test(model-configuration-converter)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
    add_pinocchio_unit_test(
      reachable-workspace
    )
endif()

if(PINOCCHIO_BUILD_VISUALIZERS)
    add_pinocchio_unit_test(visualizer)
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
    add_pinocchio_unit_test(python_parser PYTHON_PARSER)
endif()

# Test over the joints
add_pinocchio_unit_test(all-joints)
add_pinocchio_unit_test(joint-revolute)
add_pinocchio_unit_test(joint-prismatic)
add_pinocchio_unit_test(joint-planar)
add_pinocchio_unit_test(joint-free-flyer)
add_pinocchio_unit_test(joint-spherical)
add_pinocchio_unit_test(joint-ellipsoid)
add_pinocchio_unit_test(joint-translation)
add_pinocchio_unit_test(joint-generic)
add_pinocchio_unit_test(joint-composite)
add_pinocchio_unit_test(joint-mimic)
add_pinocchio_unit_test(joint-helical)
add_pinocchio_unit_test(joint-universal)
add_pinocchio_unit_test(joint-visitors)

# Main corpus
add_pinocchio_unit_test(model)
add_pinocchio_unit_test(data)
add_pinocchio_unit_test(joint-motion-subspace)
add_pinocchio_unit_test(compute-all-terms)
add_pinocchio_unit_test(energy)
add_pinocchio_unit_test(frames)
if(NOT MSVC AND NOT MSVC_VERSION)
    add_pinocchio_unit_test(joint-configurations)
endif()
add_pinocchio_unit_test(explog)
add_pinocchio_unit_test(finite-differences)
add_pinocchio_unit_test(visitor)
add_pinocchio_unit_test(algo-check)

# Warning ignore with pragma doesn't work for a part of this test, so we ignore two warning globally
add_pinocchio_unit_test(liegroups)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(
        pinocchio-test-cpp-liegroups
        PRIVATE -Wno-maybe-uninitialized -Wuse-after-free=0
    )
endif()

add_pinocchio_unit_test(cartesian-product-liegroups)
add_pinocchio_unit_test(regressor)
add_pinocchio_unit_test(version)
add_pinocchio_unit_test(copy)
add_pinocchio_unit_test(constraint-cholesky)
add_pinocchio_unit_test(classic-acceleration)
add_pinocchio_unit_test(box-set)
add_pinocchio_unit_test(full-space-cone)
add_pinocchio_unit_test(zero-cone)
add_pinocchio_unit_test(orthant-cone)
add_pinocchio_unit_test(coulomb-friction-cone)

# Solvers
add_pinocchio_unit_test(pgs-solver)
add_pinocchio_unit_test(admm-solver)

# Symmetric cones jordan operations
add_pinocchio_unit_test(second-order-cone-jordan-operation)
add_pinocchio_unit_test(orthant-cone-jordan-operation)

# Serialization
add_pinocchio_unit_test(serialization)
add_pinocchio_unit_test(serialization-math)
add_pinocchio_unit_test(serialization-multibody)
add_pinocchio_unit_test(serialization-multibody-joint)
add_pinocchio_unit_test(serialization-algorithm)
add_pinocchio_unit_test(serialization-constraints)
add_pinocchio_unit_test(csv)

# algorithm
add_pinocchio_unit_test(algorithm-utils-motion)
add_pinocchio_unit_test(algorithm-utils-force)

# Multiprecision arithmetic
if(BUILD_ADVANCED_TESTING)
    add_pinocchio_unit_test(multiprecision)
    if(PINOCCHIO_BUILD_MPFR_TESTING)
        add_pinocchio_unit_test(multiprecision-mpfr)
        target_link_libraries(
            pinocchio-test-cpp-multiprecision-mpfr
            PRIVATE MPFR::MPFR
        )
    endif()
endif()

# special scalars
if(BUILD_WITH_AUTODIFF_SUPPORT)
    macro(ADD_CPPAD_UNIT_TEST name)
        add_pinocchio_unit_test(${name})
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
        add_pinocchio_unit_test(${name})
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
        add_pinocchio_unit_test(${name})
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
    add_pinocchio_unit_test(dynamic-allocations)
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
        add_pinocchio_unit_test(${name})
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
