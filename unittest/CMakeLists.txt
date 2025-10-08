#
# Copyright (c) 2015-2018 CNRS
# Copyright (c) 2018-2026 INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

function(ADD_TEST_CFLAGS target)
    if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
        get_cpp_test_name(${target} ${CMAKE_CURRENT_SOURCE_DIR} test_name)
        foreach(ARG ${ARGN})
            set_property(
                TARGET ${test_name}
                APPEND_STRING
                PROPERTY COMPILE_FLAGS "${ARG} "
            )
        endforeach()
    endif()
endfunction()

set(PINOCCHIO_UNIT_TEST_HEADERS
    ${PROJECT_SOURCE_DIR}/unittest/constraints/init_constraints.hpp
    ${PROJECT_SOURCE_DIR}/unittest/constraints/jacobians-checker.hpp
)

# Compute flags outside the macro to avoid recomputing it for each tests
cxx_flags_by_compiler_frontend(MSVC _USE_MATH_DEFINES OUTPUT TEST_PRIVATE_DEFINITIONS)

function(GET_CPP_TEST_NAME name src_dir full_test_name)
    string(REPLACE "${PINOCCHIO_UNIT_TEST_DIR}" "" prefix_name "${src_dir}")
    string(REGEX REPLACE "[/]" "-" prefix_name "${prefix_name}-")

    set(${full_test_name}
        "${PROJECT_NAME}-test-cpp${prefix_name}${name}"
        PARENT_SCOPE
    )
endfunction()

set(PINOCCHIO_UNIT_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

function(ADD_PINOCCHIO_UNIT_TEST name)
    if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
        set(options
            HEADER_ONLY
            PARSERS
            EXTRA
            COLLISION
            CPPAD
            PARALLEL
            VISUALIZERS
            PYTHON_PARSER
            PARSERS_OPTIONAL
            EXTRA_OPTIONAL
            COLLISION_OPTIONAL
            PARALLEL_OPTIONAL
        )
        set(oneValueArgs)
        set(multiValueArgs PACKAGES)
        cmake_parse_arguments(
            unit_test
            "${options}"
            "${oneValueArgs}"
            "${multiValueArgs}"
            ${ARGN}
        )

        set(PKGS ${unit_test_PACKAGES})

        get_cpp_test_name(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
        add_unit_test(${TEST_NAME} ${name}.cpp)

        set(MODULE_NAME "${NAME}Test")
        string(REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
        target_compile_definitions(
            ${TEST_NAME}
            PRIVATE
                ${TEST_PRIVATE_DEFINITIONS}
                BOOST_TEST_DYN_LINK
                BOOST_TEST_MODULE=${MODULE_NAME}
                PINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\"
        )

        # There is no RPATH in Windows, then we must use the PATH to find the DLL
        if(WIN32)
            string(REPLACE ";" "\\\;" _PATH "$ENV{PATH}")
            set(ENV_VARIABLES
                "PATH=${_PATH}\\\;${PROJECT_BINARY_DIR}/src\\\;${PROJECT_BINARY_DIR}/bindings/python/pinocchio"
            )
            set_tests_properties(
                ${TEST_NAME}
                PROPERTIES ENVIRONMENT "${ENV_VARIABLES}"
            )
        endif()

        set_target_properties(${TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)
        target_include_directories(
            ${TEST_NAME}
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        )

        if(unit_test_HEADER_ONLY)
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_headers)
        else()
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_default)
        endif()

        if(
            unit_test_PARSERS
            OR (unit_test_PARSERS_OPTIONAL AND TARGET ${PROJECT_NAME}_parsers)
        )
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_parsers)
        endif()

        if(
            unit_test_PARALLEL
            OR (unit_test_PARALLEL_OPTIONAL AND TARGET ${PROJECT_NAME}_parallel)
        )
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_parallel)
        endif()

        if(
            unit_test_COLLISION
            OR (unit_test_COLLISION_OPTIONAL AND TARGET ${PROJECT_NAME}_collision)
        )
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_collision)
            if(
                unit_test_PARALLE
                OR (
                    unit_test_PARALLEL_OPTIONAL
                    AND TARGET ${PROJECT_NAME}_parallel
                )
            )
                target_link_libraries(
                    ${TEST_NAME}
                    PUBLIC ${PROJECT_NAME}_collision_parallel
                )
            endif()
        endif()

        if(unit_test_CPPAD)
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_cppad)
        endif()

        if(
            unit_test_EXTRA
            OR (unit_test_EXTRA_OPTIONAL AND TARGET ${PROJECT_NAME}_extra)
        )
            target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_extra)
        endif()

        if(unit_test_VISUALIZERS)
            target_link_libraries(
                ${TEST_NAME}
                PUBLIC ${PROJECT_NAME}_visualizers
            )
        endif()

        if(unit_test_PYTHON_PARSER)
            target_link_libraries(
                ${TEST_NAME}
                PUBLIC ${PROJECT_NAME}_python_parser
            )
            add_windows_dll_path_to_test(${TEST_NAME})
            get_test_property(${TEST_NAME} ENVIRONMENT ENV_VARIABLES)
            compute_pythonpath(PYTHON_ENV_VARIABLES "bindings/python")
            list(APPEND ENV_VARIABLES ${PYTHON_ENV_VARIABLES})
            if(WIN32)
                # This line is mandatory because of Github action. The test run well on Windows + Conda.
                # This hide something wrong. Maybe the test is linking against the wrong Python library or
                # call the wrong interpreter.
                get_filename_component(_PYTHONHOME ${PYTHON_EXECUTABLE} PATH)
                list(APPEND ENV_VARIABLES "PYTHONHOME=${_PYTHONHOME}")
            endif()
            set_tests_properties(
                ${TEST_NAME}
                PROPERTIES ENVIRONMENT "${ENV_VARIABLES}"
            )
        endif()

        modernize_target_link_libraries(
          ${TEST_NAME}
          SCOPE PRIVATE
          TARGETS Boost::unit_test_framework
          LIBRARIES ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
        )

        modernize_target_link_libraries(
          ${TEST_NAME}
          SCOPE PRIVATE
          TARGETS Boost::filesystem
          LIBRARIES ${Boost_FILESYSTEM_LIBRARY}
          INCLUDE_DIRS ${Boost_INCLUDE_DIRS}
        )

        if(PKGS)
            target_link_libraries(${TEST_NAME} PRIVATE ${PKGS})
        endif()
        if(TARGET example-robot-data::example-robot-data)
            target_link_libraries(
                ${TEST_NAME}
                PRIVATE example-robot-data::example-robot-data
            )
        else()
            target_compile_definitions(
                ${TEST_NAME}
                PRIVATE
                    "EXAMPLE_ROBOT_DATA_MODEL_DIR=\"${EXAMPLE_ROBOT_DATA_MODEL_DIR}\""
            )
        endif()
    endif()
endfunction()

macro(ADD_PINOCCHIO_PARALLEL_UNIT_TEST NAME)
    if(BUILD_WITH_OPENMP_SUPPORT)
        add_pinocchio_unit_test(${ARGV} PARALLEL)
    endif()
endmacro()

# Find Boost.UnitTestFramework
find_package(Boost COMPONENTS unit_test_framework)

# Header only
add_pinocchio_unit_test(alloca HEADER_ONLY)
add_pinocchio_unit_test(double-entry-container HEADER_ONLY)
add_pinocchio_unit_test(macros HEADER_ONLY)
add_pinocchio_unit_test(reference HEADER_ONLY)
add_pinocchio_unit_test(eigen-storage HEADER_ONLY)
add_pinocchio_unit_test(matrix-stack HEADER_ONLY)
add_pinocchio_unit_test(size-in-bytes HEADER_ONLY)

# Math components
add_pinocchio_unit_test(eigen-basic-op HEADER_ONLY)
add_pinocchio_unit_test(matrix HEADER_ONLY)
add_pinocchio_unit_test(block-diagonal-matrix HEADER_ONLY)
add_pinocchio_unit_test(matrix-block-element HEADER_ONLY)
add_pinocchio_unit_test(eigen-tensor HEADER_ONLY)
add_pinocchio_unit_test(sincos HEADER_ONLY)
add_pinocchio_unit_test(quaternion HEADER_ONLY)
add_pinocchio_unit_test(rpy HEADER_ONLY)
add_pinocchio_unit_test(rotation HEADER_ONLY)
add_pinocchio_unit_test(vector HEADER_ONLY)
add_pinocchio_unit_test(matrix-inverse HEADER_ONLY)
add_pinocchio_unit_test(matrix-product HEADER_ONLY)
add_pinocchio_unit_test(eigenvalues HEADER_ONLY)
add_pinocchio_unit_test(tridiagonal-matrix HEADER_ONLY)
add_pinocchio_unit_test(lanczos-decomposition HEADER_ONLY)
add_pinocchio_unit_test(gram-schmidt-orthonormalisation HEADER_ONLY)
add_pinocchio_unit_test(promote-static-eval HEADER_ONLY)

# Derivatives algo
add_pinocchio_unit_test(kinematics-derivatives)
add_pinocchio_unit_test(frames-derivatives)
add_pinocchio_unit_test(rnea-derivatives)
add_pinocchio_unit_test(aba-derivatives)
add_pinocchio_unit_test(centroidal-derivatives)
add_pinocchio_unit_test(center-of-mass-derivatives)
add_pinocchio_unit_test(constrained-dynamics-derivatives)

# if(BUILD_WITH_SDF_SUPPORT)
#   add_pinocchio_unit_test(contact-dynamics-derivatives PARSERS)
# else()
add_pinocchio_unit_test(contact-dynamics-derivatives)
# endif()
add_pinocchio_unit_test(impulse-dynamics-derivatives)
add_pinocchio_unit_test(rnea-second-order-derivatives)

# # Pinocchio features
add_pinocchio_unit_test(spatial)
add_pinocchio_unit_test(symmetric)
add_pinocchio_unit_test(aba)
add_pinocchio_parallel_unit_test(parallel-aba)
add_pinocchio_unit_test(rnea)
add_pinocchio_parallel_unit_test(parallel-rnea)
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
add_pinocchio_unit_test(sample-models COLLISION_OPTIONAL)
add_pinocchio_unit_test(kinematics)
add_pinocchio_unit_test(delassus)
add_pinocchio_unit_test(delassus-operations)
add_pinocchio_unit_test(delassus-operator-dense)
# delassus-operator-preconditioned is broken and unused
#add_pinocchio_unit_test(delassus-operator-preconditioned)
add_pinocchio_unit_test(delassus-operator-rigid-body)
add_pinocchio_unit_test(preconditioner)
add_pinocchio_unit_test(pv-solver)
add_pinocchio_parallel_unit_test(openmp-exception)

add_pinocchio_unit_test(
  mjcf
  PARSERS
  COLLISION_OPTIONAL
)

if(BUILD_WITH_URDF_SUPPORT)
    add_pinocchio_unit_test(
      urdf
      PARSERS
      COLLISION_OPTIONAL
    )

    add_pinocchio_unit_test(value PARSERS)
    if(BUILD_WITH_COLLISION_SUPPORT)
        add_pinocchio_unit_test(geometry-object COLLISION)
        add_pinocchio_unit_test(
          geometry-model
          PARSERS
          COLLISION
        )
        add_pinocchio_unit_test(
          geometry-algorithms
          PARSERS
          COLLISION
        )
        add_pinocchio_unit_test(
          broadphase
          PARSERS
          COLLISION
        )
        add_pinocchio_unit_test(
          tree-broadphase
          PARSERS
          COLLISION
        )
        add_pinocchio_parallel_unit_test(parallel-geometry PARSERS COLLISION)
        add_pinocchio_unit_test(
          srdf
          PARSERS
          COLLISION
        )
    endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
    add_pinocchio_unit_test(sdf PARSERS)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
    add_pinocchio_unit_test(
      reachable-workspace
      PARSERS
      EXTRA
      COLLISION_OPTIONAL
    )
endif()

if(PINOCCHIO_BUILD_VISUALIZERS)
    add_pinocchio_unit_test(visualizer VISUALIZERS)
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
    add_pinocchio_unit_test(python_parser PYTHON_PARSER)
endif()

if(BUILD_PYTHON_INTERFACE)
    add_subdirectory(python)
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
add_pinocchio_unit_test(joint-spline)

# Main corpus
add_pinocchio_unit_test(model COLLISION_OPTIONAL)
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
add_pinocchio_unit_test(model-graph PARSERS)
add_pinocchio_unit_test(model-configuration-converter PARSERS)

# Warning ignore with pragma doesn't work for a part of this test, so we ignore two warning globally
cxx_flags_by_compiler_frontend(
  GNU -Wno-maybe-uninitialized -Wuse-after-free=0
  OUTPUT LIEGROUPS_OPTIONS
  FILTER
)
add_pinocchio_unit_test(liegroups)
add_test_cflags(liegroups ${LIEGROUPS_OPTIONS})

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
make_directory("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")
add_pinocchio_unit_test(serialization)
add_test_cflags(
  serialization
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
add_pinocchio_unit_test(serialization-math)
add_test_cflags(
  serialization-math
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
add_pinocchio_unit_test(serialization-multibody COLLISION_OPTIONAL)
add_test_cflags(
  serialization-multibody
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
add_pinocchio_unit_test(serialization-multibody-joint)
add_test_cflags(
  serialization-multibody-joint
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
add_pinocchio_unit_test(serialization-algorithm)
add_test_cflags(
  serialization-algorithm
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
add_pinocchio_unit_test(serialization-constraints)
add_test_cflags(
  serialization-constraints
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)

add_pinocchio_unit_test(csv)
add_test_cflags(
  csv "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)

add_subdirectory(algorithm)

# Multiprecision arithmetic
if(BUILD_ADVANCED_TESTING)
    add_pinocchio_unit_test(multiprecision)
    if(PINOCCHIO_BUILD_MPFR_TESTING)
        add_pinocchio_unit_test(multiprecision-mpfr PACKAGES mpfr gmp)
    endif()
endif()

if(BUILD_WITH_AUTODIFF_SUPPORT)
    add_subdirectory(cppad)
endif()

if(BUILD_WITH_CODEGEN_SUPPORT)
    add_subdirectory(cppadcg)
endif()

if(BUILD_WITH_CASADI_SUPPORT)
    add_subdirectory(casadi)
endif()

if(BUILD_WITH_RTSAN)
    add_pinocchio_unit_test(dynamic-allocations)
    get_cpp_test_name(dynamic-allocations ${CMAKE_CURRENT_SOURCE_DIR} test_name)
    target_compile_options(${test_name} PRIVATE -fsanitize=realtime)
    target_link_options(${test_name} PRIVATE -fsanitize=realtime)
    # On osx-arm64 mutex are called to initialize some static variables.
    # We use a suppression file to make this test pass.
    if(APPLE AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set_tests_properties(
            ${test_name}
            PROPERTIES
                ENVIRONMENT_MODIFICATION
                    "RTSAN_OPTIONS=path_list_append:suppressions=${CMAKE_CURRENT_SOURCE_DIR}/dynamic-allocations-osx-arm64.supp"
        )
    endif()
endif()
