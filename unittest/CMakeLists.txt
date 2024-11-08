#
# Copyright (c) 2015-2024 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

function(ADD_TEST_CFLAGS target)
  get_cpp_test_name(${target} ${CMAKE_CURRENT_SOURCE_DIR} test_name)
  foreach(ARG ${ARGN})
    set_property(
      TARGET ${test_name}
      APPEND_STRING
      PROPERTY COMPILE_FLAGS "${ARG} ")
  endforeach()
endfunction()

# Compute flags outside the macro to avoid recomputing it for each tests
cxx_flags_by_compiler_frontend(MSVC _USE_MATH_DEFINES OUTPUT TEST_PRIVATE_DEFINITIONS)

function(GET_CPP_TEST_NAME name src_dir full_test_name)
  string(REPLACE "${PINOCCHIO_UNIT_TEST_DIR}" "" prefix_name "${src_dir}")
  string(REGEX REPLACE "[/]" "-" prefix_name "${prefix_name}-")

  set(${full_test_name}
      "${PROJECT_NAME}-test-cpp${prefix_name}${name}"
      PARENT_SCOPE)
endfunction()

set(PINOCCHIO_UNIT_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

function(ADD_PINOCCHIO_UNIT_TEST name)
  set(options
      HEADER_ONLY
      PARSERS
      EXTRA
      COLLISION
      PARALLEL
      PYTHON_PARSER
      PARSERS_OPTIONAL
      EXTRA_OPTIONAL
      COLLISION_OPTIONAL
      PARALLEL_OPTIONAL)
  set(oneValueArgs)
  set(multiValueArgs PACKAGES)
  cmake_parse_arguments(unit_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PKGS ${unit_test_PACKAGES})

  get_cpp_test_name(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  add_unit_test(${TEST_NAME} ${name}.cpp)

  set(MODULE_NAME "${NAME}Test")
  string(REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
  target_compile_definitions(
    ${TEST_NAME}
    PRIVATE ${TEST_PRIVATE_DEFINITIONS} BOOST_TEST_DYN_LINK BOOST_TEST_MODULE=${MODULE_NAME}
            PINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\")

  # There is no RPATH in Windows, then we must use the PATH to find the DLL
  if(WIN32)
    string(REPLACE ";" "\\\;" _PATH "$ENV{PATH}")
    set(ENV_VARIABLES
        "PATH=${_PATH}\\\;${PROJECT_BINARY_DIR}/src\\\;${PROJECT_BINARY_DIR}/bindings/python/pinocchio"
    )
    set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
  endif()

  set_target_properties(${TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_include_directories(${TEST_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  if(NOT unit_test_HEADER_ONLY)
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_default)
  endif()

  if(unit_test_PARSERS OR (unit_test_PARSERS_OPTIONAL AND TARGET ${PROJECT_NAME}_parsers))
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_parsers)
  endif()

  if(unit_test_PARALLEL OR (unit_test_PARALLEL_OPTIONAL AND TARGET ${PROJECT_NAME}_parallel))
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_parallel)
  endif()

  if(unit_test_COLLISION OR (unit_test_COLLISION_OPTIONAL AND TARGET ${PROJECT_NAME}_collision))
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_collision)
    if(unit_test_PARALLE OR (unit_test_PARALLEL_OPTIONAL AND TARGET ${PROJECT_NAME}_parallel))
      target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_collision_parallel)
    endif()
  endif()

  if(unit_test_EXTRA OR (unit_test_EXTRA_OPTIONAL AND TARGET ${PROJECT_NAME}_extra))
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_extra)
  endif()

  if(unit_test_PYTHON_PARSER)
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME}_python_parser)
  endif()

  modernize_target_link_libraries(
    ${TEST_NAME}
    SCOPE PRIVATE
    TARGETS Boost::unit_test_framework
    LIBRARIES ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

  if(PKGS)
    target_link_libraries(${TEST_NAME} PRIVATE ${PKGS})
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
add_pinocchio_unit_test(macros HEADER_ONLY)

# Math components
add_pinocchio_unit_test(eigen-basic-op)
add_pinocchio_unit_test(eigen-tensor)
add_pinocchio_unit_test(sincos)
add_pinocchio_unit_test(quaternion)
add_pinocchio_unit_test(rpy)
add_pinocchio_unit_test(rotation)
add_pinocchio_unit_test(vector)
add_pinocchio_unit_test(eigenvalues)
add_pinocchio_unit_test(tridiagonal-matrix)
# TOOD This test contains an undefined behavior and is not stable. Reactivte the test when the fix
# is merged.

# add_pinocchio_unit_test(lanczos-decomposition)
add_pinocchio_unit_test(gram-schmidt-orthonormalisation)

# Derivatives algo
add_pinocchio_unit_test(kinematics-derivatives)
add_pinocchio_unit_test(frames-derivatives)
add_pinocchio_unit_test(rnea-derivatives)
add_pinocchio_unit_test(aba-derivatives)
add_pinocchio_unit_test(centroidal-derivatives)
add_pinocchio_unit_test(center-of-mass-derivatives)
add_pinocchio_unit_test(constrained-dynamics-derivatives)
add_pinocchio_unit_test(rnea-second-order-derivatives)

# Pinocchio features
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
add_pinocchio_unit_test(contact-models)
add_pinocchio_unit_test(contact-dynamics)
add_pinocchio_unit_test(contact-inverse-dynamics)
add_pinocchio_unit_test(closed-loop-dynamics)
add_pinocchio_unit_test(sample-models COLLISION_OPTIONAL)
add_pinocchio_unit_test(kinematics)
# Test for unstable features TODO fix and reactivate these tests in next version
# ADD_PINOCCHIO_UNIT_TEST(delassus) ADD_PINOCCHIO_UNIT_TEST(pv-solver)
add_pinocchio_unit_test(impulse-dynamics-derivatives)
if(BUILD_WITH_SDF_SUPPORT)
  add_pinocchio_unit_test(contact-dynamics-derivatives PARSERS)
else()
  add_pinocchio_unit_test(contact-dynamics-derivatives)
endif()
add_pinocchio_unit_test(constraint-variants)
add_pinocchio_unit_test(impulse-dynamics)

add_pinocchio_unit_test(
  mjcf
  PARSERS
  COLLISION_OPTIONAL)

if(BUILD_WITH_URDF_SUPPORT)
  add_pinocchio_unit_test(
    urdf
    PARSERS
    COLLISION_OPTIONAL)

  add_pinocchio_unit_test(value PARSERS)
  if(BUILD_WITH_HPP_FCL_SUPPORT)
    add_pinocchio_unit_test(geometry-object COLLISION)
    add_pinocchio_unit_test(
      geometry-model
      PARSERS
      COLLISION)
    add_pinocchio_unit_test(
      geometry-algorithms
      PARSERS
      COLLISION)
    add_pinocchio_unit_test(
      broadphase
      PARSERS
      COLLISION)
    add_pinocchio_unit_test(
      tree-broadphase
      PARSERS
      COLLISION)
    add_pinocchio_parallel_unit_test(parallel-geometry PARSERS COLLISION)
    add_pinocchio_unit_test(
      srdf
      PARSERS
      COLLISION)
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
    COLLISION_OPTIONAL)
endif()

if(BUILD_WITH_LIBPYTHON)
  add_pinocchio_unit_test(python_parser PYTHON_PARSER)
  get_cpp_test_name(python_parser ${CMAKE_CURRENT_SOURCE_DIR} python_parser_target)

  get_test_property(${python_parser_target} ENVIRONMENT ENV_VARIABLES)
  compute_pythonpath(PYTHON_ENV_VARIABLES "bindings/python")
  list(APPEND ENV_VARIABLES ${PYTHON_ENV_VARIABLES})
  if(WIN32)
    # This line is mandatory because of Github action. The test run well on Windows + Conda. This
    # hide something wrong. Maybe the test is linking against the wrong Python library or call the
    # wrong interpreter.
    get_filename_component(_PYTHONHOME ${PYTHON_EXECUTABLE} PATH)
    list(APPEND ENV_VARIABLES "PYTHONHOME=${_PYTHONHOME}")
  endif()
  set_tests_properties(${python_parser_target} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
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
add_pinocchio_unit_test(joint-translation)
add_pinocchio_unit_test(joint-generic)
add_pinocchio_unit_test(joint-composite)
add_pinocchio_unit_test(joint-mimic)
add_pinocchio_unit_test(joint-helical)
add_pinocchio_unit_test(joint-universal)

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

# Warning ignore with pragma doesn't work for a part of this test, so we ignore two warning globally
cxx_flags_by_compiler_frontend(
  GNU -Wno-maybe-uninitialized -Wuse-after-free=0
  OUTPUT LIEGROUPS_OPTIONS
  FILTER)
add_pinocchio_unit_test(liegroups)
add_test_cflags(liegroups ${LIEGROUPS_OPTIONS})

add_pinocchio_unit_test(cartesian-product-liegroups)
add_pinocchio_unit_test(regressor)
add_pinocchio_unit_test(version)
add_pinocchio_unit_test(copy)
add_pinocchio_unit_test(contact-cholesky)
add_pinocchio_unit_test(classic-acceleration)
add_pinocchio_unit_test(coulomb-friction-cone)

# Serialization
make_directory("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")
add_pinocchio_unit_test(serialization COLLISION_OPTIONAL)
add_test_cflags(
  serialization
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\"")
add_pinocchio_unit_test(csv)
add_test_cflags(
  csv "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\"")

add_subdirectory(algorithm)

# Multiprecision arithmetic
if(BUILD_ADVANCED_TESTING)
  add_pinocchio_unit_test(multiprecision)

  if(MPFR_FOUND)
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
