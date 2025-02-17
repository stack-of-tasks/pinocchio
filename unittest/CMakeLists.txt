#
# Copyright (c) 2015-2024 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

function(ADD_TEST_CFLAGS target)
  GET_CPP_TEST_NAME(${target} ${CMAKE_CURRENT_SOURCE_DIR} test_name)
  foreach(ARG ${ARGN})
    set_property(TARGET ${test_name} APPEND_STRING PROPERTY COMPILE_FLAGS "${ARG} ")
  endforeach()
endfunction()

# Compute flags outside the macro to avoid recomputing it for each tests
CXX_FLAGS_BY_COMPILER_FRONTEND(MSVC _USE_MATH_DEFINES OUTPUT TEST_PRIVATE_DEFINITIONS)

function(GET_CPP_TEST_NAME name src_dir full_test_name)
  string(REPLACE "${PINOCCHIO_UNIT_TEST_DIR}" "" prefix_name "${src_dir}")
  string(REGEX REPLACE "[/]" "-" prefix_name "${prefix_name}-")

  set(${full_test_name} "${PROJECT_NAME}-test-cpp${prefix_name}${name}" PARENT_SCOPE)
endfunction()

set(PINOCCHIO_UNIT_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

function(ADD_PINOCCHIO_UNIT_TEST name)
  set(
    options
    HEADER_ONLY
    PARSERS
    EXTRA
    COLLISION
    PARALLEL
    PYTHON_PARSER
    PARSERS_OPTIONAL
    EXTRA_OPTIONAL
    COLLISION_OPTIONAL
    PARALLEL_OPTIONAL
  )
  set(oneValueArgs)
  set(multiValueArgs PACKAGES)
  cmake_parse_arguments(unit_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PKGS ${unit_test_PACKAGES})

  GET_CPP_TEST_NAME(${name} ${CMAKE_CURRENT_SOURCE_DIR} TEST_NAME)
  ADD_UNIT_TEST(${TEST_NAME} ${name}.cpp)

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
    set(
      ENV_VARIABLES
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
    ADD_WINDOWS_DLL_PATH_TO_TEST(${TEST_NAME})
    get_test_property(${TEST_NAME} ENVIRONMENT ENV_VARIABLES)
    COMPUTE_PYTHONPATH(PYTHON_ENV_VARIABLES "bindings/python")
    list(APPEND ENV_VARIABLES ${PYTHON_ENV_VARIABLES})
    if(WIN32)
      # This line is mandatory because of Github action. The test run well on Windows + Conda. This
      # hide something wrong. Maybe the test is linking against the wrong Python library or call the
      # wrong interpreter.
      get_filename_component(_PYTHONHOME ${PYTHON_EXECUTABLE} PATH)
      list(APPEND ENV_VARIABLES "PYTHONHOME=${_PYTHONHOME}")
    endif()
    set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
  endif()

  MODERNIZE_TARGET_LINK_LIBRARIES(
    ${TEST_NAME}
    SCOPE PRIVATE
    TARGETS Boost::unit_test_framework
    LIBRARIES ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  )

  if(PKGS)
    target_link_libraries(${TEST_NAME} PRIVATE ${PKGS})
  endif()
endfunction()

macro(ADD_PINOCCHIO_PARALLEL_UNIT_TEST NAME)
  if(BUILD_WITH_OPENMP_SUPPORT)
    ADD_PINOCCHIO_UNIT_TEST(${ARGV} PARALLEL)
  endif()
endmacro()

# Find Boost.UnitTestFramework
find_package(Boost COMPONENTS unit_test_framework)

# Header only
ADD_PINOCCHIO_UNIT_TEST(macros HEADER_ONLY)

# Math components
ADD_PINOCCHIO_UNIT_TEST(eigen-basic-op)
ADD_PINOCCHIO_UNIT_TEST(eigen-tensor)
ADD_PINOCCHIO_UNIT_TEST(sincos)
ADD_PINOCCHIO_UNIT_TEST(quaternion)
ADD_PINOCCHIO_UNIT_TEST(rpy)
ADD_PINOCCHIO_UNIT_TEST(rotation)
ADD_PINOCCHIO_UNIT_TEST(vector)
ADD_PINOCCHIO_UNIT_TEST(eigenvalues)
ADD_PINOCCHIO_UNIT_TEST(tridiagonal-matrix)
# TOOD This test contains an undefined behavior and is not stable. Reactivte the test when the fix
# is merged.

# add_pinocchio_unit_test(lanczos-decomposition)
ADD_PINOCCHIO_UNIT_TEST(gram-schmidt-orthonormalisation)

# Derivatives algo
ADD_PINOCCHIO_UNIT_TEST(kinematics-derivatives)
ADD_PINOCCHIO_UNIT_TEST(frames-derivatives)
ADD_PINOCCHIO_UNIT_TEST(rnea-derivatives)
ADD_PINOCCHIO_UNIT_TEST(aba-derivatives)
ADD_PINOCCHIO_UNIT_TEST(centroidal-derivatives)
ADD_PINOCCHIO_UNIT_TEST(center-of-mass-derivatives)
ADD_PINOCCHIO_UNIT_TEST(constrained-dynamics-derivatives)
ADD_PINOCCHIO_UNIT_TEST(rnea-second-order-derivatives)

# Pinocchio features
ADD_PINOCCHIO_UNIT_TEST(spatial)
ADD_PINOCCHIO_UNIT_TEST(symmetric)
ADD_PINOCCHIO_UNIT_TEST(aba)
ADD_PINOCCHIO_PARALLEL_UNIT_TEST(parallel-aba)
ADD_PINOCCHIO_UNIT_TEST(rnea)
ADD_PINOCCHIO_PARALLEL_UNIT_TEST(parallel-rnea)
ADD_PINOCCHIO_UNIT_TEST(crba)
ADD_PINOCCHIO_UNIT_TEST(centroidal)
ADD_PINOCCHIO_UNIT_TEST(com)
ADD_PINOCCHIO_UNIT_TEST(joint-jacobian)
ADD_PINOCCHIO_UNIT_TEST(cholesky)
ADD_PINOCCHIO_UNIT_TEST(constrained-dynamics)
ADD_PINOCCHIO_UNIT_TEST(contact-models)
ADD_PINOCCHIO_UNIT_TEST(contact-dynamics)
ADD_PINOCCHIO_UNIT_TEST(contact-inverse-dynamics)
ADD_PINOCCHIO_UNIT_TEST(closed-loop-dynamics)
ADD_PINOCCHIO_UNIT_TEST(sample-models COLLISION_OPTIONAL)
ADD_PINOCCHIO_UNIT_TEST(kinematics)
# Test for unstable features TODO fix and reactivate these tests in next version
# ADD_PINOCCHIO_UNIT_TEST(delassus) ADD_PINOCCHIO_UNIT_TEST(pv-solver)
ADD_PINOCCHIO_UNIT_TEST(impulse-dynamics-derivatives)
if(BUILD_WITH_SDF_SUPPORT)
  ADD_PINOCCHIO_UNIT_TEST(contact-dynamics-derivatives PARSERS)
else()
  ADD_PINOCCHIO_UNIT_TEST(contact-dynamics-derivatives)
endif()
ADD_PINOCCHIO_UNIT_TEST(constraint-variants)
ADD_PINOCCHIO_UNIT_TEST(impulse-dynamics)

ADD_PINOCCHIO_UNIT_TEST(mjcf PARSERS COLLISION_OPTIONAL)

if(BUILD_WITH_URDF_SUPPORT)
  ADD_PINOCCHIO_UNIT_TEST(urdf PARSERS COLLISION_OPTIONAL)

  ADD_PINOCCHIO_UNIT_TEST(value PARSERS)
  if(BUILD_WITH_HPP_FCL_SUPPORT)
    ADD_PINOCCHIO_UNIT_TEST(geometry-object COLLISION)
    ADD_PINOCCHIO_UNIT_TEST(geometry-model PARSERS COLLISION)
    ADD_PINOCCHIO_UNIT_TEST(geometry-algorithms PARSERS COLLISION)
    ADD_PINOCCHIO_UNIT_TEST(broadphase PARSERS COLLISION)
    ADD_PINOCCHIO_UNIT_TEST(tree-broadphase PARSERS COLLISION)
    ADD_PINOCCHIO_PARALLEL_UNIT_TEST(parallel-geometry PARSERS COLLISION)
    ADD_PINOCCHIO_UNIT_TEST(srdf PARSERS COLLISION)
  endif()
endif()

if(BUILD_WITH_SDF_SUPPORT)
  ADD_PINOCCHIO_UNIT_TEST(sdf PARSERS)
endif()

if(BUILD_WITH_EXTRA_SUPPORT)
  ADD_PINOCCHIO_UNIT_TEST(reachable-workspace PARSERS EXTRA COLLISION_OPTIONAL)
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
  ADD_PINOCCHIO_UNIT_TEST(python_parser PYTHON_PARSER)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Test over the joints
ADD_PINOCCHIO_UNIT_TEST(all-joints)
ADD_PINOCCHIO_UNIT_TEST(joint-revolute)
ADD_PINOCCHIO_UNIT_TEST(joint-prismatic)
ADD_PINOCCHIO_UNIT_TEST(joint-planar)
ADD_PINOCCHIO_UNIT_TEST(joint-free-flyer)
ADD_PINOCCHIO_UNIT_TEST(joint-spherical)
ADD_PINOCCHIO_UNIT_TEST(joint-translation)
ADD_PINOCCHIO_UNIT_TEST(joint-generic)
ADD_PINOCCHIO_UNIT_TEST(joint-composite)
ADD_PINOCCHIO_UNIT_TEST(joint-mimic)
ADD_PINOCCHIO_UNIT_TEST(joint-helical)
ADD_PINOCCHIO_UNIT_TEST(joint-universal)

# Main corpus
ADD_PINOCCHIO_UNIT_TEST(model COLLISION_OPTIONAL)
ADD_PINOCCHIO_UNIT_TEST(data)
ADD_PINOCCHIO_UNIT_TEST(joint-motion-subspace)
ADD_PINOCCHIO_UNIT_TEST(compute-all-terms)
ADD_PINOCCHIO_UNIT_TEST(energy)
ADD_PINOCCHIO_UNIT_TEST(frames)
if(NOT MSVC AND NOT MSVC_VERSION)
  ADD_PINOCCHIO_UNIT_TEST(joint-configurations)
endif()
ADD_PINOCCHIO_UNIT_TEST(explog)
ADD_PINOCCHIO_UNIT_TEST(finite-differences)
ADD_PINOCCHIO_UNIT_TEST(visitor)
ADD_PINOCCHIO_UNIT_TEST(algo-check)

# Warning ignore with pragma doesn't work for a part of this test, so we ignore two warning globally
CXX_FLAGS_BY_COMPILER_FRONTEND(
  GNU -Wno-maybe-uninitialized -Wuse-after-free=0
  OUTPUT LIEGROUPS_OPTIONS
  FILTER
)
ADD_PINOCCHIO_UNIT_TEST(liegroups)
ADD_TEST_CFLAGS(liegroups ${LIEGROUPS_OPTIONS})

ADD_PINOCCHIO_UNIT_TEST(cartesian-product-liegroups)
ADD_PINOCCHIO_UNIT_TEST(regressor)
ADD_PINOCCHIO_UNIT_TEST(version)
ADD_PINOCCHIO_UNIT_TEST(copy)
ADD_PINOCCHIO_UNIT_TEST(contact-cholesky)
ADD_PINOCCHIO_UNIT_TEST(classic-acceleration)
ADD_PINOCCHIO_UNIT_TEST(coulomb-friction-cone)

# Serialization
make_directory("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")
ADD_PINOCCHIO_UNIT_TEST(serialization COLLISION_OPTIONAL)
ADD_TEST_CFLAGS(
  serialization
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)
ADD_PINOCCHIO_UNIT_TEST(csv)
ADD_TEST_CFLAGS(
  csv
  "-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\""
)

add_subdirectory(algorithm)

# Multiprecision arithmetic
if(BUILD_ADVANCED_TESTING)
  ADD_PINOCCHIO_UNIT_TEST(multiprecision)

  if(MPFR_FOUND)
    ADD_PINOCCHIO_UNIT_TEST(multiprecision-mpfr PACKAGES mpfr gmp)
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
