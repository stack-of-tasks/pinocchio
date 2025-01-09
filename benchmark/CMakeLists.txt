#
# Copyright (c) 2015-2023 CNRS INRIA
#

# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
add_custom_target(bench)
add_project_private_dependency(benchmark REQUIRED)

macro(ADD_PINOCCHIO_BENCH bench_name)
  set(options HEADER_ONLY GOOGLE_BENCHMARK)
  set(oneValueArgs)
  set(multiValueArgs PACKAGES)
  cmake_parse_arguments(unit_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(BUILD_BENCHMARK)
    add_executable(${bench_name} ${bench_name}.cpp)
  else()
    add_executable(${bench_name} EXCLUDE_FROM_ALL ${bench_name}.cpp)
  endif()

  target_compile_definitions(${bench_name} PRIVATE PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")
  if(NOT unit_test_HEADER_ONLY)
    target_link_libraries(${bench_name} PRIVATE pinocchio_default)
  else()
    target_link_libraries(${bench_name} PRIVATE pinocchio_headers)
  endif()

  if(unit_test_GOOGLE_BENCHMARK)
    target_link_libraries(${bench_name} PRIVATE benchmark::benchmark)
  endif()

  add_dependencies(bench ${bench_name})
endmacro()

# timings
#
add_pinocchio_bench(timings)
target_link_libraries(timings PRIVATE pinocchio_parsers)
if(BUILD_WITH_CODEGEN_SUPPORT)
  add_pinocchio_bench(timings-cg)
  target_link_libraries(timings-cg PRIVATE pinocchio_parsers pinocchio_cppadcg ${CMAKE_DL_LIBS})
endif()

if(BUILD_WITH_OPENMP_SUPPORT)
  add_pinocchio_bench(timings-parallel)
  target_link_libraries(timings-parallel PRIVATE pinocchio_parsers pinocchio_parallel
                                                 pinocchio_collision)
endif()

# timings cholesky
#
add_pinocchio_bench(timings-cholesky)
target_link_libraries(timings-cholesky PRIVATE pinocchio_parsers)

# timings derivatives
#
add_pinocchio_bench(timings-derivatives)
target_link_libraries(timings-derivatives PRIVATE pinocchio_parsers)
if(BUILD_WITH_AUTODIFF_SUPPORT)
  # timings-cppad-jit
  add_pinocchio_bench(timings-cppad-jit)
  target_link_libraries(timings-derivatives PRIVATE pinocchio_cppad)
  target_link_libraries(timings-cppad-jit PRIVATE pinocchio_cppad ${CMAKE_DL_LIBS})
  target_compile_definitions(timings-cppad-jit
                             PRIVATE PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")
endif()

# timings-eigen
add_pinocchio_bench(timings-eigen)
modernize_target_link_libraries(
  timings-eigen
  SCOPE PUBLIC
  TARGETS Eigen3::Eigen
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# timings-geometry
#
if(BUILD_WITH_URDF_SUPPORT AND BUILD_WITH_COLLISION_SUPPORT)
  add_pinocchio_bench(timings-geometry)
  target_link_libraries(timings-geometry PRIVATE pinocchio_parsers pinocchio_collision)
endif()

# timings-jacobian
#
add_pinocchio_bench(timings-jacobian)
target_link_libraries(timings-jacobian PRIVATE pinocchio_parsers)

# timings-contact-dynamics
#
add_pinocchio_bench(timings-delassus-operations)
target_link_libraries(timings-delassus-operations PRIVATE pinocchio_parsers)
add_pinocchio_bench(timings-contact-dynamics)
target_link_libraries(timings-contact-dynamics PRIVATE pinocchio_parsers)
add_pinocchio_bench(timings-constrained-dynamics-derivatives)
target_link_libraries(timings-constrained-dynamics-derivatives PRIVATE pinocchio_parsers)
