#
# Copyright (c) 2015-2023 CNRS INRIA
#

# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
add_custom_target(bench)

macro(ADD_BENCH bench_name)
  if(BUILD_BENCHMARK)
    add_executable(${bench_name} ${bench_name}.cpp)
  else(BUILD_BENCHMARK)
    add_executable(${bench_name} EXCLUDE_FROM_ALL ${bench_name}.cpp)
  endif(BUILD_BENCHMARK)
  set(ExtraMacroArgs ${ARGN})
  list(LENGTH ExtraMacroArgs NumExtraMacroArgs)
  if(NumExtraMacroArgs GREATER 0)
    set(link_to_main_lib ${ARGV1})
    if(link_to_main_lib)
      set_target_properties(${bench_name} PROPERTIES COMPILE_DEFINITIONS
                                                     PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")
      target_link_libraries(${bench_name} PUBLIC ${PROJECT_NAME})
    endif(link_to_main_lib)
  endif()

  add_dependencies(bench ${bench_name})
endmacro(ADD_BENCH)

macro(ADD_TEST_CFLAGS target flag)
  set_property(
    TARGET ${target}
    APPEND_STRING
    PROPERTY COMPILE_FLAGS " ${flag}")
endmacro(ADD_TEST_CFLAGS)

if(CPPAD_FOUND)
  include_directories(${cppad_INCLUDE_DIR})
endif(CPPAD_FOUND)

if(cppadcg_FOUND)
  include_directories(${cppadcg_INCLUDE_DIR})
endif(cppadcg_FOUND)

# timings
#
add_bench(timings TRUE)
if(cppadcg_FOUND)
  add_bench(timings-cg TRUE)
  target_link_libraries(timings-cg PUBLIC ${CMAKE_DL_LIBS} ${cppad_LIBRARY})
endif(cppadcg_FOUND)

if(BUILD_WITH_OPENMP_SUPPORT)
  add_bench(timings-parallel TRUE)
endif(BUILD_WITH_OPENMP_SUPPORT)

# timings
#
add_bench(timings-cholesky TRUE)

# timings derivatives

add_bench(timings-derivatives TRUE)
if(CPPAD_FOUND)
  # timings-cppad-jit
  add_bench(timings-cppad-jit TRUE)
  target_link_libraries(timings-derivatives PUBLIC ${cppad_LIBRARY})
  target_link_libraries(timings-cppad-jit PUBLIC ${cppad_LIBRARY} ${CMAKE_DL_LIBS})
  target_compile_definitions(timings-cppad-jit
                             PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")
endif()

# timings-eigen
#
add_bench(timings-eigen)
modernize_target_link_libraries(
  timings-eigen
  SCOPE PUBLIC
  TARGETS Eigen3::Eigen
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# timings-geometry
#
if(URDFDOM_FOUND AND coal_FOUND)
  add_bench(timings-geometry TRUE)
endif(URDFDOM_FOUND AND coal_FOUND)

# timings-jacobian
#
add_bench(timings-jacobian TRUE)

# timings-contact-dynamics
#
add_bench(timings-delassus-operations TRUE)
add_bench(timings-contact-dynamics TRUE)
add_bench(timings-constrained-dynamics-derivatives TRUE)
