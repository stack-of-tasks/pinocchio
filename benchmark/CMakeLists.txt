#
# Copyright (c) 2015-2023 CNRS INRIA
#

# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
IF(WIN32)
  ADD_DEFINITIONS("-DNOMINMAX")
ENDIF()

ADD_CUSTOM_TARGET(bench)

MACRO(ADD_BENCH bench_name)
  IF(BUILD_BENCHMARK)
    ADD_EXECUTABLE(${bench_name} ${bench_name}.cpp)
  ELSE(BUILD_BENCHMARK)
    ADD_EXECUTABLE(${bench_name} EXCLUDE_FROM_ALL ${bench_name}.cpp)
  ENDIF(BUILD_BENCHMARK)
  SET(ExtraMacroArgs ${ARGN})
  LIST(LENGTH ExtraMacroArgs NumExtraMacroArgs)
  IF(NumExtraMacroArgs GREATER 0)
    SET(link_to_main_lib ${ARGV1})
    IF(link_to_main_lib)
      SET_TARGET_PROPERTIES(${bench_name} PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")
      TARGET_LINK_LIBRARIES(${bench_name} PUBLIC ${PROJECT_NAME})
    ENDIF(link_to_main_lib)
  ENDIF()

  ADD_DEPENDENCIES(bench ${bench_name})
ENDMACRO(ADD_BENCH)

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

IF(CPPAD_FOUND)
  INCLUDE_DIRECTORIES(${cppad_INCLUDE_DIR})
ENDIF(CPPAD_FOUND)

IF(cppadcg_FOUND)
  INCLUDE_DIRECTORIES(${cppadcg_INCLUDE_DIR})
ENDIF(cppadcg_FOUND)

# timings
#
ADD_BENCH(timings TRUE)
IF(cppadcg_FOUND)
  ADD_BENCH(timings-cg TRUE)
  SET_PROPERTY(TARGET timings-cg PROPERTY CXX_STANDARD 11)
  TARGET_LINK_LIBRARIES(timings-cg PUBLIC ${CMAKE_DL_LIBS})
ENDIF(cppadcg_FOUND)

IF(BUILD_WITH_OPENMP_SUPPORT)
  ADD_BENCH(timings-parallel TRUE)
ENDIF(BUILD_WITH_OPENMP_SUPPORT)

# timings
#
ADD_BENCH(timings-cholesky TRUE)

# timings derivatives

ADD_BENCH(timings-derivatives TRUE)
IF(CPPAD_FOUND)
  # timings-cppad-jit
  ADD_BENCH(timings-cppad-jit TRUE)
  TARGET_LINK_LIBRARIES(timings-derivatives PUBLIC ${cppad_LIBRARY})
  TARGET_LINK_LIBRARIES(timings-cppad-jit PUBLIC ${cppad_LIBRARY} ${CMAKE_DL_LIBS})
ENDIF(CPPAD_FOUND)
IF(cppadcg_FOUND)
  SET_PROPERTY(TARGET timings-derivatives PROPERTY CXX_STANDARD 11)
ENDIF(cppadcg_FOUND)

# timings-eigen
#
ADD_BENCH(timings-eigen)
MODERNIZE_TARGET_LINK_LIBRARIES(timings-eigen SCOPE PUBLIC
  TARGETS Eigen3::Eigen
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# timings-geometry
#
IF(URDFDOM_FOUND AND hpp-fcl_FOUND)
  ADD_BENCH(timings-geometry TRUE)
ENDIF(URDFDOM_FOUND AND hpp-fcl_FOUND)

# timings-jacobian
#
ADD_BENCH(timings-jacobian TRUE)

# timings-contact-dynamics
#
ADD_BENCH(timings-delassus-operations TRUE)
ADD_BENCH(timings-contact-dynamics TRUE)
ADD_BENCH(timings-constrained-dynamics-derivatives TRUE)
