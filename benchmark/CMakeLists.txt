#
# Copyright (c) 2015-2019 CNRS INRIA
#

# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
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
      PKG_CONFIG_USE_DEPENDENCY(${bench_name} eigen3)

      IF(HPP_FCL_FOUND)
        PKG_CONFIG_USE_DEPENDENCY(${bench_name} hpp-fcl)
      ENDIF(HPP_FCL_FOUND)

      IF(URDFDOM_FOUND)
        PKG_CONFIG_USE_DEPENDENCY(${bench_name} urdfdom)
      ENDIF(URDFDOM_FOUND)

    ENDIF(link_to_main_lib)
  ENDIF()
  
  ADD_DEPENDENCIES(bench ${bench_name})
ENDMACRO(ADD_BENCH)

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

# timings
# 
ADD_BENCH(timings TRUE)
IF(CPPADCG_FOUND)
  ADD_BENCH(timings-cg TRUE)
  SET_PROPERTY(TARGET timings-cg PROPERTY CXX_STANDARD 11)
  PKG_CONFIG_USE_DEPENDENCY(timings-cg "cppadcg")
  TARGET_LINK_LIBRARIES(timings-cg PUBLIC ${CMAKE_DL_LIBS})
ENDIF(CPPADCG_FOUND)

# timings
# 
ADD_BENCH(timings-cholesky TRUE)

# timings derivatives
 
ADD_BENCH(timings-derivatives TRUE)
IF(CPPAD_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(timings-derivatives "cppad")
ENDIF(CPPAD_FOUND)
IF(CPPADCG_FOUND)
  SET_PROPERTY(TARGET timings-derivatives PROPERTY CXX_STANDARD 11)
  PKG_CONFIG_USE_DEPENDENCY(timings-derivatives "cppadcg")
ENDIF(CPPADCG_FOUND)

# timings-eigen
# 
ADD_BENCH(timings-eigen)
PKG_CONFIG_USE_DEPENDENCY(timings-eigen "eigen3")

# timings-geometry
# 
IF(URDFDOM_FOUND AND HPP_FCL_FOUND)
  ADD_BENCH(timings-geometry TRUE)
ENDIF(URDFDOM_FOUND AND HPP_FCL_FOUND)

# timings-jacobian
#
ADD_BENCH(timings-jacobian TRUE)

# timings-contact-dynamics
#
ADD_BENCH(timings-contact-dynamics TRUE)
