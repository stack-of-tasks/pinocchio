#
# Copyright (c) 2015-2018 CNRS
#
# This file is part of Pinocchio
# pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# pinocchio If not, see
# <http://www.gnu.org/licenses/>.

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
    SET_TARGET_PROPERTIES(${bench_name} PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_SOURCE_DIR="${${PROJECT_NAME}_SOURCE_DIR}")
    TARGET_LINK_LIBRARIES(${bench_name} ${PROJECT_NAME})
    PKG_CONFIG_USE_DEPENDENCY(${bench_name} eigen3)
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

# timings
# 
ADD_BENCH(timings-cholesky TRUE)

# timings derivatives
# 
ADD_BENCH(timings-derivatives TRUE)
TARGET_LINK_LIBRARIES(timings-derivatives ${PROJECT_NAME})

# timings-eigen
# 
ADD_BENCH(timings-eigen)

# timings-geometry
# 
IF(URDFDOM_FOUND AND HPP_FCL_FOUND)
  ADD_BENCH(timings-geometry TRUE)
  PKG_CONFIG_USE_DEPENDENCY(timings-geometry hpp-fcl)
ENDIF(URDFDOM_FOUND AND HPP_FCL_FOUND)


