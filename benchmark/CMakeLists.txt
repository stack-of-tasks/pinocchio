#
# Copyright (c) 2015-2016 CNRS
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
MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)


# timings
# 
IF(BUILD_BENCHMARK)
  ADD_EXECUTABLE(timings timings.cpp)
ELSE(BUILD_BENCHMARK)
  ADD_EXECUTABLE(timings EXCLUDE_FROM_ALL timings.cpp)
ENDIF(BUILD_BENCHMARK)
TARGET_LINK_LIBRARIES (timings ${PROJECT_NAME})
SET_TARGET_PROPERTIES (timings PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_SOURCE_DIR="${${PROJECT_NAME}_SOURCE_DIR}")

# timings-eigen
# 
IF(BUILD_BENCHMARK)
  ADD_EXECUTABLE(timings-eigen timings-eigen.cpp)
ELSE(BUILD_BENCHMARK)
  ADD_EXECUTABLE(timings-eigen EXCLUDE_FROM_ALL timings-eigen.cpp)
ENDIF(BUILD_BENCHMARK)
PKG_CONFIG_USE_DEPENDENCY(timings-eigen eigen3)


# geomTimings
# 
IF(URDFDOM_FOUND AND HPP_FCL_FOUND)
  IF(BUILD_BENCHMARK)
    ADD_EXECUTABLE(geomTimings timings-geometry.cpp)
  ELSE(BUILD_BENCHMARK)
    ADD_EXECUTABLE(geomTimings EXCLUDE_FROM_ALL timings-geometry.cpp)
  ENDIF(BUILD_BENCHMARK)

    IF(BUILD_TESTS_WITH_HPP)
      ADD_OPTIONAL_DEPENDENCY("hpp-model-urdf")
      IF(HPP_MODEL_URDF_FOUND)
        PKG_CONFIG_USE_DEPENDENCY(geomTimings hpp-model-urdf)
        ADD_TEST_CFLAGS(geomTimings "-DWITH_HPP_MODEL_URDF")
      ENDIF(HPP_MODEL_URDF_FOUND)
    ENDIF(BUILD_TESTS_WITH_HPP)

  TARGET_LINK_LIBRARIES (geomTimings ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(geomTimings hpp-fcl)
  SET_TARGET_PROPERTIES (geomTimings PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_SOURCE_DIR="${${PROJECT_NAME}_SOURCE_DIR}")
ENDIF(URDFDOM_FOUND AND HPP_FCL_FOUND)


