#
# Copyright (c) 2015-2016 CNRS
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
MACRO(ADD_UNIT_TEST NAME PKGS)
  IF (BUILD_UNIT_TESTS)
    ADD_EXECUTABLE(${NAME} ${NAME})
  ELSE (BUILD_UNIT_TESTS)
    ADD_EXECUTABLE(${NAME} EXCLUDE_FROM_ALL ${NAME})
  ENDIF (BUILD_UNIT_TESTS)
  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)

  IF(HPP_FCL_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
    ADD_TEST_CFLAGS(${NAME} "-DWITH_HPP_FCL")
  ENDIF(HPP_FCL_FOUND)

  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

  IF(BUILD_PYTHON_INTERFACE)
    TARGET_LINK_BOOST_PYTHON(${NAME})
    TARGET_LINK_LIBRARIES(${NAME} ${PYTHON_LIBRARIES})
  ENDIF(BUILD_PYTHON_INTERFACE)

  ADD_TEST(NAME ${NAME} COMMAND ${NAME})
  ADD_DEPENDENCIES(check ${NAME})
ENDMACRO(ADD_UNIT_TEST)

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND})

ADD_UNIT_TEST(tspatial eigen3)
IF(METAPOD_FOUND)
  ADD_UNIT_TEST(symmetric metapod eigen3)
  ADD_TEST_CFLAGS(symmetric "-DWITH_METAPOD")
ELSE(METAPOD_FOUND)
  ADD_UNIT_TEST(symmetric eigen3)
ENDIF(METAPOD_FOUND)
ADD_UNIT_TEST(aba eigen3)
ADD_UNIT_TEST(rnea eigen3)
ADD_UNIT_TEST(crba eigen3)
ADD_UNIT_TEST(com eigen3)
ADD_UNIT_TEST(jacobian eigen3)
ADD_UNIT_TEST(cholesky eigen3)
ADD_UNIT_TEST(dynamics eigen3)

IF(URDFDOM_FOUND)
  ADD_UNIT_TEST(urdf "eigen3;urdfdom")
  ADD_TEST_CFLAGS(urdf '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

  ADD_UNIT_TEST(value "eigen3;urdfdom")
  ADD_TEST_CFLAGS(value '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
  IF(HPP_FCL_FOUND)
    ADD_UNIT_TEST(geom "eigen3;urdfdom")
    ADD_TEST_CFLAGS(geom '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
    IF(BUILD_TESTS_WITH_HPP)
      ADD_OPTIONAL_DEPENDENCY("hpp-model-urdf")
      IF(HPP_MODEL_URDF_FOUND)
        ADD_OPTIONAL_DEPENDENCY("romeo_description >= 0.2")
        PKG_CONFIG_USE_DEPENDENCY(geom hpp-model-urdf)
        ADD_TEST_CFLAGS(geom "-DWITH_HPP_MODEL_URDF")
        IF(ROMEO_DESCRIPTION_FOUND)
          ADD_TEST_CFLAGS(geom '-DROMEO_DESCRIPTION_MODEL_DIR=\\\"${ROMEO_DESCRIPTION_DATAROOTDIR}\\\"')
        ENDIF(ROMEO_DESCRIPTION_FOUND)
      ENDIF(HPP_MODEL_URDF_FOUND)
    ENDIF(BUILD_TESTS_WITH_HPP)
  ENDIF(HPP_FCL_FOUND)
ENDIF(URDFDOM_FOUND)

IF(LUA5_1_FOUND)
  ADD_UNIT_TEST(lua "eigen3;lua5.1")
  ADD_TEST_CFLAGS(lua '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ENDIF(LUA5_1_FOUND)

IF(BUILD_PYTHON_INTERFACE)
  ADD_UNIT_TEST(python_parser eigen3)
  ADD_TEST_CFLAGS(python_parser '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ENDIF(BUILD_PYTHON_INTERFACE)

# Work in progress
ADD_UNIT_TEST(model eigen3)
ADD_UNIT_TEST(constraint eigen3)
#ADD_UNIT_TEST(variant eigen3)
ADD_UNIT_TEST(joints eigen3)
ADD_UNIT_TEST(compute-all-terms eigen3)
ADD_UNIT_TEST(energy eigen3)
ADD_UNIT_TEST(frames eigen3)
ADD_UNIT_TEST(joint-configurations eigen3)
ADD_UNIT_TEST(joint eigen3)
ADD_UNIT_TEST(explog eigen3)
ADD_UNIT_TEST(finite-differences eigen3)
ADD_UNIT_TEST(visitor eigen3)
ADD_UNIT_TEST(algo-check eigen3)
ADD_UNIT_TEST(joint-composite eigen3)

