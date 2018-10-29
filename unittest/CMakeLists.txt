#
# Copyright (c) 2015-2018 CNRS INRIA
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

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

MACRO(ADD_PINOCCHIO_UNIT_TEST NAME PKGS)
  ADD_UNIT_TEST(${NAME} ${NAME}.cpp)
  SET_TARGET_PROPERTIES(${NAME} PROPERTIES LINKER_LANGUAGE CXX)
  TARGET_INCLUDE_DIRECTORIES(${NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_DYN_LINK") 
  SET(MODULE_NAME "${NAME}Test")
  STRING(REGEX REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_MODULE=${MODULE_NAME}") 

  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)

  IF(HPP_FCL_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
  ENDIF(HPP_FCL_FOUND)

  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
ENDMACRO(ADD_PINOCCHIO_UNIT_TEST)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------

# Math components
ADD_PINOCCHIO_UNIT_TEST(sincos eigen3)

# Pinocchio features
ADD_PINOCCHIO_UNIT_TEST(tspatial eigen3)
ADD_PINOCCHIO_UNIT_TEST(symmetric eigen3)
ADD_PINOCCHIO_UNIT_TEST(aba eigen3)
ADD_PINOCCHIO_UNIT_TEST(rnea eigen3)
ADD_PINOCCHIO_UNIT_TEST(crba eigen3)
ADD_PINOCCHIO_UNIT_TEST(centroidal eigen3)
ADD_PINOCCHIO_UNIT_TEST(com eigen3)
ADD_PINOCCHIO_UNIT_TEST(jacobian eigen3)
ADD_PINOCCHIO_UNIT_TEST(cholesky eigen3)
ADD_PINOCCHIO_UNIT_TEST(dynamics eigen3)
ADD_PINOCCHIO_UNIT_TEST(sample-models eigen3)

IF(URDFDOM_FOUND)
  ADD_PINOCCHIO_UNIT_TEST(urdf "eigen3;urdfdom")
  ADD_TEST_CFLAGS(urdf '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

  ADD_PINOCCHIO_UNIT_TEST(value "eigen3;urdfdom")
  ADD_TEST_CFLAGS(value '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
  IF(HPP_FCL_FOUND)
    ADD_PINOCCHIO_UNIT_TEST(geom "eigen3;urdfdom")
    ADD_TEST_CFLAGS(geom '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
    ADD_PINOCCHIO_UNIT_TEST(srdf "eigen3;urdfdom")
    ADD_TEST_CFLAGS(srdf '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
  ENDIF(HPP_FCL_FOUND)
ENDIF(URDFDOM_FOUND)

IF(LUA5_FOUND)
  ADD_PINOCCHIO_UNIT_TEST(lua "eigen3;${LUA5_PACKAGE}")
  ADD_TEST_CFLAGS(lua '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ENDIF(LUA5_FOUND)

IF(BUILD_PYTHON_INTERFACE)
  ADD_PINOCCHIO_UNIT_TEST(python_parser "eigen3;eigenpy")
  ADD_TEST_CFLAGS(python_parser '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

  TARGET_LINK_LIBRARIES(python_parser ${PYWRAP})
  TARGET_LINK_LIBRARIES(python_parser ${PYTHON_LIBRARIES})
ENDIF(BUILD_PYTHON_INTERFACE)

# Work in progress
ADD_PINOCCHIO_UNIT_TEST(model eigen3)
ADD_PINOCCHIO_UNIT_TEST(constraint eigen3)
#ADD_PINOCCHIO_UNIT_TEST(variant eigen3)
ADD_PINOCCHIO_UNIT_TEST(joints eigen3)
ADD_PINOCCHIO_UNIT_TEST(compute-all-terms eigen3)
ADD_PINOCCHIO_UNIT_TEST(energy eigen3)
ADD_PINOCCHIO_UNIT_TEST(frames eigen3)
ADD_PINOCCHIO_UNIT_TEST(joint-configurations eigen3)
ADD_PINOCCHIO_UNIT_TEST(joint eigen3)
ADD_PINOCCHIO_UNIT_TEST(explog eigen3)
ADD_PINOCCHIO_UNIT_TEST(finite-differences eigen3)
ADD_PINOCCHIO_UNIT_TEST(visitor eigen3)
ADD_PINOCCHIO_UNIT_TEST(algo-check eigen3)
ADD_PINOCCHIO_UNIT_TEST(joint-composite eigen3)

ADD_PINOCCHIO_UNIT_TEST(liegroups eigen3)
ADD_PINOCCHIO_UNIT_TEST(regressor eigen3)
ADD_PINOCCHIO_UNIT_TEST(version eigen3)

# Derivatives algo
ADD_PINOCCHIO_UNIT_TEST(kinematics-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(rnea-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(aba-derivatives eigen3)
