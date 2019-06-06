#
# Copyright (c) 2015-2019 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

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
ADD_PINOCCHIO_UNIT_TEST(contact-dynamics eigen3)
ADD_PINOCCHIO_UNIT_TEST(sample-models eigen3)
ADD_PINOCCHIO_UNIT_TEST(kinematics eigen3)

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
  SET(ENV_VARIABLES "PYTHONPATH=${CMAKE_BINARY_DIR}/bindings/python:$ENV{PYTHONPATH}")
  IF(APPLE)
    LIST(APPEND ENV_VARIABLES "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}")
    LIST(APPEND ENV_VARIABLES "DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
  ENDIF(APPLE)
  SET_PROPERTY(TEST python_parser PROPERTY ENVIRONMENT ${ENV_VARIABLES})
  ADD_SUBDIRECTORY(python)
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
ADD_PINOCCHIO_UNIT_TEST(joint-generic eigen3)
ADD_PINOCCHIO_UNIT_TEST(explog eigen3)
ADD_PINOCCHIO_UNIT_TEST(finite-differences eigen3)
ADD_PINOCCHIO_UNIT_TEST(visitor eigen3)
ADD_PINOCCHIO_UNIT_TEST(algo-check eigen3)
ADD_PINOCCHIO_UNIT_TEST(joint-composite eigen3)

ADD_PINOCCHIO_UNIT_TEST(liegroups eigen3)
ADD_PINOCCHIO_UNIT_TEST(regressor eigen3)
ADD_PINOCCHIO_UNIT_TEST(version eigen3)
ADD_PINOCCHIO_UNIT_TEST(copy eigen3)
MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/serialization-data")
ADD_PINOCCHIO_UNIT_TEST(serialization eigen3)
ADD_TEST_CFLAGS(serialization '-DTEST_SERIALIZATION_FOLDER=\\\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\\\"')

# Derivatives algo
ADD_PINOCCHIO_UNIT_TEST(kinematics-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(rnea-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(aba-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(centroidal-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(center-of-mass-derivatives eigen3)
ADD_PINOCCHIO_UNIT_TEST(contact-dynamics-derivatives eigen3)

# Automatic differentiation
IF(CPPAD_FOUND)
  ADD_PINOCCHIO_UNIT_TEST(cppad-basic "eigen3;cppad")
  ADD_PINOCCHIO_UNIT_TEST(cppad-spatial "eigen3;cppad")
  ADD_PINOCCHIO_UNIT_TEST(cppad-joints "eigen3;cppad")
  ADD_PINOCCHIO_UNIT_TEST(cppad-algo "eigen3;cppad")
  ADD_PINOCCHIO_UNIT_TEST(cppad-algo-derivatives "eigen3;cppad")

  IF(CPPADCG_FOUND)
    ADD_PINOCCHIO_UNIT_TEST(cppadcg-basic "eigen3;cppadcg")
    SET_PROPERTY(TARGET cppadcg-basic PROPERTY CXX_STANDARD 11)
    TARGET_LINK_LIBRARIES(cppadcg-basic ${CMAKE_DL_LIBS})
    ADD_PINOCCHIO_UNIT_TEST(cppadcg-algo "eigen3;cppadcg")
    SET_PROPERTY(TARGET cppadcg-algo PROPERTY CXX_STANDARD 11)
    TARGET_LINK_LIBRARIES(cppadcg-algo ${CMAKE_DL_LIBS})
  ENDIF(CPPADCG_FOUND)

ENDIF(CPPAD_FOUND)

IF(CASADI_FOUND)
  ADD_PINOCCHIO_UNIT_TEST(casadi-basic "eigen3;casadi")
  SET_PROPERTY(TARGET casadi-basic PROPERTY CXX_STANDARD 11)
  ADD_PINOCCHIO_UNIT_TEST(casadi-spatial "eigen3;casadi")
  SET_PROPERTY(TARGET casadi-spatial PROPERTY CXX_STANDARD 11)
ENDIF(CASADI_FOUND)

