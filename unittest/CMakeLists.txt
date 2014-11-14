# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
MACRO(ADD_UNIT_TEST NAME PKGS)
  ADD_EXECUTABLE(${NAME} EXCLUDE_FROM_ALL ${NAME})
  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
    MESSAGE(STATUS "PACKAGE "${PKG})
  ENDFOREACH(PKG)
  ADD_TEST(NAME ${NAME} COMMAND ${NAME})
  ADD_DEPENDENCIES(check ${NAME})
ENDMACRO(ADD_UNIT_TEST)

MACRO(ADD_TEST_CFLAGS target flag)
  GET_TARGET_PROPERTY(${target}_CFLAGS ${target} COMPILE_FLAGS) 
  SET_PROPERTY(TARGET ${target} PROPERTY COMPILE_FLAGS "${${target}_CFLAGS} ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND})

ADD_UNIT_TEST(tspatial eigen3)
ADD_UNIT_TEST(symmetric eigen3)
IF(METAPOD_FOUND)
  ADD_TEST_CFLAGS(symmetric "-DWITH_METAPOD")
ENDIF(METAPOD_FOUND)
ADD_UNIT_TEST(rnea eigen3)
ADD_UNIT_TEST(crba eigen3)
ADD_UNIT_TEST(com eigen3)
ADD_UNIT_TEST(jacobian eigen3)
ADD_UNIT_TEST(cholesky eigen3)

IF(URDFDOM_FOUND)
  ADD_UNIT_TEST(urdf "eigen3;urdfdom")
  ADD_TEST_CFLAGS(urdf '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

  ADD_UNIT_TEST(value "eigen3;urdfdom")
  ADD_TEST_CFLAGS(value '-DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')
ENDIF(URDFDOM_FOUND)

# Work in progress
#ADD_UNIT_TEST(constraint eigen3)
#ADD_UNIT_TEST(variant eigen3)
