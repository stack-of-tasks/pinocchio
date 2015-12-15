# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

ADD_EXECUTABLE(timings timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigen3)
IF(URDFDOM_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)
  PKG_CONFIG_USE_DEPENDENCY(timings assimp)
ENDIF(URDFDOM_FOUND)
IF(HPP_FCL_FOUND)
	PKG_CONFIG_USE_DEPENDENCY(timings hpp-fcl)
	ADD_TEST_CFLAGS(timings "-DWITH_HPP_FCL")
ENDIF(HPP_FCL_FOUND)
TARGET_LINK_LIBRARIES (timings ${Boost_LIBRARIES} ${PROJECT_NAME})

SET_TARGET_PROPERTIES (timings PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_SOURCE_DIR="${${PROJECT_NAME}_SOURCE_DIR}")
