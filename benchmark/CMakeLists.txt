# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------

ADD_EXECUTABLE(timings timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigen3)
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

SET_TARGET_PROPERTIES (timings PROPERTIES COMPILE_DEFINITIONS PINOCCHIO_SOURCE_DIR="${${PROJECT_NAME}_SOURCE_DIR}")
