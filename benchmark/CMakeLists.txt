# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------

ADD_EXECUTABLE(timings timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigen3)
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

GET_TARGET_PROPERTY(TIMINGS_CFLAGS timings COMPILE_FLAGS) 
SET_PROPERTY(TARGET timings PROPERTY COMPILE_FLAGS "${TIMINGS_CFLAGS} -DPINOCCHIO_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"")
