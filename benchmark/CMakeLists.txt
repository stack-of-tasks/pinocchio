# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------

ADD_EXECUTABLE(timings timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigen3)
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

