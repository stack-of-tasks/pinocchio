# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------

ADD_EXECUTABLE(timings timings.cpp)
PKG_CONFIG_USE_DEPENDENCY(timings eigenpy) # Should be eigen3, to be corrected.
PKG_CONFIG_USE_DEPENDENCY(timings urdfdom)

