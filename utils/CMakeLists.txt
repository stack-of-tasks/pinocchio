#
# Copyright (c) 2015-2020 CNRS INRIA
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
macro(ADD_UTIL NAME UTIL_SRC)

  if(BUILD_UTILS)
    add_executable(${NAME} "${UTIL_SRC}.cpp")
  else(BUILD_UTILS)
    add_executable(${NAME} EXCLUDE_FROM_ALL "${UTIL_SRC}.cpp")
  endif(BUILD_UTILS)

  target_link_libraries(${NAME} PUBLIC ${PROJECT_NAME})

  add_dependencies(utils ${NAME})

  if(BUILD_UTILS)
    install(TARGETS ${NAME} DESTINATION bin)
  endif(BUILD_UTILS)

endmacro(ADD_UTIL)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
add_custom_target(utils)

if(urdfdom_FOUND)
  add_util(pinocchio_read_model pinocchio_read_model)
endif(urdfdom_FOUND)
