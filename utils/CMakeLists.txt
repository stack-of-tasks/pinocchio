#
# Copyright (c) 2015-2018 CNRS
#

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
MACRO(ADD_UTIL NAME UTIL_SRC PKGS)

  IF(BUILD_UTILS)
    ADD_EXECUTABLE(${NAME} ${UTIL_SRC})
  ELSE(BUILD_UTILS)
    ADD_EXECUTABLE(${NAME} EXCLUDE_FROM_ALL ${UTIL_SRC})
  ENDIF(BUILD_UTILS)

  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)
  TARGET_LINK_LIBRARIES (${NAME} ${Boost_LIBRARIES} ${PROJECT_NAME})
  IF(BUILD_PYTHON_INTERFACE)
    TARGET_LINK_BOOST_PYTHON(${NAME})
    TARGET_LINK_LIBRARIES(${NAME} ${PYTHON_LIBRARIES})
  ENDIF(BUILD_PYTHON_INTERFACE)

  IF(HPP_FCL_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
  ENDIF(HPP_FCL_FOUND)

  ADD_DEPENDENCIES(utils ${NAME})

  IF(BUILD_UTILS)
    INSTALL(TARGETS ${NAME} DESTINATION bin)
  ENDIF(BUILD_UTILS) 


ENDMACRO(ADD_UTIL)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
ADD_CUSTOM_TARGET(utils)

IF(URDFDOM_FOUND)
  ADD_UTIL(pinocchio_read_model pinocchio_read_model "eigen3;urdfdom")
ENDIF(URDFDOM_FOUND)

