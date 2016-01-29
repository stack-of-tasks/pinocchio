#
# Copyright (c) 2015 CNRS
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
MACRO(ADD_UTIL NAME UTIL_SRC PKGS)
  ADD_EXECUTABLE(${NAME} ${UTIL_SRC})
  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)
  TARGET_LINK_LIBRARIES (${NAME} ${Boost_LIBRARIES} ${PROJECT_NAME})
  ADD_DEPENDENCIES(utils ${NAME})
  INSTALL(TARGETS ${NAME} DESTINATION bin)
ENDMACRO(ADD_UTIL)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
ADD_CUSTOM_TARGET(utils)

IF(URDFDOM_FOUND)
  ADD_UTIL(pinocchio_read_model pinocchio_read_model "eigen3;urdfdom")
ENDIF(URDFDOM_FOUND)

