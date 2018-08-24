# 
# Copyright (c) 2015-2017 CNRS
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
# 
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.

MACRO(ADD_TARGET_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TARGET_CFLAGS)

# ----------------------------------------------------
# --- C++ --------------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MULTIBODY_SOURCES
  multibody/model.cpp
  )

SET(${PROJECT_NAME}_PARSERS_SOURCES
  parsers/sample-models.cpp
  )

IF(LUA5_FOUND)
  LIST(APPEND ${PROJECT_NAME}_PARSERS_SOURCES 
      parsers/lua/lua_tables.cpp
      parsers/lua.cpp
    )
ENDIF(LUA5_FOUND)

SET(${PROJECT_NAME}_SOURCES
  ${${PROJECT_NAME}_MULTIBODY_SOURCES}
  ${${PROJECT_NAME}_PARSERS_SOURCES}
)

IF(UNIX)
    # Create target libpinocchio.so
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${HEADERS} ${${PROJECT_NAME}_SOURCES})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    IF(BUILD_WITH_COMMIT_VERSION)
      TAG_LIBRARY_VERSION(${PROJECT_NAME})
    ENDIF(BUILD_WITH_COMMIT_VERSION)
    PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen3)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

    IF(URDFDOM_FOUND)
      PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} urdfdom)
    ENDIF(URDFDOM_FOUND)

    IF(HPP_FCL_FOUND)
      PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-fcl)
    ENDIF(HPP_FCL_FOUND)

    IF(LUA5_FOUND)
      PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} ${LUA5_PACKAGE})
    ENDIF(LUA5_FOUND)

    ADD_HEADER_GROUP(HEADERS)
    ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)

    INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)
ENDIF(UNIX)

