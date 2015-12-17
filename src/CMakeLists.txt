# 
# Copyright (c) 2015 CNRS
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

MACRO(ADD_GROUP GROUP_NAME FILENAMES)
  FOREACH(filename ${${FILENAMES}})
    GET_FILENAME_COMPONENT(filenamePath ${filename} PATH)
    IF(NOT (filenamePath STREQUAL ""))
      STRING(REGEX REPLACE "/" "\\\\" filenamePath ${filenamePath}) 
      SOURCE_GROUP("${GROUP_NAME}\\${filenamePath}" FILES ${filename})
    ELSE()
      SOURCE_GROUP("${GROUP_NAME}" FILES ${filename})
    ENDIF()
  ENDFOREACH()
ENDMACRO(ADD_GROUP) 

MACRO(ADD_HEADER_GROUP FILENAMES)
  ADD_GROUP("Header Files" ${FILENAMES})
ENDMACRO(ADD_HEADER_GROUP FILENAMES)

MACRO(ADD_SOURCE_GROUP FILENAMES)
  ADD_GROUP("Source Files" ${FILENAMES})
ENDMACRO(ADD_SOURCE_GROUP FILENAMES)

MACRO(ADD_TARGET_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TARGET_CFLAGS)

# ----------------------------------------------------
# --- C++ --------------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MULTIBODY_SOURCES
  multibody/model.cpp
  multibody/parser/sample-models.cpp
  )

SET(${PROJECT_NAME}_SOURCES
  ${${PROJECT_NAME}_MULTIBODY_SOURCES}
  )

IF(LUA5_1_FOUND)
  SET(${PROJECT_NAME}_MULTIBODY_PARSER_LUA_SOURCES 
      multibody/parser/lua/lua_tables.cpp
      multibody/parser/lua.cpp
    )
  LIST(APPEND ${PROJECT_NAME}_SOURCES ${${PROJECT_NAME}_MULTIBODY_PARSER_LUA_SOURCES})
ENDIF(LUA5_1_FOUND)


IF (UNIX)
    # Create target libpinocchio.so
    ADD_LIBRARY ( ${PROJECT_NAME} SHARED ${HEADERS} ${${PROJECT_NAME}_SOURCES} )
		SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
      PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen3)

    IF(LUA5_1_FOUND)
      PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} lua5.1)
    ENDIF(LUA5_1_FOUND)
    
    ADD_HEADER_GROUP(HEADERS)  
    ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)  

    INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)
ENDIF ( )

# ----------------------------------------------------
# --- PYTHON -----------------------------------------
# ----------------------------------------------------

IF(EIGENPY_FOUND)
  FINDPYTHON()
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")
  
  # --- COMPILE WRAPPER
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  ADD_LIBRARY(${PYWRAP} SHARED python/module.cpp python/python.cpp)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
  IF(URDFDOM_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} urdfdom)
  ENDIF(URDFDOM_FOUND)
  IF(LUA5_1_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} lua5.1)
  ENDIF(LUA5_1_FOUND)
  TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_LIBRARIES} ${PROJECT_NAME})

  IF(APPLE)
    # We need to change the extension for python bindings
    SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
  ENDIF(APPLE)

  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}")

  INSTALL(FILES
    "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}/lib${PYWRAP}.so"
    DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  
  # --- INSTALL SCRIPTS 
  SET(PYTHON_FILES
    python/__init__.py
    python/utils.py
    python/robot_wrapper.py
    python/romeo_wrapper.py
    python/rpy.py
    python/explog.py
    )
  
	FOREACH(python ${PYTHON_FILES})
    GET_FILENAME_COMPONENT(pythonFile ${python} NAME)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/src/${python}
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})
      
    # Tag pyc file as generated.
    SET_SOURCE_FILES_PROPERTIES(
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      PROPERTIES GENERATED TRUE)
      
    EXECUTE_PROCESS(COMMAND
      ${PYTHON_EXECUTABLE} -m py_compile
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile} )
      
    # Clean generated files.
    SET_PROPERTY(
      DIRECTORY APPEND PROPERTY
      ADDITIONAL_MAKE_CLEAN_FILES
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c")
      
    INSTALL(FILES
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}"
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  ENDFOREACH(python)

ENDIF(EIGENPY_FOUND)

