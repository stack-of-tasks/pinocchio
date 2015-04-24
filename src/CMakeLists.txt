#
# Copyright (c) 2015 CNRS
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

MACRO (ADD_HEADERS_GROUP NAME LIST_OF_FILES)
  SOURCE_GROUP ( "Header Files\\${NAME}" FILES ${${LIST_OF_FILES}} )
ENDMACRO (ADD_HEADERS_GROUP) 

MACRO (ADD_HEADERS_SUB_GROUP GROUP SUB_GROUP LIST_OF_FILES)
  SOURCE_GROUP ( "Header Files\\${GROUP}\\${SUB_GROUP}" FILES ${${LIST_OF_FILES}} )
ENDMACRO (ADD_HEADERS_SUB_GROUP) 

MACRO (ADD_SOURCES_GROUP NAME LIST_OF_FILES)
  SOURCE_GROUP ( "Source Files\\${NAME}" FILES ${${LIST_OF_FILES}} )
ENDMACRO (ADD_SOURCES_GROUP) 

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
  TARGET_LINK_LIBRARIES(${PYWRAP} ${Boost_LIBRARIES} eigenpy)
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

IF (UNIX )  
    # Create target libpinocchio.so
    ADD_LIBRARY ( ${PROJECT_NAME} SHARED ${HEADERS} ${${PROJECT_NAME}_SOURCES} )
		SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    
		ADD_HEADERS_GROUP ("math" ${PROJECT_NAME}_MATH_HEADERS )  

    # Install target libpinocchio.so
    INSTALL(FILES
      "${CMAKE_BINARY_DIR}/src/lib${PROJECT_NAME}.so"
      DESTINATION ${CMAKE_INSTALL_LIBDIR})

    ADD_HEADERS_GROUP ("tools" ${PROJECT_NAME}_TOOLS_HEADERS )
    ADD_HEADERS_GROUP ("spatial" ${PROJECT_NAME}_SPATIAL_HEADERS )
    ADD_HEADERS_GROUP ("multibody" ${PROJECT_NAME}_MULTIBODY_HEADERS )
    ADD_HEADERS_SUB_GROUP ("multibody" "joint" ${PROJECT_NAME}_MULTIBODY_JOINT_HEADERS )
    ADD_HEADERS_GROUP ("algorithm" ${PROJECT_NAME}_ALGORITHM_HEADERS )
    ADD_HEADERS_GROUP ("python" ${PROJECT_NAME}_PYTHON_HEADERS )

ENDIF ( )
