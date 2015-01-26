MACRO (ADD_HEADERS_GROUP NAME LIST_OF_FILES)
  SOURCE_GROUP ( "Header Files\\${NAME}" FILES ${${LIST_OF_FILES}} )
ENDMACRO (ADD_HEADERS_GROUP) 

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


IF ( ${CMAKE_GENERATOR} MATCHES "Unix Makefiles" )

ELSE ( ${CMAKE_GENERATOR} MATCHES "Unix Makefiles" )

    ADD_LIBRARY ( ${PROJECT_NAME} SHARED ${HEADERS} ${${PROJECT_NAME}_SOURCES} )
		SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    
		ADD_HEADERS_GROUP ("math" ${PROJECT_NAME}_MATH_HEADERS )  
    ADD_HEADERS_GROUP ("tools" ${PROJECT_NAME}_TOOLS_HEADERS )
    ADD_HEADERS_GROUP ("spatial" ${PROJECT_NAME}_SPATIAL_HEADERS )
    ADD_HEADERS_GROUP ("multibody" ${PROJECT_NAME}_MULTIBODY_HEADERS )
    ADD_HEADERS_GROUP ("algorithm" ${PROJECT_NAME}_ALGORITHM_HEADERS )
    ADD_HEADERS_GROUP ("python" ${PROJECT_NAME}_PYTHON_HEADERS )

ENDIF ( ${CMAKE_GENERATOR} MATCHES "Unix Makefiles" )
