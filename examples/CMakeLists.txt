#
# Copyright (c) 2015-2020 CNRS INRIA
#

FUNCTION(ADD_PINOCCHIO_CPP_EXAMPLE EXAMPLE)
  GET_FILENAME_COMPONENT(EXAMPLE_NAME ${EXAMPLE} NAME)
  SET(EXAMPLE_NAME "example-cpp-${EXAMPLE_NAME}")
  ADD_UNIT_TEST(${EXAMPLE_NAME} ${EXAMPLE})
  TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(${EXAMPLE_NAME} eigen3)
  IF(URDFDOM_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${EXAMPLE_NAME} urdfdom)
  ENDIF(URDFDOM_FOUND)
  IF(HPP_FCL_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${EXAMPLE_NAME} hpp-fcl)
  ENDIF(HPP_FCL_FOUND)
  IF(CPPAD_FOUND)
    PKG_CONFIG_USE_DEPENDENCY(${EXAMPLE_NAME} "cppad")
  ENDIF(CPPAD_FOUND)
ENDFUNCTION()

SET(${PROJECT_NAME}_EXAMPLES
  inverse-kinematics
  overview-simple
  overview-lie
  overview-SE3
  interpolation-SE3
  )

IF(BUILD_WITH_URDF_SUPPORT)
  LIST(APPEND ${PROJECT_NAME}_EXAMPLES
    overview-urdf
    build-reduced-model
    geometry-models
    kinematics-derivatives
    forward-dynamics-derivatives
    inverse-dynamics-derivatives
  )
ENDIF(BUILD_WITH_URDF_SUPPORT)
  
IF(BUILD_ADVANCED_TESTING)
  LIST(APPEND ${PROJECT_NAME}_EXAMPLES
    multiprecision
    )
ENDIF(BUILD_ADVANCED_TESTING)

IF(HPP_FCL_FOUND)
  LIST(APPEND ${PROJECT_NAME}_EXAMPLES
    collisions
    )
ENDIF(HPP_FCL_FOUND)

ADD_DEFINITIONS(-DPINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")

FOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})
  ADD_PINOCCHIO_CPP_EXAMPLE(${EXAMPLE})
ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})

IF(BUILD_ADVANCED_TESTING)
  SET_PROPERTY(TARGET example-cpp-multiprecision PROPERTY CXX_STANDARD 11)
ENDIF(BUILD_ADVANCED_TESTING)

IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_EXAMPLES
    inverse-kinematics
    overview-simple
    overview-urdf
    kinematics-derivatives
    forward-dynamics-derivatives
    inverse-dynamics-derivatives
    gepetto-viewer
    meshcat-viewer
    meshcat-viewer-dae
    robot-wrapper-viewer
    geometry-models
    )

  IF(HPP_FCL_FOUND)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES 
      collisions
      sample-model-viewer
      display-shapes
      )
  ENDIF(HPP_FCL_FOUND)

  FOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
    ADD_PYTHON_UNIT_TEST("example-py-${EXAMPLE}" "examples/${EXAMPLE}.py" "bindings/python")
  ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(codegen)
