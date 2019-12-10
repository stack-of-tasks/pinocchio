#
# Copyright (c) 2015-2019 CNRS INRIA
#

SET(${PROJECT_NAME}_EXAMPLES
  inverse-kinematics
  overview-simple
  overview-lie
  overview-SE3
  overview-urdf
  interpolation-SE3
  build-reduced-model
  geometry-models
  )

ADD_DEFINITIONS(-DPINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")

FOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})
  SET(EXAMPLE_NAME "example-${EXAMPLE}")
  ADD_UNIT_TEST(${EXAMPLE_NAME} ${EXAMPLE})
  TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} ${PROJECT_NAME})
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
ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})

IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_EXAMPLES
    inverse-kinematics
    overview-simple
    overview-urdf
    kinematics-derivatives
    fd-derivatives
    id-derivatives
    gepetto-viewer
    meshcat-viewer
    robot-wrapper-viewer
    geometry-models
    )

  IF(HPP_FCL_FOUND)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES 
      collisions
      )
  ENDIF(HPP_FCL_FOUND)

  FOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
    ADD_PYTHON_UNIT_TEST("py-example-${EXAMPLE}" "examples/${EXAMPLE}.py" "bindings/python")
  ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
ENDIF(BUILD_PYTHON_INTERFACE)
