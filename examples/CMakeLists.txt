#
# Copyright (c) 2015-2022 CNRS INRIA
#

FUNCTION(ADD_PINOCCHIO_CPP_EXAMPLE EXAMPLE)
  GET_FILENAME_COMPONENT(EXAMPLE_NAME ${EXAMPLE} NAME)
  SET(EXAMPLE_NAME "example-cpp-${EXAMPLE_NAME}")
  ADD_UNIT_TEST(${EXAMPLE_NAME} "${EXAMPLE}.cpp")
  TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME})
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
ENDIF()

IF(BUILD_ADVANCED_TESTING)
  IF(BUILD_WITH_URDF_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_EXAMPLES
      multiprecision
      )
  ENDIF()
ENDIF(BUILD_ADVANCED_TESTING)

IF(hpp-fcl_FOUND)
  IF(BUILD_WITH_URDF_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_EXAMPLES
      collisions
      )
  ENDIF()
ENDIF(hpp-fcl_FOUND)

ADD_DEFINITIONS(-DPINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")

IF(WIN32)
  ADD_DEFINITIONS("-DNOMINMAX -D_USE_MATH_DEFINES")
ENDIF(WIN32)

FOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})
  ADD_PINOCCHIO_CPP_EXAMPLE(${EXAMPLE})
ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_EXAMPLES})

IF(BUILD_ADVANCED_TESTING AND BUILD_WITH_URDF_SUPPORT)
  SET_PROPERTY(TARGET example-cpp-multiprecision PROPERTY CXX_STANDARD 11)
ENDIF(BUILD_ADVANCED_TESTING AND BUILD_WITH_URDF_SUPPORT)

IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_EXAMPLES
    inverse-kinematics
    overview-simple
    kinematics-derivatives
    forward-dynamics-derivatives
    inverse-dynamics-derivatives
    )

  IF(BUILD_WITH_URDF_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      overview-urdf
      gepetto-viewer
      build-reduced-model
      meshcat-viewer-dae
      robot-wrapper-viewer
      geometry-models
      )
  ENDIF(BUILD_WITH_URDF_SUPPORT)

  IF(hpp-fcl_FOUND)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      sample-model-viewer
      display-shapes
      simulation-pendulum
      )
    IF(BUILD_WITH_URDF_SUPPORT)
      LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
        collisions
        collision-with-point-clouds
        append-urdf-model-with-another-model
        meshcat-viewer
        )
      IF(PYTHON_VERSION_MAJOR EQUAL 3)
        LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
          static-contact-dynamics
          )
      ENDIF()
    ENDIF(BUILD_WITH_URDF_SUPPORT)
  ENDIF(hpp-fcl_FOUND)

  IF(BUILD_WITH_OPENMP_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      run-algo-in-parallel
      )
  ENDIF(BUILD_WITH_OPENMP_SUPPORT)

  FOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
    ADD_PYTHON_UNIT_TEST("example-py-${EXAMPLE}" "examples/${EXAMPLE}.py" "bindings/python")
  ENDFOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(codegen)
