#
# Copyright (c) 2015-2022 CNRS INRIA
#

FUNCTION(ADD_PINOCCHIO_CPP_EXAMPLE EXAMPLE)
  SET(options PARSERS CPPAD CPPADCG CASADI)
  SET(oneValueArgs)
  SET(multiValueArgs)
  CMAKE_PARSE_ARGUMENTS(example "${options}" "${oneValueArgs}"
    "${multiValueArgs}" ${ARGN})

  GET_FILENAME_COMPONENT(EXAMPLE_NAME ${EXAMPLE} NAME)
  SET(EXAMPLE_NAME "example-cpp-${EXAMPLE_NAME}")
  ADD_UNIT_TEST(${EXAMPLE_NAME} "${EXAMPLE}.cpp")
  TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME})

  IF(example_PARSERS)
    TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_parsers)
  ENDIF()

  IF(example_CPPAD)
    TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_cppad)
  ENDIF()

  IF(example_CPPADCG)
    TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_cppadcg)
  ENDIF()

  IF(example_CASADI)
    TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_casadi)
  ENDIF()

  TARGET_COMPILE_DEFINITIONS(${EXAMPLE_NAME} PRIVATE PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")

  IF(WIN32)
    TARGET_COMPILE_DEFINITIONS(${EXAMPLE_NAME} PRIVATE NOMINMAX _USE_MATH_DEFINES)
  ENDIF()

  # There is no RPATH in Windows, then we must use the PATH to find the DLL
  IF(WIN32)
    STRING(REPLACE ";" "\\\;" _PATH "$ENV{PATH}")
    SET(ENV_VARIABLES "PATH=${_PATH}\\\;${CMAKE_BINARY_DIR}/src\\\;${CMAKE_BINARY_DIR}/bindings/python/pinocchio")
    SET_TESTS_PROPERTIES(${EXAMPLE_NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
  ENDIF()
ENDFUNCTION()

ADD_PINOCCHIO_CPP_EXAMPLE(inverse-kinematics)
ADD_PINOCCHIO_CPP_EXAMPLE(overview-simple)
ADD_PINOCCHIO_CPP_EXAMPLE(overview-lie)
ADD_PINOCCHIO_CPP_EXAMPLE(overview-SE3)
ADD_PINOCCHIO_CPP_EXAMPLE(interpolation-SE3)

IF(BUILD_WITH_URDF_SUPPORT)
  ADD_PINOCCHIO_CPP_EXAMPLE(overview-urdf PARSERS)
  ADD_PINOCCHIO_CPP_EXAMPLE(build-reduced-model PARSERS)
  ADD_PINOCCHIO_CPP_EXAMPLE(geometry-models PARSERS)
  ADD_PINOCCHIO_CPP_EXAMPLE(kinematics-derivatives PARSERS)
  ADD_PINOCCHIO_CPP_EXAMPLE(forward-dynamics-derivatives PARSERS)
  ADD_PINOCCHIO_CPP_EXAMPLE(inverse-dynamics-derivatives PARSERS)
  IF(BUILD_ADVANCED_TESTING)
    ADD_PINOCCHIO_CPP_EXAMPLE(multiprecision PARSERS)
  ENDIF()
ENDIF()

IF(BUILD_WITH_COLLISION_SUPPORT)
  IF(BUILD_WITH_URDF_SUPPORT)
    ADD_PINOCCHIO_CPP_EXAMPLE(collisions PARSERS)
  ENDIF()
ENDIF()

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
  ENDIF()

  IF(BUILD_WITH_COLLISION_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      sample-model-viewer
      display-shapes
      simulation-pendulum
      )
    IF(BUILD_WITH_URDF_SUPPORT)
      LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
        meshcat-viewer
        collisions
        collision-with-point-clouds
        append-urdf-model-with-another-model
        simulation-contact-dynamics
        )
      IF(PYTHON_VERSION_MAJOR EQUAL 3)
        LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
          static-contact-dynamics
          )
      ENDIF()
    ENDIF()
  ENDIF()

  IF(BUILD_WITH_OPENMP_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      run-algo-in-parallel
      )
  ENDIF()

  IF(BUILD_WITH_CASADI_SUPPORT)
    LIST(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES
      casadi-quadrotor-ocp
      )
  ENDIF()

  FOREACH(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
    SET(EXAMPLE_NAME "example-py-${EXAMPLE}")
    ADD_PYTHON_UNIT_TEST("${EXAMPLE_NAME}" "examples/${EXAMPLE}.py" "bindings/python")
    ADD_WINDOWS_DLL_PATH_TO_TEST(${EXAMPLE_NAME})
  ENDFOREACH()
ENDIF()

ADD_SUBDIRECTORY(autodiff)
ADD_SUBDIRECTORY(codegen)
