#
# Copyright (c) 2015-2024 CNRS INRIA
#

# Compute flags outside the macro to avoid recomputing it for each tests
cxx_flags_by_compiler_frontend(MSVC _USE_MATH_DEFINES OUTPUT EXAMPLE_PRIVATE_DEFINITIONS)

function(ADD_PINOCCHIO_CPP_EXAMPLE EXAMPLE)
  set(options PARSERS CPPAD CPPADCG CASADI)
  set(oneValueArgs)
  set(multiValueArgs)
  cmake_parse_arguments(example "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME)
  set(EXAMPLE_NAME "${PROJECT_NAME}-example-cpp-${EXAMPLE_NAME}")
  add_unit_test(${EXAMPLE_NAME} "${EXAMPLE}.cpp")
  target_link_libraries(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME})

  if(example_PARSERS)
    target_link_libraries(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_parsers)
  endif()

  if(example_CPPAD)
    target_link_libraries(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_cppad)
  endif()

  if(example_CPPADCG)
    target_link_libraries(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_cppadcg)
  endif()

  if(example_CASADI)
    target_link_libraries(${EXAMPLE_NAME} PUBLIC ${PROJECT_NAME}_casadi)
  endif()

  target_compile_definitions(${EXAMPLE_NAME} PRIVATE ${EXAMPLE_PRIVATE_DEFINITIONS}
                                                     PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")

  # There is no RPATH in Windows, then we must use the PATH to find the DLL
  if(WIN32)
    string(REPLACE ";" "\\\;" _PATH "$ENV{PATH}")
    set(ENV_VARIABLES
        "PATH=${_PATH}\\\;${PROJECT_BINARY_DIR}/src\\\;${PROJECT_BINARY_DIR}/bindings/python/pinocchio"
    )
    set_tests_properties(${EXAMPLE_NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
  endif()
endfunction()

add_pinocchio_cpp_example(inverse-kinematics)
add_pinocchio_cpp_example(inverse-kinematics-3d)
add_pinocchio_cpp_example(overview-simple)
add_pinocchio_cpp_example(overview-lie)
add_pinocchio_cpp_example(overview-SE3)
add_pinocchio_cpp_example(interpolation-SE3)

if(BUILD_WITH_URDF_SUPPORT)
  add_pinocchio_cpp_example(overview-urdf PARSERS)
  add_pinocchio_cpp_example(build-reduced-model PARSERS)
  add_pinocchio_cpp_example(geometry-models PARSERS)
  add_pinocchio_cpp_example(kinematics-derivatives PARSERS)
  add_pinocchio_cpp_example(forward-dynamics-derivatives PARSERS)
  add_pinocchio_cpp_example(inverse-dynamics-derivatives PARSERS)
  if(BUILD_ADVANCED_TESTING)
    add_pinocchio_cpp_example(multiprecision PARSERS)
  endif()
endif()

if(BUILD_WITH_COLLISION_SUPPORT)
  if(BUILD_WITH_URDF_SUPPORT)
    add_pinocchio_cpp_example(collisions PARSERS)
  endif()
endif()

if(BUILD_PYTHON_INTERFACE)
  set(${PROJECT_NAME}_PYTHON_EXAMPLES
      inverse-kinematics inverse-kinematics-3d overview-simple kinematics-derivatives
      forward-dynamics-derivatives inverse-dynamics-derivatives)

  if(BUILD_WITH_URDF_SUPPORT)
    list(
      APPEND
      ${PROJECT_NAME}_PYTHON_EXAMPLES
      overview-urdf
      gepetto-viewer
      build-reduced-model
      robot-wrapper-viewer
      geometry-models)
  endif()

  if(BUILD_WITH_COLLISION_SUPPORT)
    list(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES sample-model-viewer display-shapes
         simulation-pendulum simulation-closed-kinematic-chains)
    if(BUILD_WITH_URDF_SUPPORT)
      list(
        APPEND
        ${PROJECT_NAME}_PYTHON_EXAMPLES
        meshcat-viewer
        meshcat-viewer-dae
        collisions
        collision-with-point-clouds
        append-urdf-model-with-another-model
        simulation-contact-dynamics
        # talos-simulation
        static-contact-dynamics)
    endif()
  endif()

  if(BUILD_WITH_OPENMP_SUPPORT)
    list(APPEND ${PROJECT_NAME}_PYTHON_EXAMPLES run-algo-in-parallel)
  endif()

  foreach(EXAMPLE ${${PROJECT_NAME}_PYTHON_EXAMPLES})
    set(EXAMPLE_NAME "${PROJECT_NAME}-example-py-${EXAMPLE}")
    add_python_unit_test("${EXAMPLE_NAME}" "examples/${EXAMPLE}.py" "bindings/python")
    add_windows_dll_path_to_test(${EXAMPLE_NAME})
  endforeach()
endif()

add_subdirectory(casadi)
add_subdirectory(cppad)
add_subdirectory(codegen)
