#
# Copyright (c) 2015-2023 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

include(${JRL_CMAKE_MODULES}/python-helpers.cmake)
include(${JRL_CMAKE_MODULES}/stubs.cmake)
set(CMAKE_MODULE_PATH "${JRL_CMAKE_MODULES}/find-external/GMP"
                      "${JRL_CMAKE_MODULES}/find-external/MPFR" ${CMAKE_MODULE_PATH})

# --- PYTHON TARGET --- #
set(PYWRAP ${PROJECT_NAME}_pywrap)
set(PYWRAP
    ${PYWRAP}
    PARENT_SCOPE)

# --- COMPILE WRAPPER
make_directory("${${PROJECT_NAME}_BINARY_DIR}/bindings/python/${PROJECT_NAME}")
set(${PYWRAP}_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES})
set(${PYWRAP}_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS})

function(PINOCCHIO_PYTHON_BINDINGS_SPECIFIC_TYPE scalar_name)
  set(scalar_name "${scalar_name}")
  set(PYTHON_LIB_NAME "${PYWRAP}_${scalar_name}")

  string(TOUPPER ${scalar_name} upper_scalar_name)
  set(EXTRA_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_${upper_scalar_name}_SOURCES})
  set(EXTRA_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_${upper_scalar_name}_PUBLIC_HEADERS})

  set(${PYTHON_LIB_NAME}_SOURCES ${${PYWRAP}_SOURCES} ${EXTRA_SOURCES})
  set(${PYTHON_LIB_NAME}_HEADERS ${${PYWRAP}_HEADERS} ${EXTRA_HEADERS})

  add_library(${PYTHON_LIB_NAME} SHARED ${${PYTHON_LIB_NAME}_SOURCES} ${${PYTHON_LIB_NAME}_HEADERS})

  # Links against accelerate
  if(BUILD_WITH_ACCELERATE_SUPPORT)
    # modernize_target_link_libraries(${PYTHON_LIB_NAME} SCOPE PUBLIC TARGETS Accelerate)
    target_link_libraries(${PYTHON_LIB_NAME} PRIVATE "-framework accelerate")
  endif(BUILD_WITH_ACCELERATE_SUPPORT)

  if(BUILD_WITH_OPENMP_SUPPORT)
    # OpenMP is linked with pinocchio_parallel target
    target_compile_definitions(${PYTHON_LIB_NAME} PRIVATE -DPINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP)
  endif()
  add_dependencies(${PROJECT_NAME}-python ${PYTHON_LIB_NAME})

  # Remove lib prefix for the target and use the right define for DLLAPI definition.
  #
  # Set inlined function visibility to hidden by default. This avoid bug on OSX where two symbol
  # from two different binding (`pinocchio::python::exposeSpecificTypeFeatures`) can be wrongly
  # loaded (see https://github.com/stack-of-tasks/pinocchio/issues/2462).
  set_target_properties(
    ${PYTHON_LIB_NAME}
    PROPERTIES PREFIX ""
               DEFINE_SYMBOL "${PYWRAP}_EXPORTS"
               CXX_VISIBILITY_PRESET hidden
               VISIBILITY_INLINES_HIDDEN ON)

  # Do not report:
  #
  # * -Wconversion as the BOOST_PYTHON_FUNCTION_OVERLOADS implicitly converts.
  # * -Wcomment as latex equations have multi-line comments.
  # * -Wself-assign-overloaded as bp::self operations trigger this (Clang only).
  cxx_flags_by_compiler_frontend(
    GNU -Wno-conversion -Wno-comment -Wno-self-assign-overloaded
    OUTPUT PRIVATE_OPTIONS
    FILTER)
  target_compile_options(${PYTHON_LIB_NAME} PRIVATE ${PRIVATE_OPTIONS})

  set(PINOCCHIO_PYTHON_CONTEXT_FILE_VALUE "pinocchio/bindings/python/context/${scalar_name}.hpp")
  target_compile_definitions(
    ${PYTHON_LIB_NAME}
    PRIVATE PINOCCHIO_PYTHON_CONTEXT_FILE="${PINOCCHIO_PYTHON_CONTEXT_FILE_VALUE}"
            PINOCCHIO_PYTHON_MODULE_NAME=${PYTHON_LIB_NAME})

  set_target_properties(${PYTHON_LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
  if(BUILD_WITH_COMMIT_VERSION)
    tag_library_version(${PYTHON_LIB_NAME})
  endif(BUILD_WITH_COMMIT_VERSION)
  add_header_group(${PYTHON_LIB_NAME}_HEADERS)
  add_source_group(${PYTHON_LIB_NAME}_SOURCES)

  modernize_target_link_libraries(
    ${PYTHON_LIB_NAME}
    SCOPE PUBLIC
    TARGETS eigenpy::eigenpy)
  target_link_libraries(${PYTHON_LIB_NAME} PUBLIC ${PROJECT_NAME})

  if(BUILD_WITH_URDF_SUPPORT)
    # Link directly against console_bridge since we bind some enums and call
    # console_bridge::setLogLevel function.
    modernize_target_link_libraries(
      ${PYTHON_LIB_NAME}
      SCOPE PUBLIC
      TARGETS console_bridge::console_bridge
      LIBRARIES ${console_bridge_LIBRARIES}
      INCLUDE_DIRS ${console_bridge_INCLUDE_DIRS})
  endif()
  if(BUILD_WITH_HPP_FCL_SUPPORT)
    target_compile_definitions(${PYTHON_LIB_NAME}
                               PRIVATE -DPINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS)
  endif()
  if(WIN32)
    target_link_libraries(${PYTHON_LIB_NAME} PUBLIC ${PYTHON_LIBRARY})
  endif()

  set(${PYWRAP}_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})

  set_target_properties(
    ${PYTHON_LIB_NAME}
    PROPERTIES PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX}
               OUTPUT_NAME "${PYTHON_LIB_NAME}"
               LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python/${PROJECT_NAME}"
               # On Windows, shared library are treat as binary
               RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python/${PROJECT_NAME}")

  if(UNIX)
    get_relative_rpath(${${PYWRAP}_INSTALL_DIR} ${PYWRAP}_INSTALL_RPATH)
    set_target_properties(${PYTHON_LIB_NAME} PROPERTIES INSTALL_RPATH "${${PYWRAP}_INSTALL_RPATH}")
  endif()

  install(
    TARGETS ${PYTHON_LIB_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${PINOCCHIO_PYTHON_INSTALL_DIR})
endfunction()

function(INSTALL_PYTHON_FILES)
  set(options)
  set(oneValueArgs MODULE)
  set(multiValueArgs FILES)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(SOURCE_PATH ${PROJECT_NAME})
  set(INSTALL_PATH ${PINOCCHIO_PYTHON_INSTALL_DIR})
  if(ARGS_MODULE)
    set(SOURCE_PATH ${SOURCE_PATH}/${ARGS_MODULE})
    set(INSTALL_PATH ${INSTALL_PATH}/${ARGS_MODULE})
  endif()

  foreach(f ${ARGS_FILES})
    python_build(${SOURCE_PATH} ${f})
    install(FILES ${SOURCE_PATH}/${f} DESTINATION ${INSTALL_PATH})
  endforeach()
endfunction()

if(BUILD_PYTHON_INTERFACE)
  add_custom_target(${PROJECT_NAME}-python)
  set_target_properties(${PROJECT_NAME}-python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  python_build_get_target(python_build_target)
  add_dependencies(${PROJECT_NAME}-python ${python_build_target})

  set(PKG_CONFIG_PYWRAP_REQUIRES "eigenpy >= 2.6.5")
  if(IS_ABSOLUTE ${PYTHON_SITELIB})
    set(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
  else()
    set(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
  endif()
  set(PINOCCHIO_PYTHON_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})

  pinocchio_python_bindings_specific_type(default)
  target_precompile_headers(${PYWRAP}_default PUBLIC
                            ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pch.hpp)
  set(PYTHON_LIB_NAME "${PYWRAP}_default")
  set(STUBGEN_DEPENDENCIES "${PYWRAP}_default")

  if(BUILD_WITH_AUTODIFF_SUPPORT)
    pinocchio_python_bindings_specific_type(cppad cppad)
    # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
    # This can happen when Pinocchio is build in Debug mode and pycppad is build in Release mode.
    # See
    # https://coin-or.github.io/CppAD/doc/preprocessor.htm#Documented%20Here.CPPAD_DEBUG_AND_RELEASE.
    target_compile_definitions(
      ${PYWRAP}_cppad PRIVATE PYCPPAD_EXCLUDE_EIGEN_NUMTRAITS_SPECIALIZATION
                              CPPAD_DEBUG_AND_RELEASE)
    target_include_directories(${PYWRAP}_cppad SYSTEM
                               PUBLIC $<BUILD_INTERFACE:${cppad_INCLUDE_DIR}>)
    target_link_libraries(${PYWRAP}_cppad PUBLIC ${cppad_LIBRARY})
    list(APPEND STUBGEN_DEPENDENCIES "${PYWRAP}_cppad")

    # --- INSTALL SCRIPTS
    install_python_files(MODULE cppad FILES __init__.py)
  endif(BUILD_WITH_AUTODIFF_SUPPORT)

  if(BUILD_WITH_CODEGEN_SUPPORT)
    pinocchio_python_bindings_specific_type(cppadcg cppadcg)

    # On osx, use default visiblitiy because of an issue with thread_local storage defined in
    # cppad/cg/cg.hpp header (see
    # https://github.com/stack-of-tasks/pinocchio/pull/2469#issuecomment-2461845127)
    if(APPLE)
      set_target_properties(${PYWRAP}_cppadcg PROPERTIES CXX_VISIBILITY_PRESET default)
    endif()

    # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
    target_compile_definitions(
      ${PYWRAP}_cppadcg PRIVATE PYCPPAD_EXCLUDE_EIGEN_NUMTRAITS_SPECIALIZATION
                                CPPAD_DEBUG_AND_RELEASE)
    target_include_directories(${PYWRAP}_cppadcg SYSTEM
                               PUBLIC $<BUILD_INTERFACE:${cppadcg_INCLUDE_DIR}>)
    target_link_libraries(${PYWRAP}_cppadcg PUBLIC ${cppadcg_LIBRARY} ${cppad_LIBRARY})
    list(APPEND STUBGEN_DEPENDENCIES "${PYWRAP}_cppadcg")

    # --- INSTALL SCRIPTS
    install_python_files(MODULE cppadcg FILES __init__.py)
  endif(BUILD_WITH_CODEGEN_SUPPORT)

  if(BUILD_WITH_CASADI_SUPPORT)
    pinocchio_python_bindings_specific_type(casadi)
    target_link_libraries(${PYWRAP}_casadi PUBLIC casadi)
    list(APPEND STUBGEN_DEPENDENCIES "${PYWRAP}_casadi")

    # --- INSTALL SCRIPTS
    install_python_files(MODULE casadi FILES __init__.py)
  endif(BUILD_WITH_CASADI_SUPPORT)

  if(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)
    find_package(GMP REQUIRED 6.0.0)
    find_package(MPFR REQUIRED 4.0.0)
    pinocchio_python_bindings_specific_type(mpfr)

    target_link_libraries(${PYWRAP}_mpfr PRIVATE gmp mpfr)
    list(APPEND STUBGEN_DEPENDENCIES "${PYWRAP}_mpfr")

    # --- INSTALL SCRIPTS
    install_python_files(MODULE mpfr FILES __init__.py)
  endif(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)

  # --- INSTALL SCRIPTS
  install_python_files(
    FILES __init__.py
          deprecated.py
          deprecation.py
          utils.py
          robot_wrapper.py
          romeo_wrapper.py
          explog.py
          shortcuts.py
          windows_dll_manager.py)

  # --- INSTALL DERIVATIVE SCRIPTS
  install_python_files(MODULE derivative FILES xm.py dcrba.py lambdas.py)

  # --- INSTALL VISUALIZATION SCRIPTS
  install_python_files(
    MODULE visualize
    FILES __init__.py
          base_visualizer.py
          gepetto_visualizer.py
          meshcat_visualizer.py
          panda3d_visualizer.py
          rviz_visualizer.py
          visualizers.py)

  # --- STUBS --- #
  if(GENERATE_PYTHON_STUBS)
    python_build_get_target(python_build_target_name)
    load_stubgen()
    # Set PYWRAP and PROJECT_NAME as stubs dependencies.
    #
    # PROJECT_NAME is mandatory (even if it's a PYWRAP dependency) to find PROJECT_NAME name DLL on
    # windows.
    generate_stubs(
      ${CMAKE_CURRENT_BINARY_DIR}
      ${PROJECT_NAME}
      ${ABSOLUTE_PYTHON_SITELIB}
      ${PYWRAP}_default
      ${PROJECT_NAME}_default
      ${STUBGEN_DEPENDENCIES}
      ${python_build_target_name})
  endif(GENERATE_PYTHON_STUBS)

  # --- PACKAGING --- #

  # Format string
  set(_PKG_CONFIG_PYWRAP_LIBDIR ${PINOCCHIO_PYTHON_INSTALL_DIR})
  set(_PKG_CONFIG_PYWRAP_BINDIR ${PINOCCHIO_PYTHON_INSTALL_DIR})
  set(_PKG_CONFIG_PYWRAP_CONFLICTS)
  set(_PKG_CONFIG_PYWRAP_REQUIRES "${PROJECT_NAME}")
  foreach(dep ${PKG_CONFIG_PYWRAP_REQUIRES})
    set(_PKG_CONFIG_PYWRAP_REQUIRES "${_PKG_CONFIG_PYWRAP_REQUIRES}, ${dep}")
  endforeach(dep ${PKG_CONFIG_PYWRAP_REQUIRES})

  set(_PKG_CONFIG_PYWRAP_LIBS "-L\${libdir} -l${PYWRAP}")
  if(APPLE)
    set(_PKG_CONFIG_PYWRAP_LIBS
        "${_PKG_CONFIG_PYWRAP_LIBS} -Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY}"
    )
  else(APPLE)
    set(_PKG_CONFIG_PYWRAP_LIBS "${_PKG_CONFIG_PYWRAP_LIBS} ${LIBINCL_KW}boost_python")
  endif(APPLE)

  set(_PKG_CONFIG_PYWRAP_CFLAGS "-I\${includedir}")
  set(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} -I${PYTHON_INCLUDE_DIRS}")
  foreach(cflags ${CFLAGS_DEPENDENCIES})
    set(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} ${cflags}")
  endforeach(cflags ${CFLAGS_DEPENDENCIES})
  foreach(cflags ${CFLAGS_OPTIONS})
    set(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} ${cflags}")
  endforeach()

  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pinocchiopy.pc.cmake"
                 "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc")

  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

  if(DOXYGEN_FOUND AND DOXYGEN_VERSION VERSION_GREATER 1.8.17)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_LATEX NO)
    set(DOXYGEN_PROJECT_NAME "Pinocchio PyBind11 helpers.")
    set(_source_headers_root "../../include/${PROJECT_NAME}/bindings/python")
    doxygen_add_docs(
      doc_pybind11 ${_source_headers_root}/pybind11.hpp ${_source_headers_root}/pybind11-all.hpp
      USE_STAMP_FILE COMMENT "Generating documentation of the PyBind11 helpers.")
  endif()
endif(BUILD_PYTHON_INTERFACE)
