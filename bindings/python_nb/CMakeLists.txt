#
# Copyright (c) 2025 INRIA
#

include(${JRL_CMAKE_MODULES}/python-helpers.cmake)
include(${JRL_CMAKE_MODULES}/stubs.cmake)
set(CMAKE_MODULE_PATH "${JRL_CMAKE_MODULES}/find-external/GMP"
                      "${JRL_CMAKE_MODULES}/find-external/MPFR" ${CMAKE_MODULE_PATH})

# --- PYTHON TARGET --- #
set(PYWRAP_NB ${PROJECT_NAME}_pywrap_nb)
set(PYWRAP_NB
    ${PYWRAP_NB}
    PARENT_SCOPE)

# --- COMPILE WRAPPER
make_directory("${${PROJECT_NAME}_BINARY_DIR}/bindings/python_nb/${PROJECT_NAME}")
set(${PYWRAP_NB}_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_NB_SOURCES})
set(${PYWRAP_NB}_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_NB_HEADERS})

function(PINOCCHIO_PYTHON_NB_BINDINGS_SPECIFIC_TYPE scalar_name)
  set(scalar_name "${scalar_name}")
  set(PYTHON_LIB_NAME_NB "${PYWRAP_NB}_${scalar_name}")

  string(TOUPPER ${scalar_name} upper_scalar_name) # TODO : Extra content may not be nneeded (just
                                                   # Boost specific content)
  set(EXTRA_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_NB_EXTRA_${upper_scalar_name}_SOURCES})
  set(EXTRA_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_NB_EXTRA_${upper_scalar_name}_PUBLIC_HEADERS})

  set(${PYTHON_LIB_NAME_NB}_SOURCES ${${PYWRAP_NB}_SOURCES} ${EXTRA_SOURCES})
  set(${PYTHON_LIB_NAME_NB}_HEADERS ${${PYWRAP_NB}_HEADERS} ${EXTRA_HEADERS})

  add_library(${PYTHON_LIB_NAME_NB} SHARED ${${PYTHON_LIB_NAME_NB}_SOURCES}
                                           ${${PYTHON_LIB_NAME_NB}_HEADERS})

  # # Links against accelerate # TODO : Test with Apple device if(BUILD_WITH_ACCELERATE_SUPPORT)
  # target_link_libraries(${PYTHON_LIB_NAME_NB} PRIVATE "-framework accelerate")
  # endif(BUILD_WITH_ACCELERATE_SUPPORT)

  if(BUILD_WITH_OPENMP_SUPPORT)
    # OpenMP is linked with pinocchio_parallel target
    target_compile_definitions(${PYTHON_LIB_NAME_NB}
                               PRIVATE -DPINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP)
  endif()
  add_dependencies(${PROJECT_NAME}-python_nb ${PYTHON_LIB_NAME_NB})

  set_target_properties(
    ${PYTHON_LIB_NAME_NB}
    PROPERTIES PREFIX ""
               DEFINE_SYMBOL "${PYWRAP_NB}_EXPORTS"
               CXX_VISIBILITY_PRESET hidden
               VISIBILITY_INLINES_HIDDEN ON)

  cxx_flags_by_compiler_frontend(
    GNU -Wno-conversion -Wno-comment -Wno-self-assign-overloaded -Xclang=-fno-pch-timestamp
    MSVC -Xclang=-fno-pch-timestamp
    OUTPUT PRIVATE_OPTIONS
    FILTER)
  target_compile_options(${PYTHON_LIB_NAME_NB} PRIVATE ${PRIVATE_OPTIONS})

  set(PINOCCHIO_PYTHON_NB_CONTEXT_FILE_VALUE
      "pinocchio/bindings/python_nb/context/${scalar_name}.hpp") # TODO : Create this file
  target_compile_definitions(
    ${PYTHON_LIB_NAME_NB}
    PRIVATE PINOCCHIO_PYTHON_NB_CONTEXT_FILE="${PINOCCHIO_PYTHON_NB_CONTEXT_FILE_VALUE}"
            PINOCCHIO_PYTHON_NB_MODULE_NAME=${PYTHON_LIB_NAME_NB})

  set_target_properties(${PYTHON_LIB_NAME_NB} PROPERTIES VERSION ${PROJECT_VERSION})
  if(BUILD_WITH_COMMIT_VERSION)
    tag_library_version(${PYTHON_LIB_NAME_NB})
  endif(BUILD_WITH_COMMIT_VERSION)
  add_header_group(${PYTHON_LIB_NAME_NB}_HEADERS) # TODO : Check if needed with nanobind (not needed
                                                  # # in coal for eg)
  add_source_group(${PYTHON_LIB_NAME_NB}_SOURCES)

  # modernize_target_link_libraries( # TODO : conda or from source ? (due to register function)
  # ${PYTHON_LIB_NAME_NB} SCOPE PUBLIC TARGETS nanoeigenpy::nanoeigenpy)

  # Beginning of the file from coal (to adapt)
  if(CMAKE_VERSION VERSION_LESS 3.18)
    set(DEV_MODULE Development)
  else()
    set(DEV_MODULE Development.Module)
  endif()

  # Python
  find_package(
    Python
    COMPONENTS Interpreter ${DEV_MODULE}
    REQUIRED)

  # Detect the installed nanobind package and import it into CMake
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)
  find_package(nanoeigenpy CONFIG REQUIRED)

  target_link_libraries(${PYTHON_LIB_NAME_NB} PUBLIC ${PROJECT_NAME})

  if(BUILD_WITH_URDF_SUPPORT)
    # Link directly against console_bridge since we bind some enums and call
    # console_bridge::setLogLevel function.
    modernize_target_link_libraries(
      ${PYTHON_LIB_NAME_NB}
      SCOPE PUBLIC
      TARGETS console_bridge::console_bridge
      LIBRARIES ${console_bridge_LIBRARIES}
      INCLUDE_DIRS ${console_bridge_INCLUDE_DIRS})
  endif()
  if(BUILD_WITH_HPP_FCL_SUPPORT)
    target_compile_definitions(${PYTHON_LIB_NAME_NB}
                               PRIVATE -DPINOCCHIO_PYTHON_NB_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS)
  endif()
  if(WIN32)
    target_link_libraries(${PYTHON_LIB_NAME_NB} PUBLIC ${PYTHON_LIBRARY})
  endif()

  set(${PYWRAP_NB}_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME}) # TODO : Check if no
                                                                           # installation conflicts
                                                                           # if built previously
                                                                           # with Boost.Python
                                                                           # (design properly to
                                                                           # build one, then another
                                                                           # easily)

  set_target_properties(
    # TODO : Same + check if could be lightened (as in coal for nanobind)
    ${PYTHON_LIB_NAME_NB}
    PROPERTIES PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX}
               OUTPUT_NAME "${PYTHON_LIB_NAME_NB}"
               LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python_nb/${PROJECT_NAME}"
               # On Windows, shared library are treat as binary
               RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python_nb/${PROJECT_NAME}")

  if(UNIX)
    get_relative_rpath(${${PYWRAP_NB}_INSTALL_DIR} ${PYWRAP_NB}_INSTALL_RPATH)
    set_target_properties(${PYTHON_LIB_NAME_NB} PROPERTIES INSTALL_RPATH
                                                           "${${PYWRAP_NB}_INSTALL_RPATH}")
  endif()

  install(
    TARGETS ${PYTHON_LIB_NAME_NB}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${PINOCCHIO_PYTHON_NB_INSTALL_DIR})
endfunction()

function(INSTALL_PYTHON_FILES)
  set(options)
  set(oneValueArgs MODULE)
  set(multiValueArgs FILES)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(SOURCE_PATH ${PROJECT_NAME})
  set(INSTALL_PATH ${PINOCCHIO_PYTHON_NB_INSTALL_DIR})
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
  add_custom_target(${PROJECT_NAME}-python_nb)
  set_target_properties(${PROJECT_NAME}-python_nb PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

  python_build_get_target(python_build_target)
  add_dependencies(${PROJECT_NAME}-python_nb ${python_build_target})

  # set(PKG_CONFIG_PYWRAP_NB_REQUIRES "nanoeigenpy >= 0.1.0")                         # TODO : conda
  # or from source ? (due to register function)
  if(IS_ABSOLUTE ${PYTHON_SITELIB}) # TODO : Check if can be lightened
    set(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
  else()
    set(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
  endif()
  set(PINOCCHIO_PYTHON_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})

  pinocchio_python_nb_bindings_specific_type(default)

  # target_precompile_headers(${PYWRAP}_default PRIVATE                             # TODO : If I
  # use precompiled headers ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python_nb/pch.hpp)

  set(PYTHON_LIB_NAME_NB "${PYWRAP_NB}_default")
  set(STUBGEN_DEPENDENCIES "${PYWRAP_NB}_default")

  if(BUILD_WITH_AUTODIFF_SUPPORT)

  endif(BUILD_WITH_AUTODIFF_SUPPORT)

  if(BUILD_WITH_CODEGEN_SUPPORT)

  endif(BUILD_WITH_CODEGEN_SUPPORT)

  if(BUILD_WITH_CASADI_SUPPORT)

  endif(BUILD_WITH_CASADI_SUPPORT)

  if(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)

  endif(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)

  if(BUILD_WITH_AUTODIFF_SUPPORT)

  endif(BUILD_WITH_AUTODIFF_SUPPORT)

  if(BUILD_WITH_CODEGEN_SUPPORT)

  endif(BUILD_WITH_CODEGEN_SUPPORT)

  if(BUILD_WITH_CASADI_SUPPORT)

  endif(BUILD_WITH_CASADI_SUPPORT)

  if(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)

  endif(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)

  # --- INSTALL SCRIPTS                                       # TODO : Adapt to the new files
  install_python_files(
    # FILES __init__.py deprecated.py deprecation.py utils.py robot_wrapper.py romeo_wrapper.py
    # explog.py shortcuts.py windows_dll_manager.py
  )

  # --- INSTALL DERIVATIVE SCRIPTS                            # TODO : Adapt to the new files
  install_python_files(# MODULE derivative FILES xm.py dcrba.py lambdas.py
  )

  # --- INSTALL VISUALIZATION SCRIPTS                          # TODO : Adapt to the new files
  install_python_files(
    # MODULE visualize FILES __init__.py base_visualizer.py gepetto_visualizer.py
    # meshcat_visualizer.py panda3d_visualizer.py rviz_visualizer.py visualizers.py
  )

  # --- STUBS --- # TODO : Version lightended with nanobind (cf what is done in coal)

  # --- PACKAGING --- #

  # Format string
  set(_PKG_CONFIG_PYWRAP_NB_LIBDIR ${PINOCCHIO_PYTHON_NB_INSTALL_DIR})
  set(_PKG_CONFIG_PYWRAP_NB_BINDIR ${PINOCCHIO_PYTHON_NB_INSTALL_DIR})
  set(_PKG_CONFIG_PYWRAP_NB_CONFLICTS)
  set(_PKG_CONFIG_PYWRAP_NB_REQUIRES "${PROJECT_NAME}")
  foreach(dep ${PKG_CONFIG_PYWRAP_NB_REQUIRES})
    set(_PKG_CONFIG_PYWRAP_NB_REQUIRES "${_PKG_CONFIG_PYWRAP_NB_REQUIRES}, ${dep}")
  endforeach(dep ${PKG_CONFIG_PYWRAP_NB_REQUIRES})

  set(_PKG_CONFIG_PYWRAP_NB_LIBS "-L\${libdir} -l${PYWRAP_NB_}")
  if(APPLE)
    set(_PKG_CONFIG_PYWRAP_NB_LIBS
        "${_PKG_CONFIG_PYWRAP_NB_LIBS} -Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY}"
    )
  else(APPLE)
    set(_PKG_CONFIG_PYWRAP_NB_LIBS "${_PKG_CONFIG_PYWRAP_NB_LIBS} ${LIBINCL_KW}boost_python")
  endif(APPLE)

  set(_PKG_CONFIG_PYWRAP_NB_CFLAGS "-I\${includedir}")
  set(_PKG_CONFIG_PYWRAP_NB_CFLAGS "${_PKG_CONFIG_PYWRAP_NB_CFLAGS} -I${PYTHON_INCLUDE_DIRS}")
  foreach(cflags ${CFLAGS_DEPENDENCIES})
    set(_PKG_CONFIG_PYWRAP_NB_CFLAGS "${_PKG_CONFIG_PYWRAP_NB_CFLAGS} ${cflags}")
  endforeach(cflags ${CFLAGS_DEPENDENCIES})
  foreach(cflags ${CFLAGS_OPTIONS})
    set(_PKG_CONFIG_PYWRAP_NB_CFLAGS "${_PKG_CONFIG_PYWRAP_NB_CFLAGS} ${cflags}")
  endforeach()

  # configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pinocchiopy.pc.cmake"                     # TODO :
  # Make sure it is the one from python_nb/ "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc")

  # install( FILES "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc" DESTINATION
  # ${CMAKE_INSTALL_LIBDIR}/pkgconfig PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

  # TODO : Doxygen

endif()
