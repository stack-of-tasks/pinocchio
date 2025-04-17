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
set(${PYWRAP_NB}_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_NB_SOURCES})                             # TODO: Define in sources.cmake
set(${PYWRAP_NB}_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_NB_HEADERS})                      # TODO: Define in sources.cmake

function(PINOCCHIO_PYTHON_NB_BINDINGS_SPECIFIC_TYPE scalar_name)
  set(scalar_name "${scalar_name}")
  set(PYTHON_LIB_NAME_NB "${PYWRAP_NB}_${scalar_name}")

  string(TOUPPER ${scalar_name} upper_scalar_name)
  set(EXTRA_SOURCES ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_${upper_scalar_name}_SOURCES})          # TODO : Define in sources.cmake
  set(EXTRA_HEADERS ${${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_${upper_scalar_name}_PUBLIC_HEADERS})   # TODO : Define in sources.cmake

  set(${PYTHON_LIB_NAME_NB}_SOURCES ${${PYWRAP_NB}_SOURCES} ${EXTRA_SOURCES})
  set(${PYTHON_LIB_NAME_NB}_HEADERS ${${PYWRAP_NB}_HEADERS} ${EXTRA_HEADERS})

  add_library(${PYTHON_LIB_NAME_NB} SHARED ${${PYTHON_LIB_NAME_NB}_SOURCES} ${${PYTHON_LIB_NAME_NB}_HEADERS})

#   # Links against accelerate                                                                      # TODO : Test with Apple device
#   if(BUILD_WITH_ACCELERATE_SUPPORT)
#     target_link_libraries(${PYTHON_LIB_NAME_NB} PRIVATE "-framework accelerate")
#   endif(BUILD_WITH_ACCELERATE_SUPPORT)
       
  if(BUILD_WITH_OPENMP_SUPPORT)                                                                   
    # OpenMP is linked with pinocchio_parallel target
    target_compile_definitions(${PYTHON_LIB_NAME} PRIVATE -DPINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP)
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

  set(PINOCCHIO_PYTHON_NB_CONTEXT_FILE_VALUE "pinocchio/bindings/python_nb/context/${scalar_name}.hpp")
  target_compile_definitions(
    ${PYTHON_LIB_NAME_NB}
    PRIVATE PINOCCHIO_PYTHON_NB_CONTEXT_FILE="${PINOCCHIO_PYTHON_NB_CONTEXT_FILE_VALUE}"
            PINOCCHIO_PYTHON_NB_MODULE_NAME=${PYTHON_LIB_NAME_NB})

  set_target_properties(${PYTHON_LIB_NAME_NB} PROPERTIES VERSION ${PROJECT_VERSION})
  if(BUILD_WITH_COMMIT_VERSION)
    tag_library_version(${PYTHON_LIB_NAME_NB})
  endif(BUILD_WITH_COMMIT_VERSION)
  add_header_group(${PYTHON_LIB_NAME_NB}_HEADERS)                                                   # TODO : Check if needed with nanobind (not needed in coal for eg)
  add_source_group(${PYTHON_LIB_NAME_NB}_SOURCES)

#   modernize_target_link_libraries(                                                                # TODO : To put with nanoeigenpy if needed (eg if conda install nanoeigenpy -c conda-forge)
#     ${PYTHON_LIB_NAME}
#     SCOPE PUBLIC
#     TARGETS eigenpy::eigenpy)
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

  set(${PYWRAP_NB}_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})                          # TODO : Check if no installation conflicts if built previously with Boost.Python (design properly to build one, then another easily)

  set_target_properties(                                                                            # TODO : Same + check if could be lightened (as in coal for nanobind)
    ${PYTHON_LIB_NAME_NB}
    PROPERTIES PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX}
               OUTPUT_NAME "${PYTHON_LIB_NAME_NB}"
               LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python_nb/${PROJECT_NAME}"
               # On Windows, shared library are treat as binary
               RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bindings/python_nb/${PROJECT_NAME}")

  if(UNIX)
    get_relative_rpath(${${PYWRAP_NB}_INSTALL_DIR} ${PYWRAP_NB}_INSTALL_RPATH)
    set_target_properties(${PYTHON_LIB_NAME_NB} PROPERTIES INSTALL_RPATH "${${PYWRAP_NB}_INSTALL_RPATH}")
  endif()

  install(
    TARGETS ${PYTHON_LIB_NAME_NB}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${PINOCCHIO_PYTHON_NB_INSTALL_DIR})
endfunction()

function(INSTALL_PYTHON_NB_FILES)
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

  python_build_get_target(python_build_target)                                                   # TODO : Check if should be python_build_target or python_nb_build_target
  add_dependencies(${PROJECT_NAME}-python_nb ${python_build_target})

#   set(PKG_CONFIG_PYWRAP_REQUIRES "eigenpy >= 2.6.5")                                           # TODO : Equivalent with nnoeigenpy if needed / possible
  if(IS_ABSOLUTE ${PYTHON_SITELIB})                                                              # TODO : In coal nanobind we do not use absolute paths -> maybe lighten it
    set(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
  else()
    set(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
  endif()
  set(PINOCCHIO_PYTHON_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})

  pinocchio_python_nb_bindings_specific_type(default)

#   target_precompile_headers(${PYWRAP}_default PRIVATE                                          # TODO : Give the equivalent if we use rpecompiled headers
#                             ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pch.hpp)
  set(PYTHON_LIB_NAME_NB "${PYWRAP_NB}_default")
  set(STUBGEN_DEPENDENCIES "${PYWRAP_NB}_default")

#   if(BUILD_WITH_AUTODIFF_SUPPORT)                                                              # TODO : Add equivalent of cppad files in nanobind version
#     pinocchio_python_nb_bindings_specific_type(cppad cppad)
#     # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
#     # This can happen when Pinocchio is build in Debug mode and pycppad is build in Release mode.
#     # See
#     # https://coin-or.github.io/CppAD/doc/preprocessor.htm#Documented%20Here.CPPAD_DEBUG_AND_RELEASE.
#     target_compile_definitions(
#       ${PYWRAP_NB}_cppad PRIVATE PYCPPAD_EXCLUDE_EIGEN_NUMTRAITS_SPECIALIZATION
#                               CPPAD_DEBUG_AND_RELEASE)
#     target_include_directories(${PYWRAP_NB}_cppad SYSTEM
#                                PUBLIC $<BUILD_INTERFACE:${cppad_INCLUDE_DIR}>)
#     target_link_libraries(${PYWRAP_NB}_cppad PUBLIC ${cppad_LIBRARY})
#     list(APPEND STUBGEN_DEPENDENCIES "${PYWRAP_NB}_cppad")

#     target_precompile_headers(
#       ${PYWRAP_NB}_cppad PRIVATE ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pch.hpp
#       <cppad/cppad.hpp> <boost/mpl/int.hpp>)

#     # --- INSTALL SCRIPTS
#     install_python_files(MODULE cppad FILES __init__.py)
#   endif(BUILD_WITH_AUTODIFF_SUPPORT)

# TODO : Finish structure of this CMakeLists.txt (and adapt jointly with the main one)



endif(BUILD_PYTHON_INTERFACE)

# TODO : Check if could be lightened with nanobind features
# TODO : For the design of the CMakeLists.txt's, begin with the one from the root, see its architecture, and adapt this one (not reversely, the least possible)