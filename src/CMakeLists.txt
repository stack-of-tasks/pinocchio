#
# Copyright (c) 2015-2023 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#
# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

# Create header-only target All other target will depend on it.
add_library(${PROJECT_NAME}_headers INTERFACE)
# On CMake 3.16, we can't target_sources(${PROJECT_NAME}_headers INTERFACE
# ${${PROJECT_NAME}_CORE_PUBLIC_HEADERS})

# Enforce the preprocessed version of boost::list and boost::vector This information is redundant
# with the content of include/pinocchio/container/boost-container-limits.hpp but it avoids any
# compilation issue.
target_compile_definitions(${PROJECT_NAME}_headers INTERFACE BOOST_MPL_LIMIT_LIST_SIZE=30
                                                             BOOST_MPL_LIMIT_VECTOR_SIZE=30)

if(INITIALIZE_WITH_NAN)
  target_compile_definitions(${PROJECT_NAME}_headers INTERFACE EIGEN_INITIALIZE_MATRICES_BY_NAN)
endif()

if(CHECK_RUNTIME_MALLOC)
  target_compile_definitions(${PROJECT_NAME}_headers INTERFACE PINOCCHIO_EIGEN_CHECK_MALLOC
                                                               EIGEN_RUNTIME_NO_MALLOC)
endif(CHECK_RUNTIME_MALLOC)

modernize_target_link_libraries(
  ${PROJECT_NAME}_headers
  SCOPE INTERFACE
  TARGETS Eigen3::Eigen
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
modernize_target_link_libraries(
  ${PROJECT_NAME}_headers
  SCOPE INTERFACE
  TARGETS Boost::boost Boost::serialization
  LIBRARIES ${Boost_SERIALIZATION_LIBRARY}
  INCLUDE_DIRS ${Boost_INCLUDE_DIRS})

target_include_directories(
  ${PROJECT_NAME}_headers
  INTERFACE $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

cxx_flags_by_compiler_frontend(
  MSVC "/bigobj"
  OUTPUT PUBLIC_OPTIONS
  FILTER)
target_compile_options(${PROJECT_NAME}_headers INTERFACE ${PUBLIC_OPTIONS})

cxx_flags_by_compiler_frontend(MSVC "NOMINMAX" OUTPUT PUBLIC_DEFINITIONS)
target_compile_definitions(${PROJECT_NAME}_headers INTERFACE ${PUBLIC_DEFINITIONS})

install(
  TARGETS ${PROJECT_NAME}_headers
  EXPORT ${TARGETS_EXPORT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Define a compiled target This functions take sources and scalar type to use
function(PINOCCHIO_TARGET target_name)
  set(options INTERFACE)
  set(oneValueArgs SCALAR LIBRARY_PUBLIC_SCOPE)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(LIB_NAME "${target_name}")

  # Manage different scope type if building an interface or shared library
  set(LIBRARY_TYPE SHARED)
  set(LIBRARY_PUBLIC_SCOPE PUBLIC)
  if(ARGS_INTERFACE)
    set(LIBRARY_TYPE INTERFACE)
    set(LIBRARY_PUBLIC_SCOPE INTERFACE)
  endif()

  # Export PUBLIC scope to caller
  if(ARGS_LIBRARY_PUBLIC_SCOPE)
    set(${ARGS_LIBRARY_PUBLIC_SCOPE}
        ${LIBRARY_PUBLIC_SCOPE}
        PARENT_SCOPE)
  endif()

  add_library(${LIB_NAME} ${LIBRARY_TYPE})
  target_link_libraries(${LIB_NAME} ${LIBRARY_PUBLIC_SCOPE} ${PROJECT_NAME}_headers)

  # On CMake 3.16 we can't call target_sources or set_target_properties with LINKER_LANGUAGE,
  # INSTALL_RPATH and VERSION on an INTERFACE target.
  if(NOT ARGS_INTERFACE OR CMAKE_VERSION GREATER 3.16)
    target_sources(${LIB_NAME} PRIVATE ${ARGS_SOURCES})
    target_sources(
      ${LIB_NAME} PRIVATE ${${PROJECT_NAME}_CORE_PUBLIC_HEADERS}) # For IDE to get includes part of
                                                                  # the project automatically.
    set_target_properties(
      ${LIB_NAME}
      PROPERTIES LINKER_LANGUAGE CXX
                 INSTALL_RPATH "\$ORIGIN"
                 VERSION ${PROJECT_VERSION}
                 CXX_VISIBILITY_PRESET hidden
                 VISIBILITY_INLINES_HIDDEN ON)
  endif()

  if(ENABLE_TEMPLATE_INSTANTIATION AND NOT ARGS_INTERFACE)
    set(PINOCCHIO_CONTEXT_FILE_VALUE "pinocchio/context/${ARGS_SCALAR}.hpp")
    target_compile_definitions(
      ${LIB_NAME}
      PUBLIC PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
      PRIVATE PINOCCHIO_CONTEXT_FILE="${PINOCCHIO_CONTEXT_FILE_VALUE}")
  endif()

  target_include_directories(
    ${LIB_NAME} ${LIBRARY_PUBLIC_SCOPE} $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

  if(BUILD_WITH_COMMIT_VERSION)
    tag_library_version(${LIB_NAME})
  endif()

  install(
    TARGETS ${LIB_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endfunction()

# Define a template instantiation target
function(PINOCCHIO_SPECIFIC_TYPE scalar_name scope)
  set(LIB_NAME "${PROJECT_NAME}_${scalar_name}")
  if(ENABLE_TEMPLATE_INSTANTIATION)
    pinocchio_target(
      ${LIB_NAME}
      SCALAR ${scalar_name}
      SOURCES ${${PROJECT_NAME}_CORE_SOURCES}
      LIBRARY_PUBLIC_SCOPE PUBLIC_SCOPE)
    # By default DEFINE_SYMBOL add -D${LIB_NAME}_EXPORTS. Don't use ${LIB_NAME}_EXPORTS define since
    # pinocchio/config.hpp use pinocchio_EXPORTS. This allow to use the same DLLAPI define for all
    # template instantiation libraries.
    set_target_properties(${LIB_NAME} PROPERTIES DEFINE_SYMBOL "${PROJECT_NAME}_EXPORTS")
    set(${scope}
        ${PUBLIC_SCOPE}
        PARENT_SCOPE)
  else()
    pinocchio_target(
      ${LIB_NAME}
      SCALAR ${scalar_name}
      SOURCES ${${PROJECT_NAME}_CORE_SOURCES}
      LIBRARY_PUBLIC_SCOPE PUBLIC_SCOPE
      INTERFACE)
    set(${scope}
        ${PUBLIC_SCOPE}
        PARENT_SCOPE)
  endif()
endfunction()

function(PINOCCHIO_CONFIG directory lib_name)
  string(TOUPPER ${lib_name} upper_lib_name)
  generate_configuration_header_v2(
    INCLUDE_DIR
    ${PROJECT_BINARY_DIR}/include
    HEADER_DIR
    pinocchio/${directory}
    FILENAME
    config.hpp
    LIBRARY_NAME
    ${upper_lib_name}
    EXPORT_SYMBOL
    ${lib_name}_EXPORTS)
endfunction()

add_source_group(${PROJECT_NAME}_CORE_SOURCES)
add_source_group(${PROJECT_NAME}_PARSERS_SOURCES)
add_source_group(${PROJECT_NAME}_EXTRA_SOURCES)
add_source_group(${PROJECT_NAME}_VISUALIZERS_SOURCES)
add_header_group(${PROJECT_NAME}_CORE_PUBLIC_HEADERS)
add_header_group(${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS)
add_header_group(${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS)
add_header_group(${PROJECT_NAME}_EXTRA_PUBLIC_HEADERS)
add_header_group(${PROJECT_NAME}_CORE_GENERATED_PUBLIC_HEADERS)
add_header_group(${PROJECT_NAME}_VISUALIZERS_PUBLIC_HEADERS)

# Define the default target (double).
#
# This target will also have hpp-fcl and workspace module in it.
pinocchio_specific_type(default DEFAULT_SCOPE)

# Some core library algorithms have different behavior if PINOCCHIO_WITH_HPP_FCL is defined. Since
# some are template instantiated, or some user can link only on pinocchio_default, we muste define
# PINOCCHIO_WITH_HPP_FCL in pinocchio_default.
if(BUILD_WITH_HPP_FCL_SUPPORT)
  set(LINK_TYPE "PUBLIC")
  get_target_property(IS_INTERFACE pinocchio_default TYPE)
  if(IS_INTERFACE STREQUAL "INTERFACE_LIBRARY")
    set(LINK_TYPE "INTERFACE")
  endif()
  target_compile_definitions(pinocchio_default ${LINK_TYPE} PINOCCHIO_WITH_HPP_FCL)
  target_include_directories(
    pinocchio_default ${LINK_TYPE} $<TARGET_PROPERTY:hpp-fcl::hpp-fcl,INTERFACE_INCLUDE_DIRECTORIES>)

endif()

# Define the extra target This target hold extra algorithms.
if(BUILD_WITH_EXTRA_SUPPORT)
  set(EXTRA_LIB_NAME "${PROJECT_NAME}_extra")

  pinocchio_target(
    ${EXTRA_LIB_NAME}
    SCALAR default
    SOURCES ${${PROJECT_NAME}_EXTRA_SOURCES} ${${PROJECT_NAME}_EXTRA_PUBLIC_HEADERS})
  pinocchio_config(extra ${EXTRA_LIB_NAME})

  target_link_libraries(${EXTRA_LIB_NAME} PUBLIC ${PROJECT_NAME}_default Qhull::qhullcpp
                                                 Qhull::qhull_r)

  target_compile_definitions(${EXTRA_LIB_NAME} PUBLIC PINOCCHIO_WITH_EXTRA_SUPPORT)
endif()

# Define the parallel target.
if(BUILD_WITH_OPENMP_SUPPORT)
  set(PARALLEL_LIB_NAME "${PROJECT_NAME}_parallel")

  pinocchio_target(
    ${PARALLEL_LIB_NAME}
    SCALAR default
    SOURCES ${${PROJECT_NAME}_PARALLEL_PUBLIC_HEADERS}
    INTERFACE)

  target_link_libraries(${PARALLEL_LIB_NAME} INTERFACE ${PROJECT_NAME}_default OpenMP::OpenMP_CXX)
endif()

# Define the collision target.
if(BUILD_WITH_HPP_FCL_SUPPORT)
  set(COLLISION_LIB_NAME "${PROJECT_NAME}_collision")

  pinocchio_target(
    ${COLLISION_LIB_NAME}
    SCALAR default
    SOURCES ${${PROJECT_NAME}_COLLISION_SOURCES} ${${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS})
  pinocchio_config(collision ${COLLISION_LIB_NAME})

  target_compile_definitions(${COLLISION_LIB_NAME} PUBLIC PINOCCHIO_WITH_HPP_FCL)
  target_link_libraries(${COLLISION_LIB_NAME} PUBLIC ${PROJECT_NAME}_default hpp-fcl::hpp-fcl)

  # Define the collision parallel target
  if(BUILD_WITH_OPENMP_SUPPORT)
    set(COLLISION_PARALLEL_LIB_NAME "${PROJECT_NAME}_collision_parallel")

    pinocchio_target(
      ${COLLISION_PARALLEL_LIB_NAME}
      SCALAR default
      SOURCES ${${PROJECT_NAME}_COLLISION_PARALLEL_PUBLIC_HEADERS}
      INTERFACE)

    target_link_libraries(${COLLISION_PARALLEL_LIB_NAME} INTERFACE ${COLLISION_LIB_NAME}
                                                                   OpenMP::OpenMP_CXX)
  endif()
endif()

# Define the visualizers target
set(VISUALIZERS_LIB_NAME "${PROJECT_NAME}_visualizers")
pinocchio_target(
  ${VISUALIZERS_LIB_NAME}
  SCALAR default
  SOURCES ${${PROJECT_NAME}_VISUALIZERS_SOURCES} ${${PROJECT_NAME}_VISUALIZERS_PUBLIC_HEADERS})
pinocchio_config(visualizers ${VISUALIZERS_LIB_NAME})
target_link_libraries(${VISUALIZERS_LIB_NAME} PUBLIC ${PROJECT_NAME}_default)
if(BUILD_WITH_HPP_FCL_SUPPORT)
  target_compile_definitions(${VISUALIZERS_LIB_NAME} PUBLIC PINOCCHIO_WITH_HPP_FCL)
  target_link_libraries(${VISUALIZERS_LIB_NAME} PUBLIC hpp-fcl::hpp-fcl)
endif()

# Define the parsers target.
#
# This target will have common tools for parsing/managing files and URDF/SRDF/SDF format support.
if(BUILD_WITH_PARSERS_SUPPORT)
  set(PARSERS_LIB_NAME "${PROJECT_NAME}_parsers")

  pinocchio_target(
    ${PARSERS_LIB_NAME}
    SCALAR default
    SOURCES ${${PROJECT_NAME}_PARSERS_SOURCES} ${${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS})
  pinocchio_config(parsers ${PARSERS_LIB_NAME})

  target_link_libraries(${PARSERS_LIB_NAME} PUBLIC ${PROJECT_NAME}_default)
  if(BUILD_WITH_HPP_FCL_SUPPORT)
    target_link_libraries(${PARSERS_LIB_NAME} PUBLIC ${PROJECT_NAME}_collision)
  endif()

  modernize_target_link_libraries(
    ${PARSERS_LIB_NAME}
    SCOPE PUBLIC
    TARGETS Boost::filesystem
    LIBRARIES ${Boost_FILESYSTEM_LIBRARY}}
    INCLUDE_DIRS ${Boost_INCLUDE_DIRS})

  # Special care of urdfdom version
  if(BUILD_WITH_URDF_SUPPORT)
    target_compile_definitions(${PARSERS_LIB_NAME} PUBLIC PINOCCHIO_WITH_URDFDOM)

    if(${urdfdom_VERSION} VERSION_LESS "0.3.0")
      target_compile_definitions(${PARSERS_LIB_NAME}
                                 PRIVATE PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME)
    endif()
    # defines types from version 0.4.0
    if(NOT ${urdfdom_VERSION} VERSION_LESS "0.4.0")
      target_compile_definitions(${PARSERS_LIB_NAME} PRIVATE PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
    endif()
    # std::shared_ptr appears from version 1.0.0
    if(${urdfdom_VERSION} VERSION_GREATER "0.4.2")
      target_compile_definitions(${PARSERS_LIB_NAME} PRIVATE PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)
    endif()

    modernize_target_link_libraries(
      ${PARSERS_LIB_NAME}
      SCOPE PUBLIC
      TARGETS urdfdom::urdf_parser
      LIBRARIES ${urdfdom_LIBRARIES}
      INCLUDE_DIRS ${urdfdom_INCLUDE_DIRS})
  endif()

  if(BUILD_WITH_SDF_SUPPORT)
    target_compile_definitions(${PARSERS_LIB_NAME} PUBLIC PINOCCHIO_WITH_SDFORMAT)

    target_link_libraries(${PARSERS_LIB_NAME} PUBLIC ${SDFormat_LIBRARIES})
  endif()
endif()

# Define cppad codegen target.
if(BUILD_WITH_CODEGEN_SUPPORT)
  pinocchio_specific_type(cppadcg CPPADCG_SCOPE)
  # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
  # This can happen when Pinocchio is build in Debug mode using another library using cppad.
  target_compile_definitions(${PROJECT_NAME}_cppadcg PUBLIC CPPAD_DEBUG_AND_RELEASE)
  target_include_directories(${PROJECT_NAME}_cppadcg SYSTEM ${CPPADCG_SCOPE}
                             $<BUILD_INTERFACE:${cppadcg_INCLUDE_DIR}>)
  target_link_libraries(${PROJECT_NAME}_cppadcg ${CPPADCG_SCOPE} ${cppadcg_LIBRARY}
                        ${cppad_LIBRARY})
endif()

# Define cppad target.
if(BUILD_WITH_AUTODIFF_SUPPORT)
  pinocchio_specific_type(cppad CPPAD_SCOPE)
  # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
  # This can happen when Pinocchio is build in Debug mode using another library using cppad.
  target_compile_definitions(${PROJECT_NAME}_cppad PUBLIC CPPAD_DEBUG_AND_RELEASE)
  target_include_directories(${PROJECT_NAME}_cppad SYSTEM ${CPPAD_SCOPE}
                             $<BUILD_INTERFACE:${cppad_INCLUDE_DIR}>)
  target_link_libraries(${PROJECT_NAME}_cppad ${CPPAD_SCOPE} ${cppad_LIBRARY})
endif()

# Define casadi target.
if(BUILD_WITH_CASADI_SUPPORT)
  pinocchio_specific_type(casadi CASADI_SCOPE)
  target_link_libraries(${PROJECT_NAME}_casadi ${CASADI_SCOPE} casadi)
endif()

if(BUILD_WITH_PYTHON_PARSER_SUPPORT)
  set(PYTHON_PARSER_LIB_NAME "${PROJECT_NAME}_python_parser")

  pinocchio_target(
    ${PYTHON_PARSER_LIB_NAME}
    SCALAR default
    SOURCES ${${PROJECT_NAME}_PYTHON_PARSER_SOURCES}
            ${${PROJECT_NAME}_PYTHON_PARSER_PUBLIC_HEADERS})
  pinocchio_config(python_parser ${PYTHON_PARSER_LIB_NAME})

  target_link_libraries(${PYTHON_PARSER_LIB_NAME} PUBLIC ${PROJECT_NAME}_default Python3::Python)
  target_link_boost_python(${PYTHON_PARSER_LIB_NAME} PUBLIC)
endif()

# Define main target (default, parsers, extra).
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
if(ENABLE_TEMPLATE_INSTANTIATION)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_default)
endif()
if(BUILD_WITH_PARSERS_SUPPORT)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_parsers)
endif()
if(BUILD_WITH_OPENMP_SUPPORT)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_parallel)
endif()
if(BUILD_WITH_HPP_FCL_SUPPORT)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_collision)
  if(BUILD_WITH_OPENMP_SUPPORT)
    target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_collision_parallel)
  endif()
endif()
if(BUILD_WITH_EXTRA_SUPPORT)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_extra)
endif()
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_visualizers)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
