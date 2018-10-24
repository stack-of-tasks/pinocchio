# 
# Copyright (c) 2015-2018 CNRS
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
# 
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.

# --- MACROS --- #
MACRO(SYMLINK_AND_INSTALL_HEADERS HEADERS SUBPATH)
  FOREACH(header ${HEADERS})
    GET_FILENAME_COMPONENT(headerName ${header} NAME)
    GET_FILENAME_COMPONENT(headerPath ${header} PATH)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${CMAKE_CURRENT_SOURCE_DIR}/${header}
      ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${SUBPATH}/${header})

    INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${header}
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${SUBPATH}/${headerPath}
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
  ENDFOREACH(header)
ENDMACRO(SYMLINK_AND_INSTALL_HEADERS HEADERS SUBPATH)

# --- PYTHON TARGET --- #
ADD_CUSTOM_TARGET(python)
SET_TARGET_PROPERTIES(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

# --- DEPENDENCIES --- #
SET(PKG_CONFIG_PYWRAP_REQUIRES "eigenpy >= 1.4.0")
FOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})
  ADD_COMPILE_DEPENDENCY(${dep})
ENDFOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})

# --- LIBRARY --- #
SET(${PYWRAP}_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/${PROJECT_NAME})

SET(${PROJECT_NAME}_PYTHON_HEADERS
  utils/constant.hpp
  utils/eigen_container.hpp
  utils/handler.hpp
  utils/copyable.hpp
  utils/printable.hpp
  utils/std-vector.hpp
  utils/std-aligned-vector.hpp
  utils/version.hpp
  fwd.hpp
  spatial/se3.hpp
  spatial/force.hpp
  spatial/motion.hpp
  spatial/inertia.hpp
  spatial/explog.hpp
  multibody/frame.hpp
  multibody/model.hpp
  multibody/data.hpp
  multibody/geometry-object.hpp
  multibody/geometry-model.hpp
  multibody/geometry-data.hpp
  multibody/joint/joints-models.hpp
  multibody/joint/joints-variant.hpp
  multibody/joint/joint.hpp
  multibody/joint/joint-derived.hpp
  algorithm/algorithms.hpp 
  parsers/parsers.hpp
  parsers/sample-models.hpp
  )

SET(${PROJECT_NAME}_PYTHON_SOURCES
  module.cpp
  utils/version.cpp
  spatial/expose-SE3.cpp
  spatial/expose-motion.cpp
  spatial/expose-force.cpp
  spatial/expose-inertia.cpp
  spatial/expose-explog.cpp
  multibody/expose-frame.cpp
  multibody/expose-model.cpp
  multibody/expose-data.cpp
  multibody/expose-geometry.cpp
  multibody/joint/expose-joints.cpp
  algorithm/expose-algorithms.cpp
  algorithm/expose-com.cpp
  algorithm/expose-kinematics.cpp
  algorithm/expose-dynamics.cpp
  algorithm/expose-crba.cpp
  algorithm/expose-centroidal.cpp
  algorithm/expose-rnea.cpp
  algorithm/expose-rnea-derivatives.cpp
  algorithm/expose-aba.cpp
  algorithm/expose-aba-derivatives.cpp
  algorithm/expose-jacobian.cpp
  algorithm/expose-kinematics-derivatives.cpp
  algorithm/expose-joints.cpp
  algorithm/expose-energy.cpp
  algorithm/expose-frames.cpp
  algorithm/expose-cat.cpp
  algorithm/expose-geometry.cpp
  algorithm/expose-regressor.cpp
  parsers/expose-parsers.cpp
  )

# Parser for Python model
SET(${PROJECT_NAME}_PARSER_PYTHON_HEADERS
  parsers/python.hpp
  )

SET(${PROJECT_NAME}_PARSER_PYTHON_SOURCES
  parsers/python/model.cpp
  )

# Python exposition of FCL
IF(HPP_FCL_FOUND)
  SET(${PROJECT_NAME}_PYTHON_FCL_HEADERS
    multibody/fcl/contact.hpp
    multibody/fcl/collision-result.hpp
    multibody/fcl/distance-result.hpp
    multibody/fcl/collision-geometry.hpp
    )

  SET(${PROJECT_NAME}_PYTHON_FCL_SOURCES
    multibody/fcl/expose-fcl.cpp
    )

  LIST(APPEND ${PROJECT_NAME}_PYTHON_HEADERS ${${PROJECT_NAME}_PYTHON_FCL_HEADERS}) 
  LIST(APPEND ${PROJECT_NAME}_PYTHON_SOURCES ${${PROJECT_NAME}_PYTHON_FCL_SOURCES}) 
ENDIF(HPP_FCL_FOUND)

LIST(APPEND HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS})
LIST(REMOVE_DUPLICATES HEADERS)

# Headers of the python binding
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/spatial")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/multibody")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/multibody/joint")
IF(HPP_FCL_FOUND)
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/multibody/fcl")
ENDIF(HPP_FCL_FOUND)
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/parsers")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/algorithm")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/bindings/python/utils")
SYMLINK_AND_INSTALL_HEADERS("${${PROJECT_NAME}_PYTHON_HEADERS}" "bindings/python")

# Headers of the python parser
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio/parsers/python")
SYMLINK_AND_INSTALL_HEADERS("${${PROJECT_NAME}_PARSER_PYTHON_HEADERS}" "")

# --- COMPILE WRAPPER
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/bindings/python/${PROJECT_NAME}")
SET(${PYWRAP}_SOURCES ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PARSER_PYTHON_SOURCES})
SET(${PYWRAP}_HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS} ${${PROJECT_NAME}_PARSER_PYTHON_HEADERS})

ADD_LIBRARY(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
ADD_DEPENDENCIES(python ${PYWRAP})

IF(BUILD_WITH_COMMIT_VERSION)
  TAG_LIBRARY_VERSION(${PYWRAP})
ENDIF(BUILD_WITH_COMMIT_VERSION)
ADD_HEADER_GROUP(${PYWRAP}_HEADERS)
ADD_SOURCE_GROUP(${PYWRAP}_SOURCES)
PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)

TARGET_LINK_LIBRARIES(${PYWRAP} ${PROJECT_NAME})
TARGET_LINK_BOOST_PYTHON(${PYWRAP})

IF(HPP_FCL_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} hpp-fcl)
ENDIF(HPP_FCL_FOUND)

IF(APPLE)
  # We need to change the extension for python bindings
  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
ENDIF(APPLE)

SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bindings/python/${PROJECT_NAME}")

INSTALL(TARGETS ${PYWRAP} DESTINATION ${${PYWRAP}_INSTALL_DIR})

# --- INSTALL SCRIPTS 
SET(PYTHON_FILES
  __init__.py
  deprecated.py
  deprecation.py
  utils.py
  robot_wrapper.py
  romeo_wrapper.py
  rpy.py
  explog.py
  )

FOREACH(python ${PYTHON_FILES})
  INSTALL(FILES
    "${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/scripts/${python}"
    DESTINATION ${${PYWRAP}_INSTALL_DIR})
ENDFOREACH(python)

# --- PACKAGING --- #

# Format string
SET(_PKG_CONFIG_PYWRAP_LIBDIR ${${PYWRAP}_INSTALL_DIR})
SET(_PKG_CONFIG_PYWRAP_BINDIR ${${PYWRAP}_INSTALL_DIR})
SET(_PKG_CONFIG_PYWRAP_CONFLICTS)
SET(_PKG_CONFIG_PYWRAP_REQUIRES "${PROJECT_NAME}")
FOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})
  SET(_PKG_CONFIG_PYWRAP_REQUIRES "${_PKG_CONFIG_PYWRAP_REQUIRES}, ${dep}")
ENDFOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})

SET(_PKG_CONFIG_PYWRAP_LIBS "-L\${libdir} -l${PYWRAP}")
IF(APPLE)
  SET(_PKG_CONFIG_PYWRAP_LIBS "${_PKG_CONFIG_PYWRAP_LIBS} -Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY}")
ELSE(APPLE)
  SET(_PKG_CONFIG_PYWRAP_LIBS "${_PKG_CONFIG_PYWRAP_LIBS} ${LIBINCL_KW}boost_python")
ENDIF(APPLE)

SET(_PKG_CONFIG_PYWRAP_CFLAGS "-I\${includedir} -I/usr/local/include")
SET(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} -I${PYTHON_INCLUDE_DIRS}")
FOREACH(cflags ${CFLAGS_DEPENDENCIES})
  SET(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} ${cflags}")
ENDFOREACH(cflags ${CFLAGS_DEPENDENCIES})

CONFIGURE_FILE(
   "${CMAKE_CURRENT_SOURCE_DIR}/pinocchiopy.pc.cmake"
   "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc")

INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/pinocchiopy.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

