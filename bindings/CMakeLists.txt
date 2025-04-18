#
# Copyright (c) 2016-2019 CNRS INRIA
# Copyright (c) 2025 INRIA
#

if(BUILD_PYTHON_INTERFACE)
  if(PINOCCHIO_PYTHON_NANOBIND)
    add_subdirectory(python_nb)
    set(PYWRAP_NB # TODO : Necessary ? Because also did in the dedicated CMakeLists.txt
        ${PYWRAP_NB}
        PARENT_SCOPE)
  else()
    add_subdirectory(python)
    set(PYWRAP
        ${PYWRAP}
        PARENT_SCOPE)
  endif()
endif()
