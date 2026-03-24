#pragma once

#include "./context.hpp"
#include <nanobind/nanobind.h>

#define PINOCCHIO_PYTHON_NAMESPACE_BEGIN                                                           \
  namespace pinocchio::python_nb                                                                   \
  {
#define PINOCCHIO_PYTHON_NAMESPACE_END } // pinocchio::python_nb

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;
PINOCCHIO_PYTHON_NAMESPACE_END
