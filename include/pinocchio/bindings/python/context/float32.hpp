//
// Copyright (c) 2026 Heriot-Watt University
//

#pragma once

#define PINOCCHIO_PYTHON_SCALAR_TYPE float
#define PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
#define PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE

#include "pinocchio/bindings/python/context/generic.hpp"
#include <eigenpy/eigenpy.hpp>

namespace pinocchio
{
  namespace python
  {

    inline void exposeSpecificTypeFeatures() {};

    inline boost::python::object getScalarType()
    {
      namespace bp = boost::python;
      return bp::import("numpy").attr("float32");
    }
  } // namespace python
} // namespace pinocchio

#undef PINOCCHIO_PYTHON_SCALAR_TYPE
