//
// Copyright (c) 2020-2023 INRIA
//

#ifndef __pinocchio_python_context_default_hpp__
#define __pinocchio_python_context_default_hpp__

#define PINOCCHIO_PYTHON_SCALAR_TYPE PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT
#define PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE
#define PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE

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
      return bp::object(bp::handle<>(bp::borrowed(reinterpret_cast<PyObject *>(&PyFloat_Type))));
    }
  } // namespace python
} // namespace pinocchio

#undef PINOCCHIO_PYTHON_SCALAR_TYPE
#endif // #ifndef __pinocchio_python_context_default_hpp__
