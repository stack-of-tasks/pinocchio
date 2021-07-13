//
// Copyright (c) 2020-2021 INRIA
//

#ifndef __pinocchio_python_context_default_hpp__
#define __pinocchio_python_context_default_hpp__

#define PINOCCHIO_PYTHON_SCALAR_TYPE PINOCCHIO_SCALAR_TYPE
#include "pinocchio/bindings/python/context/generic.hpp"
#include <boost/python.hpp>

namespace pinocchio { namespace python {

  inline void exposeSpecificTypeFeatures() {};

  inline boost::python::object getScalarType()
  {
    namespace bp = boost::python;
    return bp::object(bp::handle<>(bp::borrowed(reinterpret_cast<PyObject *>(&PyFloat_Type))));
  }
}}

#undef PINOCCHIO_PYTHON_SCALAR_TYPE
#endif // #ifndef __pinocchio_python_context_default_hpp__
