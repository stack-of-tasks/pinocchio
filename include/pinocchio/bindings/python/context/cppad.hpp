//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_context_cppad_hpp__
#define __pinocchio_python_context_cppad_hpp__

#include "pinocchio/autodiff/cppad.hpp"

#define PINOCCHIO_PYTHON_SCALAR_TYPE ::CppAD::AD<double>
#include "pinocchio/bindings/python/context/generic.hpp"
#undef PINOCCHIO_PYTHON_SCALAR_TYPE

#define PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
#define PINOCCHIO_PYTHON_NO_SERIALIZATION
#define PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/user-type.hpp>

namespace pinocchio
{
  namespace python
  {
    inline void exposeSpecificTypeFeatures()
    {
      boost::python::import("pycppad");
    };

    inline boost::python::object getScalarType()
    {
      return eigenpy::getInstanceClass<context::Scalar>();
    }

  } // namespace python
} // namespace pinocchio

#endif // #ifndef __pinocchio_python_context_cppad_hpp__
