//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_context_mpfr_hpp__
#define __pinocchio_python_context_mpfr_hpp__

#include "pinocchio/math/multiprecision-mpfr.hpp"

#define PINOCCHIO_PYTHON_SCALAR_TYPE ::boost::multiprecision::number< ::boost::multiprecision::mpfr_float_backend<0>, ::boost::multiprecision::et_off>
#include "pinocchio/bindings/python/context/generic.hpp"


#define PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
#define PINOCCHIO_PYTHON_NO_SERIALIZATION

#include <eigenpy/eigenpy.hpp>
#include "pinocchio/bindings/python/math/multiprecision/boost/number.hpp"

namespace pinocchio
{
  namespace python
  {
    inline void exposeSpecificTypeFeatures()
    {
      BoostNumberPythonVisitor<PINOCCHIO_PYTHON_SCALAR_TYPE>::expose("mpfr");
    };
  
    inline boost::python::object getScalarType()
    {
      namespace bp = boost::python;
      PyTypeObject * pytype = const_cast<PyTypeObject *>(bp::converter::registered_pytype_direct<PINOCCHIO_PYTHON_SCALAR_TYPE>::get_pytype());
      return bp::object(bp::handle<>(bp::borrowed(reinterpret_cast<PyObject *>(pytype))));
    }

  }
}

namespace pinocchio
{
  namespace python
  {
    namespace internal
    {
  
      template<typename T> struct has_operator_equal;
  
      template<>
      struct has_operator_equal< PINOCCHIO_PYTHON_SCALAR_TYPE > : boost::true_type   {};
      
    }
  }
}

#undef PINOCCHIO_PYTHON_SCALAR_TYPE

#endif // #ifndef __pinocchio_python_context_mpfr_hpp__
