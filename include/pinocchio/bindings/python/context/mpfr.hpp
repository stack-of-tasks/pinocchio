//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_python_context_mpfr_hpp__
#define __pinocchio_python_context_mpfr_hpp__

#include "pinocchio/math/multiprecision-mpfr.hpp"

#define PINOCCHIO_PYTHON_SCALAR_TYPE ::boost::multiprecision::number< ::boost::multiprecision::mpfr_float_backend<0>, ::boost::multiprecision::et_off>
#include "pinocchio/bindings/python/context/generic.hpp"

#define PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
#define PINOCCHIO_PYTHON_NO_SERIALIZATION
#define PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE

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

#endif // #ifndef __pinocchio_python_context_mpfr_hpp__
