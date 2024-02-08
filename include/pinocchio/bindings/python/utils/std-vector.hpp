//
// Copyright (c) 2016-2024 CNRS INRIA
//

#ifndef __pinocchio_python_utils_std_vector_hpp__
#define __pinocchio_python_utils_std_vector_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include <eigenpy/std-vector.hpp>

namespace eigenpy {

template<typename Derived>
struct has_operator_equal< ::pinocchio::NumericalBase<Derived> > : has_operator_equal<typename ::pinocchio::NumericalBase<Derived>::Scalar>
{};

} // namespace eigenpy

namespace pinocchio
{
  namespace python
  {
  template <class vector_type, bool NoProxy = false,
  bool EnableFromPythonListConverter = true, 
#ifdef PINOCCHIO_PYTHON_NO_SERIALIZATION
  bool pickable = false>
#else
  bool pickable = true>
#endif
  using StdVectorPythonVisitor = eigenpy::StdVectorPythonVisitor<vector_type, NoProxy, EnableFromPythonListConverter, pickable>;
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_vector_hpp__
