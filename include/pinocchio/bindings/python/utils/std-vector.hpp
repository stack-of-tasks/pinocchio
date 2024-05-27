//
// Copyright (c) 2016-2024 CNRS INRIA
//

#ifndef __pinocchio_python_utils_std_vector_hpp__
#define __pinocchio_python_utils_std_vector_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include <eigenpy/std-vector.hpp>

#include <type_traits>

namespace eigenpy
{

  template<typename Derived>
  struct has_operator_equal<
    Derived,
    typename std::enable_if<
      std::is_base_of<::pinocchio::NumericalBase<Derived>, Derived>::value,
      Derived>::type> : has_operator_equal<typename ::pinocchio::NumericalBase<Derived>::Scalar>
  {
  };

  template<typename _Scalar, int _Rows, int _Cols, int _Options>
  struct has_operator_equal<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options>>
  : has_operator_equal<typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options>::Scalar>
  {
  };

} // namespace eigenpy

namespace pinocchio
{
  namespace python
  {
    template<
      class vector_type,
      bool NoProxy = false,
      bool EnableFromPythonListConverter = true,
#ifdef PINOCCHIO_PYTHON_NO_SERIALIZATION
      bool pickable = false>
#else
      bool pickable = true>
#endif
    using StdVectorPythonVisitor = eigenpy::
      StdVectorPythonVisitor<vector_type, NoProxy, EnableFromPythonListConverter, pickable>;
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_vector_hpp__
