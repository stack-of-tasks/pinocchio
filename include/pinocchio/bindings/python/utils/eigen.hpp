//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_eigen_hpp__
#define __pinocchio_python_utils_eigen_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
  
    template<typename Matrix>
    Eigen::Ref<Matrix> make_ref(Eigen::PlainObjectBase<Matrix> & mat)
    {
      typedef Eigen::Ref<Matrix> ReturnType;
      return ReturnType(mat);
    }
  
    template<typename Matrix>
    void make_symmetric(Eigen::MatrixBase<Matrix> & mat)
    {
      mat.template triangularView<Eigen::StrictlyLower>() =
      mat.transpose().template triangularView<Eigen::StrictlyLower>();
    }
  
    template<typename Matrix>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix) make_copy(const Eigen::MatrixBase<Matrix> & mat)
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix) ReturnType;
      return ReturnType(mat);
    }
  
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_eigen_hpp__
