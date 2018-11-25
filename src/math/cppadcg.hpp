//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_math_ccpadcg_hpp__
#define __pinocchio_math_ccpadcg_hpp__

#include <cmath>
#include <cppad/cg/support/cppadcg_eigen.hpp>

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for CppAD input types
    template<typename Scalar>
    struct cast_impl<CppAD::cg::CG<Scalar>,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const CppAD::cg::CG<Scalar> & x)
      {
        return x.getValue();
      }
    };
  }
}

#endif // #ifndef __pinocchio_math_ccpadcg_hpp__
