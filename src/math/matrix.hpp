//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_math_matrix_hpp__
#define __pinocchio_math_matrix_hpp__

#include <Eigen/Core>

namespace pinocchio
{

  template<typename Derived>
  inline bool hasNaN(const Eigen::DenseBase<Derived> & m) 
  {
    return !((m.derived().array()==m.derived().array()).all());
  }

  template<typename M1, typename M2>
  struct MatrixProduct
  {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
    typedef typename Eigen::Product<M1,M2> type;
#else
    typedef typename Eigen::ProductReturnType<M1,M2>::Type type;
#endif
  };

}
#endif //#ifndef __pinocchio_math_matrix_hpp__
