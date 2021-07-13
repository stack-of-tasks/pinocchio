//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_algorithm_fwd_hpp__
#define __pinocchio_algorithm_fwd_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{
  template<typename Scalar> struct ProximalSettingsTpl;
  typedef ProximalSettingsTpl<double> ProximalSettings;

  namespace cholesky
  {
    template<typename Scalar, int Options> struct ContactCholeskyDecompositionTpl;
    typedef ContactCholeskyDecompositionTpl<double,0> ContactCholeskyDecomposition;
  }

  template<typename Scalar, int Options> struct RigidConstraintModelTpl;
  template<typename Scalar, int Options> struct RigidConstraintDataTpl;
  
  typedef RigidConstraintModelTpl<double,0> RigidConstraintModel;
  typedef RigidConstraintDataTpl<double,0> RigidConstraintData;
}


#endif // ifndef __pinocchio_algorithm_fwd_hpp__
