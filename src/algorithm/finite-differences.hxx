//
// Copyright (c) 2016,2018 CNRS
//

#ifndef __pinocchio_finite_differences_hxx__
#define __pinocchio_finite_differences_hxx__

#include <limits>

/// @cond DEV

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType
  finiteDifferenceIncrement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::TangentVectorType ReturnType;
    
    static Scalar sqrt_eps = math::sqrt(std::numeric_limits<Scalar>::epsilon());
    ReturnType fd_increment(ReturnType::Constant(model.nv,sqrt_eps));

    return fd_increment;
  }
  
} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_finite_differences_hxx__
