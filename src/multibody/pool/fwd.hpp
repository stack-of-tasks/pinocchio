//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_pool_fwd_hpp__
#define __pinocchio_multibody_pool_fwd_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class ModelPoolTpl;
  typedef ModelPoolTpl<double> ModelPool;

#ifdef PINOCCHIO_WITH_HPP_FCL

  template<typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class GeometryPoolTpl;
  typedef GeometryPoolTpl<double> GeometryPool;

  template<typename BroadPhaseManagerDerived, typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class BroadPhaseManagerPoolTpl;
  template<typename BroadPhaseManagerDerived, typename Scalar>
using BroadPhaseManagerPool = BroadPhaseManagerPoolTpl<BroadPhaseManagerDerived,Scalar>;

#endif

}

#endif // ifndef __pinocchio_multibody_pool_fwd_hpp__
