//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_pool_fwd_hpp__
#define __pinocchio_multibody_pool_fwd_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options = 0,
    template<typename, int> class JointCollectionTpl = JointCollectionDefaultTpl>
  class ModelPoolTpl;
  typedef ModelPoolTpl<context::Scalar> ModelPool;

  template<
    typename Scalar,
    int Options = 0,
    template<typename, int> class JointCollectionTpl = JointCollectionDefaultTpl>
  class GeometryPoolTpl;
  typedef GeometryPoolTpl<context::Scalar> GeometryPool;

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_pool_fwd_hpp__
