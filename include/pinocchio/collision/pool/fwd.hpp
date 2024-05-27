//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_collision_pool_fwd_hpp__
#define __pinocchio_collision_pool_fwd_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  template<
    typename BroadPhaseManagerDerived,
    typename Scalar,
    int Options = 0,
    template<typename, int> class JointCollectionTpl = JointCollectionDefaultTpl>
  class BroadPhaseManagerPoolBase;

  template<typename Manager>
  struct BroadPhaseManagerTpl; // fwd

  template<
    typename ManagerDerived,
    typename Scalar,
    int Options = 0,
    template<typename, int> class JointCollectionTpl = JointCollectionDefaultTpl>
  using BroadPhaseManagerPoolTpl = BroadPhaseManagerPoolBase<
    BroadPhaseManagerTpl<ManagerDerived>,
    Scalar,
    Options,
    JointCollectionTpl>;

  template<typename ManagerDerived, typename Scalar>
  using BroadPhaseManagerPool = BroadPhaseManagerPoolTpl<ManagerDerived, Scalar>;

  template<typename Manager>
  struct TreeBroadPhaseManagerTpl; // fwd

  template<
    typename ManagerDerived,
    typename Scalar,
    int Options = 0,
    template<typename, int> class JointCollectionTpl = JointCollectionDefaultTpl>
  using TreeBroadPhaseManagerPoolTpl = BroadPhaseManagerPoolBase<
    TreeBroadPhaseManagerTpl<ManagerDerived>,
    Scalar,
    Options,
    JointCollectionTpl>;

  template<typename ManagerDerived, typename Scalar>
  using TreeBroadPhaseManagerPool = TreeBroadPhaseManagerPoolTpl<ManagerDerived, Scalar>;

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_pool_fwd_hpp__
