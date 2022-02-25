//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_pool_fwd_hpp__
#define __pinocchio_multibody_pool_fwd_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class ModelPoolTpl;
  typedef ModelPoolTpl<context::Scalar> ModelPool;

#ifdef PINOCCHIO_WITH_HPP_FCL

  template<typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class GeometryPoolTpl;
  typedef GeometryPoolTpl<context::Scalar> GeometryPool;

  template<typename BroadPhaseManagerDerived, typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl> class BroadPhaseManagerPoolBaseTpl;

  template<typename Manager> struct BroadPhaseManagerTpl; // fwd

  template<typename ManagerDerived, typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl>
  using BroadPhaseManagerPoolTpl = BroadPhaseManagerPoolBaseTpl<BroadPhaseManagerTpl<ManagerDerived>,Scalar,Options,JointCollectionTpl>;

  template<typename ManagerDerived, typename Scalar>
  using BroadPhaseManagerPool = BroadPhaseManagerPoolTpl<ManagerDerived,Scalar>;

  template<typename Manager> struct TreeBroadPhaseManagerTpl; // fwd

  template<typename ManagerDerived, typename Scalar, int Options = 0, template<typename,int> class JointCollectionTpl = JointCollectionDefaultTpl>
  using TreeBroadPhaseManagerPoolTpl = BroadPhaseManagerPoolBaseTpl<TreeBroadPhaseManagerTpl<ManagerDerived>,Scalar,Options,JointCollectionTpl>;

  template<typename ManagerDerived, typename Scalar>
  using TreeBroadPhaseManagerPool = TreeBroadPhaseManagerPoolTpl<ManagerDerived,Scalar>;

#endif

}

#endif // ifndef __pinocchio_multibody_pool_fwd_hpp__
