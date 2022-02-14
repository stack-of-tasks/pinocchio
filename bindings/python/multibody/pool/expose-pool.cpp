//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
#include "pinocchio/bindings/python/multibody/pool/geometry.hpp"
#include "pinocchio/bindings/python/multibody/pool/broadphase-manager.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/broadphase_SSaP.h>
#include <hpp/fcl/broadphase/broadphase_SaP.h>
#include <hpp/fcl/broadphase/broadphase_bruteforce.h>
#include <hpp/fcl/broadphase/broadphase_interval_tree.h>
#include <hpp/fcl/broadphase/broadphase_spatialhash.h>
#endif

namespace pinocchio
{
  namespace python
  {
    
    void exposePool()
    {
      ModelPoolPythonVisitor<context::ModelPool>::expose();
      
#ifdef PINOCCHIO_WITH_HPP_FCL
      BroadPhaseManagerPoolPythonVisitor< BroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager,context::Scalar> >::expose();
      GeometryPoolPythonVisitor<GeometryPool>::expose();
#endif
    }
    
  } // namespace python
} // namespace pinocchio

