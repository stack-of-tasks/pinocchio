//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"

#include "pinocchio/bindings/python/collision/pool/geometry.hpp"
#include "pinocchio/bindings/python/collision/pool/broadphase-manager.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/broadphase_SSaP.h>
#include <hpp/fcl/broadphase/broadphase_SaP.h>
#include <hpp/fcl/broadphase/broadphase_bruteforce.h>
#include <hpp/fcl/broadphase/broadphase_interval_tree.h>
#include <hpp/fcl/broadphase/broadphase_spatialhash.h>

namespace pinocchio
{
  namespace python
  {

    void exposePoolCollision()
    {
      GeometryPoolPythonVisitor<GeometryPool>::expose();
      BroadPhaseManagerPoolPythonVisitor<
        BroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double>>::expose();
      BroadPhaseManagerPoolPythonVisitor<
        TreeBroadPhaseManagerPool<hpp::fcl::DynamicAABBTreeCollisionManager, double>>::expose();
    }

  } // namespace python
} // namespace pinocchio
