//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/pool/model.hpp"

#include "pinocchio/bindings/python/collision/pool/geometry.hpp"
#include "pinocchio/bindings/python/collision/pool/broadphase-manager.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <coal/broadphase/broadphase_SSaP.h>
#include <coal/broadphase/broadphase_SaP.h>
#include <coal/broadphase/broadphase_bruteforce.h>
#include <coal/broadphase/broadphase_interval_tree.h>
#include <coal/broadphase/broadphase_spatialhash.h>

namespace pinocchio
{
  namespace python
  {

    void exposePoolCollision()
    {
      GeometryPoolPythonVisitor<GeometryPool>::expose();
      BroadPhaseManagerPoolPythonVisitor<
        BroadPhaseManagerPool<coal::DynamicAABBTreeCollisionManager, double>>::expose();
      BroadPhaseManagerPoolPythonVisitor<
        TreeBroadPhaseManagerPool<coal::DynamicAABBTreeCollisionManager, double>>::expose();
    }

  } // namespace python
} // namespace pinocchio
