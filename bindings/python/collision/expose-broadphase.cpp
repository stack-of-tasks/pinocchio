//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/bindings/python/collision/broadphase-manager.hpp"
#include "pinocchio/bindings/python/collision/tree-broadphase-manager.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include "pinocchio/collision/broadphase.hpp"

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
    void exposeBroadphaseCallbacks(); // fwd

    template<typename BroadPhaseManager>
    void _exposeBroadphaseAlgo()
    {

      typedef BroadPhaseManager Manager;
      typedef BroadPhaseManagerBase<BroadPhaseManager> BaseManager;

      bp::def(
        "computeCollisions", (bool (*)(BaseManager &, CollisionCallBackBase *))&computeCollisions,
        (bp::arg("manager"), bp::arg("callback")),
        "Determine if all collision pairs are effectively in collision or not.\n"
        "This function assumes that updateGeometryPlacements and broadphase_manager.update() "
        "have been called first.");

      bp::def(
        "computeCollisions", (bool (*)(BaseManager &, const bool))&computeCollisions,
        (bp::arg("manager"), bp::arg("stop_at_first_collision") = false),
        "Determine if all collision pairs are effectively in collision or not.\n"
        "This function assumes that updateGeometryPlacements and broadphase_manager.update() "
        "have been called first.");

      bp::def(
        "computeCollisions",
        (bool (*)(
          const Model &, Data &, BaseManager &, const Eigen::MatrixBase<Eigen::VectorXd> &,
          const bool))&computeCollisions<double, 0, JointCollectionDefaultTpl, Manager, Eigen::VectorXd>,
        (bp::arg("model"), bp::arg("data"), bp::arg("broadphase_manager"), bp::arg("q"),
         bp::arg("stop_at_first_collision") = false),
        "Compute the forward kinematics, update the geometry placements and run the "
        "collision detection using the broadphase manager.");

      bp::def(
        "computeCollisions",
        (bool (*)(
          const Model &, Data &, BaseManager &, CollisionCallBackBase *,
          const Eigen::MatrixBase<
            Eigen::
              VectorXd> &))&computeCollisions<double, 0, JointCollectionDefaultTpl, Manager, Eigen::VectorXd>,
        (bp::arg("model"), bp::arg("data"), bp::arg("broadphase_manager"), bp::arg("callback"),
         bp::arg("q")),
        "Compute the forward kinematics, update the geometry placements and run the "
        "collision detection using the broadphase manager.");
    }

    template<typename BroadPhaseManager>
    void exposeBroadphaseAlgo()
    {
      BroadPhaseManagerPythonVisitor<BroadPhaseManager>::expose();
      _exposeBroadphaseAlgo<BroadPhaseManagerTpl<BroadPhaseManager>>();

      TreeBroadPhaseManagerPythonVisitor<BroadPhaseManager>::expose();
      _exposeBroadphaseAlgo<TreeBroadPhaseManagerTpl<BroadPhaseManager>>();
    }

    void exposeBroadphase()
    {
      using namespace Eigen;
      exposeBroadphaseCallbacks();

      typedef ::hpp::fcl::CollisionObject * CollisionObjectPointer;
      StdVectorPythonVisitor<std::vector<CollisionObjectPointer>>::expose(
        "StdVec_FCL_CollisionObjectPointer");
      StdVectorPythonVisitor<std::vector<CollisionObject>>::expose("StdVec_CollisionObject");

      exposeBroadphaseAlgo<hpp::fcl::DynamicAABBTreeCollisionManager>();
      exposeBroadphaseAlgo<hpp::fcl::DynamicAABBTreeArrayCollisionManager>();
      exposeBroadphaseAlgo<hpp::fcl::SSaPCollisionManager>();
      exposeBroadphaseAlgo<hpp::fcl::SaPCollisionManager>();
      exposeBroadphaseAlgo<hpp::fcl::NaiveCollisionManager>();
      exposeBroadphaseAlgo<hpp::fcl::IntervalTreeCollisionManager>();
      //      exposeBroadphaseAlgo<hpp::fcl::SpatialHashingCollisionManager<> >();
    }
  } // namespace python
} // namespace pinocchio
