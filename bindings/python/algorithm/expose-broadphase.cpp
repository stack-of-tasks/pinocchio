//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/multibody/broadphase-manager.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include "pinocchio/algorithm/broadphase.hpp"

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
    void exposeBroadphaseCallbacks();
  
    template<typename BroadPhaseManager>
    void exposeBroadphaseAlgo()
    {
      
      BroadPhaseManagerPythonVisitor<BroadPhaseManager>::expose();
      typedef BroadPhaseManagerTpl<BroadPhaseManager> Manager;
      
      bp::def("computeCollisions",
              (bool (*)(Manager &, CollisionCallBackBase *))&computeCollisions,
              (bp::arg("manager"),bp::arg("callback")),
              "Determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeCollisions",
              (bool (*)(Manager &, const bool))&computeCollisions,
              (bp::arg("manager"),bp::arg("stop_at_first_collision") = false),
              "Determine if all collision pairs are effectively in collision or not."
              );
      
      bp::def("computeCollisions",
              (bool (*)(const Model &, Data &, Manager &, const Eigen::MatrixBase<Eigen::VectorXd> &, const bool))&computeCollisions<double,0,JointCollectionDefaultTpl,BroadPhaseManager,Eigen::VectorXd>,
              (bp::arg("model"),bp::arg("data"),bp::arg("broadphase_manager"),bp::arg("q"),bp::arg("stop_at_first_collision") = false),
              "Compute the forward kinematics, update the geometry placements and run the collision detection using the broadphase manager."
              );
      
      bp::def("computeCollisions",
              (bool (*)(const Model &, Data &, Manager &,
                        CollisionCallBackBase *, const Eigen::MatrixBase<Eigen::VectorXd> &))&computeCollisions<double,0,JointCollectionDefaultTpl,BroadPhaseManager,Eigen::VectorXd>,
              (bp::arg("model"),bp::arg("data"),bp::arg("broadphase_manager"),bp::arg("callback"),bp::arg("q")),
              "Compute the forward kinematics, update the geometry placements and run the collision detection using the broadphase manager."
              );
    }
  
    void exposeBroadphase()
    {
      using namespace Eigen;
      exposeBroadphaseCallbacks();
      
      typedef ::hpp::fcl::CollisionObject* CollisionObjectPointer;
      StdVectorPythonVisitor< std::vector<CollisionObjectPointer> >::expose("StdVec_CollisionObjectPointer");
      StdVectorPythonVisitor< std::vector<::hpp::fcl::CollisionObject> >::expose("StdVec_CollisionObject");
      
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

