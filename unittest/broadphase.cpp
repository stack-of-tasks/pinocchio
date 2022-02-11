//
// Copyright (c) 2022 INRIA
//

#include <iostream>

#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/multibody/broadphase-manager.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;


BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)
  
BOOST_AUTO_TEST_CASE (test_collisions)
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;
  typedef pinocchio::Data Data;
  typedef pinocchio::GeometryData GeometryData;
  
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > packageDirs;
  const std::string meshDir  = PINOCCHIO_MODEL_DIR;
  packageDirs.push_back(meshDir);
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/romeo_description/srdf/romeo.srdf");
  
  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, packageDirs);
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geom_model,srdf_filename,false);
  
  Data data(model);
  GeometryData geom_data(geom_model), geom_data_broadphase(geom_model);

  pinocchio::srdf::loadReferenceConfigurations(model,srdf_filename,false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data_broadphase, q);

  BOOST_CHECK(computeCollisions(geom_model,geom_data) == false);
  BOOST_CHECK(computeCollisions(geom_model,geom_data,false) == false);
  
  BroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> broadphase_manager(&geom_model, &geom_data_broadphase);
  std::cout << "map:\n" << geom_model.collisionPairMapping << std::endl;
  BOOST_CHECK(computeCollisions(broadphase_manager) == false);
  BOOST_CHECK(computeCollisions(broadphase_manager,false) == false);
  
  for(size_t cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
  {
    const CollisionPair & cp = geom_model.collisionPairs[cp_index];
    const GeometryObject & obj1 = geom_model.geometryObjects[cp.first];
    const GeometryObject & obj2 = geom_model.geometryObjects[cp.second];
     
    hpp::fcl::CollisionResult other_res;
    computeCollision(geom_model,geom_data,cp_index);
    
    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[cp.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[cp.second]));
    
    fcl::collide(obj1.geometry.get(), oM1,
                 obj2.geometry.get(), oM2,
                 geom_data.collisionRequests[cp_index],
                 other_res);
    
    {
      const hpp::fcl::CollisionResult & res = geom_data.collisionResults[cp_index];
      
      BOOST_CHECK(res.isCollision() == other_res.isCollision());
      BOOST_CHECK(!res.isCollision());
    }
    
    {
      const hpp::fcl::CollisionResult & res = geom_data_broadphase.collisionResults[cp_index];
      
      BOOST_CHECK(res.isCollision() == other_res.isCollision());
      BOOST_CHECK(!res.isCollision());
    }
  }
    
  // test other signatures
  {
    Data data(model);
    GeometryData geom_data(geom_model);
    BOOST_CHECK(computeCollisions(model,data,geom_model,geom_data,q) == false);
  }
}

BOOST_AUTO_TEST_SUITE_END()
