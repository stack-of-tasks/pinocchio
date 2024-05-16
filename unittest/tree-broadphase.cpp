//
// Copyright (c) 2022 INRIA
//

#include <iostream>

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_tree_broadphase_with_empty_models)
{
  Model model;
  GeometryModel geom_model;
  GeometryData geom_data(geom_model);

  TreeBroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> broadphase_manager(
    &model, &geom_model, &geom_data);

  BOOST_CHECK(broadphase_manager.check());
}

BOOST_AUTO_TEST_CASE(test_collisions)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector<std::string> packageDirs;
  const std::string meshDir = PINOCCHIO_MODEL_DIR;
  packageDirs.push_back(meshDir);
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/srdf/romeo.srdf");

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, packageDirs);
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename, false);

  Data data(model), data_broadphase(model);
  GeometryData geom_data(geom_model), geom_data_broadphase(geom_model);

  pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename, false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);
  pinocchio::updateGeometryPlacements(model, data_broadphase, geom_model, geom_data_broadphase, q);

  BOOST_CHECK(computeCollisions(geom_model, geom_data) == false);
  BOOST_CHECK(computeCollisions(geom_model, geom_data, false) == false);

  TreeBroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> broadphase_manager(
    &model, &geom_model, &geom_data_broadphase);
  BOOST_CHECK(computeCollisions(broadphase_manager) == false);
  BOOST_CHECK(computeCollisions(broadphase_manager, false) == false);
  BOOST_CHECK(computeCollisions(model, data_broadphase, broadphase_manager, q) == false);
  BOOST_CHECK(computeCollisions(model, data_broadphase, broadphase_manager, q, false) == false);

  const int num_configs = 1000;
  for (int i = 0; i < num_configs; ++i)
  {
    Eigen::VectorXd q_rand = randomConfiguration(model);
    q_rand.head<7>() = q.head<7>();

    BOOST_CHECK(
      computeCollisions(model, data_broadphase, broadphase_manager, q_rand)
      == computeCollisions(model, data, geom_model, geom_data, q_rand));
  }
}

BOOST_AUTO_TEST_SUITE_END()
