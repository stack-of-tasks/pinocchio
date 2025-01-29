//
// Copyright (c) 2022 INRIA
//

#include <iostream>

#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"
#include "pinocchio/collision/broadphase-manager.hpp"
#include "pinocchio/collision/broadphase.hpp"

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_broadphase_with_empty_models)
{
  Model model;
  GeometryModel geom_model;
  GeometryData geom_data(geom_model);

  BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager> broadphase_manager(
    &model, &geom_model, &geom_data);

  BOOST_CHECK(broadphase_manager.check());
}

BOOST_AUTO_TEST_CASE(test_broadphase)
{
  Model model;
  Data data(model);
  GeometryModel geom_model;

  coal::CollisionGeometryPtr_t sphere_ptr(new coal::Sphere(0.5));
  sphere_ptr->computeLocalAABB();
  coal::CollisionGeometryPtr_t box_ptr(new coal::Box(0.5, 0.5, 0.5));
  box_ptr->computeLocalAABB();

  GeometryObject obj1("obj1", 0, SE3::Identity(), sphere_ptr);
  const GeomIndex obj1_index = geom_model.addGeometryObject(obj1);

  GeometryObject obj2("obj2", 0, SE3::Identity(), box_ptr);
  const GeomIndex obj2_index = geom_model.addGeometryObject(obj2);

  GeometryObject & go = geom_model.geometryObjects[obj1_index];

  GeometryData geom_data(geom_model);
  updateGeometryPlacements(model, data, geom_model, geom_data);

  BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager> broadphase_manager(
    &model, &geom_model, &geom_data);
  BOOST_CHECK(broadphase_manager.check());
  BOOST_CHECK(sphere_ptr.get() == go.geometry.get());

  coal::CollisionGeometryPtr_t sphere_new_ptr(new coal::Sphere(5.));
  sphere_new_ptr->computeLocalAABB();
  go.geometry = sphere_new_ptr;
  BOOST_CHECK(!broadphase_manager.check());
  BOOST_CHECK(sphere_ptr.get() != go.geometry.get());
  BOOST_CHECK(
    broadphase_manager.getCollisionObjects()[obj1_index].collisionGeometry().get()
    == sphere_ptr.get());
  BOOST_CHECK(
    broadphase_manager.getCollisionObjects()[obj2_index].collisionGeometry().get()
    == box_ptr.get());
  //  BOOST_CHECK(broadphase_manager.getObjects()[obj1_index]->collisionGeometry().get() ==
  //  sphere_ptr.get());
  BOOST_CHECK(
    broadphase_manager.getCollisionObjects()[obj1_index].collisionGeometry().get()
    != go.geometry.get());
  BOOST_CHECK(sphere_new_ptr.get() == go.geometry.get());

  broadphase_manager.update(false);
  BOOST_CHECK(
    broadphase_manager.getCollisionObjects()[obj1_index].collisionGeometry().get()
    != sphere_ptr.get());
  BOOST_CHECK(
    broadphase_manager.getCollisionObjects()[obj1_index].collisionGeometry().get()
    == go.geometry.get());
  //  BOOST_CHECK(broadphase_manager.getObjects()[obj_index]->collisionGeometry().get() ==
  //  go.geometry.get());

  BOOST_CHECK(broadphase_manager.check());
}

BOOST_AUTO_TEST_CASE(test_advanced_filters)
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

  GeometryData geom_data(geom_model);

  typedef BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager> BroadPhaseManager;
  for (size_t joint_id = 0; joint_id < (size_t)model.njoints; ++joint_id)
  {
    const GeometryObjectFilterSelectByJoint filter(joint_id);
    BroadPhaseManager manager(&model, &geom_model, &geom_data, filter);
    BOOST_CHECK(manager.check());
  }
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

  Data data(model);
  GeometryData geom_data(geom_model), geom_data_broadphase(geom_model);

  pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename, false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data_broadphase, q);

  BOOST_CHECK(computeCollisions(geom_model, geom_data) == false);
  BOOST_CHECK(computeCollisions(geom_model, geom_data, false) == false);

  BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager> broadphase_manager(
    &model, &geom_model, &geom_data_broadphase);
  std::cout << "map:\n" << geom_model.collisionPairMapping << std::endl;
  BOOST_CHECK(computeCollisions(broadphase_manager) == false);
  BOOST_CHECK(computeCollisions(broadphase_manager, false) == false);
  BOOST_CHECK(computeCollisions(model, data, broadphase_manager, q) == false);
  BOOST_CHECK(computeCollisions(model, data, broadphase_manager, q, false) == false);

  const int num_configs = 1000;
  for (int i = 0; i < num_configs; ++i)
  {
    Eigen::VectorXd q_rand = randomConfiguration(model);
    q_rand.head<7>() = q.head<7>();

    BOOST_CHECK(
      computeCollisions(model, data, broadphase_manager, q_rand)
      == computeCollisions(model, data, geom_model, geom_data, q_rand));
  }
}

BOOST_AUTO_TEST_SUITE_END()
