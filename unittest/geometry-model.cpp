//
// Copyright (c) 2015-2022 CNRS INRIA
//

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(manage_collision_pairs)
{
  std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector<std::string> package_dirs;
  std::string mesh_dir = PINOCCHIO_MODEL_DIR;
  package_dirs.push_back(mesh_dir);

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, package_dirs);
  geom_model.addAllCollisionPairs();

  for (Eigen::DenseIndex i = 0; i < (Eigen::DenseIndex)geom_model.ngeoms; ++i)
  {
    for (Eigen::DenseIndex j = i + 1; j < (Eigen::DenseIndex)geom_model.ngeoms; ++j)
    {
      BOOST_CHECK(geom_model.collisionPairMapping(i, j) < (int)geom_model.collisionPairs.size());
      BOOST_CHECK(geom_model.collisionPairMapping(j, i) < (int)geom_model.collisionPairs.size());
      BOOST_CHECK(geom_model.collisionPairMapping(j, i) == geom_model.collisionPairMapping(i, j));

      if (geom_model.collisionPairMapping(i, j) != -1)
      {
        const PairIndex pair_index = (PairIndex)geom_model.collisionPairMapping(i, j);
        const CollisionPair & cp_ref = geom_model.collisionPairs[pair_index];
        const CollisionPair cp((size_t)i, (size_t)j);
        BOOST_CHECK(cp == cp_ref);
      }
    }
  }

  GeometryModel::MatrixXb collision_map(GeometryModel::MatrixXb::Zero(
    (Eigen::DenseIndex)geom_model.ngeoms, (Eigen::DenseIndex)geom_model.ngeoms));

  for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = geom_model.collisionPairs[k];
    collision_map((Eigen::DenseIndex)cp.first, (Eigen::DenseIndex)cp.second) = true;
  }
  GeometryModel::MatrixXb collision_map_lower = collision_map.transpose();

  GeometryModel geom_model_copy, geom_model_copy_lower;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model_copy, package_dirs);
  pinocchio::urdf::buildGeom(
    model, filename, pinocchio::COLLISION, geom_model_copy_lower, package_dirs);
  geom_model_copy.setCollisionPairs(collision_map);
  geom_model_copy_lower.setCollisionPairs(collision_map_lower, false);

  BOOST_CHECK(geom_model_copy.collisionPairs.size() == geom_model.collisionPairs.size());
  BOOST_CHECK(geom_model_copy_lower.collisionPairs.size() == geom_model.collisionPairs.size());
  for (size_t k = 0; k < geom_model_copy.collisionPairs.size(); ++k)
  {
    BOOST_CHECK(geom_model.existCollisionPair(geom_model_copy.collisionPairs[k]));
    BOOST_CHECK(geom_model.existCollisionPair(geom_model_copy_lower.collisionPairs[k]));
  }
  for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
  {
    BOOST_CHECK(geom_model_copy.existCollisionPair(geom_model.collisionPairs[k]));
    BOOST_CHECK(geom_model_copy_lower.existCollisionPair(geom_model.collisionPairs[k]));
  }

  {
    GeometryData geom_data(geom_model);
    geom_data.activateAllCollisionPairs();

    for (size_t k = 0; k < geom_data.activeCollisionPairs.size(); ++k)
      BOOST_CHECK(geom_data.activeCollisionPairs[k]);
  }

  {
    GeometryData geom_data(geom_model);
    geom_data.deactivateAllCollisionPairs();

    for (size_t k = 0; k < geom_data.activeCollisionPairs.size(); ++k)
      BOOST_CHECK(!geom_data.activeCollisionPairs[k]);
  }

  {
    GeometryData geom_data(geom_model), geom_data_copy(geom_model),
      geom_data_copy_lower(geom_model);
    geom_data_copy.deactivateAllCollisionPairs();
    geom_data_copy_lower.deactivateAllCollisionPairs();

    GeometryData::MatrixXb collision_map(GeometryModel::MatrixXb::Zero(
      (Eigen::DenseIndex)geom_model.ngeoms, (Eigen::DenseIndex)geom_model.ngeoms));
    for (size_t k = 0; k < geom_data.activeCollisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];
      collision_map((Eigen::DenseIndex)cp.first, (Eigen::DenseIndex)cp.second) =
        geom_data.activeCollisionPairs[k];
    }
    GeometryData::MatrixXb collision_map_lower = collision_map.transpose();

    geom_data_copy.setActiveCollisionPairs(geom_model, collision_map);
    BOOST_CHECK(geom_data_copy.activeCollisionPairs == geom_data.activeCollisionPairs);

    geom_data_copy_lower.setActiveCollisionPairs(geom_model, collision_map_lower, false);
    BOOST_CHECK(geom_data_copy_lower.activeCollisionPairs == geom_data.activeCollisionPairs);
  }

  // Test security margins
  {
    GeometryData geom_data_upper(geom_model), geom_data_lower(geom_model);

    const GeometryData::MatrixXs security_margin_map(GeometryData::MatrixXs::Ones(
      (Eigen::DenseIndex)geom_model.ngeoms, (Eigen::DenseIndex)geom_model.ngeoms));
    GeometryData::MatrixXs security_margin_map_upper(security_margin_map);
    security_margin_map_upper.triangularView<Eigen::Lower>().fill(0.);

    geom_data_upper.setSecurityMargins(geom_model, security_margin_map, true, true);
    for (size_t k = 0; k < geom_data_upper.collisionRequests.size(); ++k)
    {
      BOOST_CHECK(geom_data_upper.collisionRequests[k].security_margin == 1.);
      BOOST_CHECK(
        geom_data_upper.collisionRequests[k].security_margin
        == geom_data_upper.collisionRequests[k].distance_upper_bound);
    }

    geom_data_lower.setSecurityMargins(geom_model, security_margin_map, false);
    for (size_t k = 0; k < geom_data_lower.collisionRequests.size(); ++k)
    {
      BOOST_CHECK(geom_data_lower.collisionRequests[k].security_margin == 1.);
    }
  }

  // Test enableGeometryCollision
  {
    GeometryData geom_data(geom_model);
    geom_data.deactivateAllCollisionPairs();
    geom_data.setGeometryCollisionStatus(geom_model, 0, true);

    for (size_t k = 0; k < geom_data.activeCollisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];
      if (cp.first == 0 || cp.second == 0)
      {
        BOOST_CHECK(geom_data.activeCollisionPairs[k]);
      }
      else
      {
        BOOST_CHECK(!geom_data.activeCollisionPairs[k]);
      }
    }
  }

  // Test disableGeometryCollision
  {
    GeometryData geom_data(geom_model);
    geom_data.activateAllCollisionPairs();
    geom_data.setGeometryCollisionStatus(geom_model, 0, false);

    for (size_t k = 0; k < geom_data.activeCollisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];
      if (cp.first == 0 || cp.second == 0)
      {
        BOOST_CHECK(!geom_data.activeCollisionPairs[k]);
      }
      else
      {
        BOOST_CHECK(geom_data.activeCollisionPairs[k]);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_clone)
{
  std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector<std::string> package_dirs;
  std::string mesh_dir = PINOCCHIO_MODEL_DIR;
  package_dirs.push_back(mesh_dir);

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, package_dirs);
  geom_model.addAllCollisionPairs();

  geom_model.geometryObjects[0].geometry =
    GeometryObject::CollisionGeometryPtr(new ::coal::Sphere(0.5));
  GeometryModel geom_model_clone = geom_model.clone();
  GeometryModel geom_model_copy = geom_model;

  BOOST_CHECK(geom_model_clone == geom_model);
  BOOST_CHECK(geom_model_copy == geom_model);

  static_cast<::coal::Sphere *>(geom_model.geometryObjects[0].geometry.get())->radius = 1.;
  BOOST_CHECK(geom_model_clone != geom_model);
  BOOST_CHECK(geom_model_copy == geom_model);
}

BOOST_AUTO_TEST_SUITE_END()
