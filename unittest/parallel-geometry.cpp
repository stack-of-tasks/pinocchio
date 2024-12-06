//
// Copyright (c) 2021-2023 INRIA
//

#include <iostream>

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/pool/fwd.hpp"
#include "pinocchio/collision/pool/broadphase-manager.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"
#include "pinocchio/collision/parallel/broadphase.hpp"
#include "pinocchio/collision/parallel/geometry.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/mesh_loader/loader.h>

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_geometry_pool)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  Data data(model);

  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = std::make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1, package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(
    model, filename, COLLISION, geometry_model, package_paths, mesh_loader);

  const size_t num_thread = (size_t)omp_get_max_threads();
  pinocchio::GeometryModel geometry_model_empty;
  GeometryPool pool(model, geometry_model_empty, num_thread);

  pool.update(GeometryData(geometry_model));
}

BOOST_AUTO_TEST_CASE(test_broadphase_pool)
{
  Model model;
  model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "ff");

  Data data(model);
  GeometryModel geom_model;

  hpp::fcl::CollisionGeometryPtr_t sphere_ptr(new hpp::fcl::Sphere(0.1));
  hpp::fcl::CollisionGeometryPtr_t sphere2_ptr(new hpp::fcl::Sphere(0.1));

  GeometryObject obj1("obj1", 1, SE3::Identity(), sphere_ptr);
  geom_model.addGeometryObject(obj1);

  GeometryObject obj2("obj2", 0, SE3::Identity(), sphere2_ptr);
  const GeomIndex obj2_index = geom_model.addGeometryObject(obj2);

  geom_model.addAllCollisionPairs();

  const GeometryModel geom_model_clone = geom_model.clone();

  //  GeometryObject & go1 = geom_model.geometryObjects[obj_index];

  const size_t num_thread = (size_t)omp_get_max_threads();
  typedef BroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> BroadPhaseManager;
  typedef BroadPhaseManagerPoolBase<BroadPhaseManager, double> BroadPhaseManagerPool;
  BroadPhaseManagerPool pool(model, geom_model, num_thread);

  auto manager = pool.getBroadPhaseManager(0);
  GeometryData & geom_data = manager.getGeometryData();

  BOOST_CHECK(pool.check());

  const int batch_size = 256;
  Eigen::MatrixXd qs(model.nq, batch_size);
  for (int i = 0; i < batch_size; ++i)
  {
    const SE3 placement = SE3::Random();
    qs.col(i).head<3>() = 0.3 * placement.translation();
    qs.col(i).tail<4>() = Eigen::Quaterniond(placement.rotation()).coeffs();
  }

  typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorBool;
  VectorBool res_all_before(batch_size), res_all_before_ref(batch_size);

  // Check potential memory leack
  {
    Data data_ref(model);
    GeometryData geom_data_ref(geom_model);

    for (int i = 0; i < batch_size; ++i)
    {
      const bool res = computeCollisions(model, data, manager, qs.col(i), false);
      const bool res_ref =
        computeCollisions(model, data_ref, geom_model, geom_data_ref, qs.col(i), false);

      BOOST_CHECK(res == res_ref);

      res_all_before_ref[i] = res_ref;
    }

    // Do collision checking
    computeCollisionsInParallel(num_thread, pool, qs, res_all_before, false);
    BOOST_CHECK(res_all_before == res_all_before_ref);
  }

  static_cast<hpp::fcl::Sphere *>(geom_model.geometryObjects[obj2_index].geometry.get())->radius =
    100;
  geom_model.geometryObjects[obj2_index].geometry->computeLocalAABB();
  BOOST_CHECK(static_cast<hpp::fcl::Sphere *>(sphere2_ptr.get())->radius == 100);

  for (GeometryModel & geom_model_pool : pool.getGeometryModels())
  {
    geom_model_pool.geometryObjects[obj2_index] = geom_model.geometryObjects[obj2_index].clone();
  }

  pool.update(geom_data);

  VectorBool res_all_intermediate(batch_size), res_all_intermediate_ref(batch_size);
  {
    Data data_ref(model);
    GeometryData geom_data_ref(geom_model);

    for (int i = 0; i < batch_size; ++i)
    {
      const bool res = computeCollisions(model, data, manager, qs.col(i), false);
      const bool res_ref =
        computeCollisions(model, data_ref, geom_model, geom_data_ref, qs.col(i), false);

      BOOST_CHECK(res == res_ref);

      res_all_intermediate_ref[i] = res_ref;
    }

    // Do collision checking
    computeCollisionsInParallel(num_thread, pool, qs, res_all_intermediate, false);
    BOOST_CHECK(res_all_intermediate == res_all_intermediate_ref);
  }

  BOOST_CHECK(res_all_intermediate != res_all_before);

  static_cast<hpp::fcl::Sphere *>(sphere2_ptr.get())->radius = 0.1;

  hpp::fcl::CollisionGeometryPtr_t new_sphere2_ptr(
    new hpp::fcl::Sphere(static_cast<hpp::fcl::Sphere &>(*sphere2_ptr.get())));
  new_sphere2_ptr->computeLocalAABB();
  geom_model.geometryObjects[obj2_index].geometry = new_sphere2_ptr;
  BOOST_CHECK(
    static_cast<hpp::fcl::Sphere *>(geom_model.geometryObjects[obj2_index].geometry.get())->radius
    == static_cast<hpp::fcl::Sphere *>(new_sphere2_ptr.get())->radius);
  BOOST_CHECK(geom_model.geometryObjects[obj2_index].geometry.get() == new_sphere2_ptr.get());
  BOOST_CHECK(geom_model.geometryObjects[obj2_index].geometry.get() != sphere2_ptr.get());
  BOOST_CHECK(
    *geom_model.geometryObjects[obj2_index].geometry.get() == *new_sphere2_ptr.get()->clone());

  for (GeometryModel & geom_model_pool : pool.getGeometryModels())
  {
    geom_model_pool.geometryObjects[obj2_index] = geom_model.geometryObjects[obj2_index].clone();
  }

  BOOST_CHECK(not pool.check());
  pool.update(geom_data);
  BOOST_CHECK(pool.check());

  VectorBool res_all_final(batch_size), res_all_final_ref(batch_size);
  {
    Data data_ref(model);
    GeometryData geom_data_ref(geom_model);

    for (int i = 0; i < batch_size; ++i)
    {
      const bool res = computeCollisions(model, data, manager, qs.col(i), false);
      const bool res_ref =
        computeCollisions(model, data_ref, geom_model, geom_data_ref, qs.col(i), false);

      BOOST_CHECK(res == res_ref);

      res_all_final_ref[i] = res_ref;
    }

    // Do collision checking
    computeCollisionsInParallel(num_thread, pool, qs, res_all_final, false);
    BOOST_CHECK(res_all_final == res_all_final_ref);
  }

  BOOST_CHECK(res_all_final == res_all_before);

  std::cout << "res_all_before: " << res_all_before.transpose() << std::endl;
  std::cout << "res_all_before_ref: " << res_all_before_ref.transpose() << std::endl;
  std::cout << "res_all_intermediate: " << res_all_intermediate.transpose() << std::endl;
  std::cout << "res_all_intermediate_ref: " << res_all_intermediate_ref.transpose() << std::endl;
  std::cout << "res_all_final: " << res_all_final.transpose() << std::endl;
  std::cout << "res_all_final_ref: " << res_all_final_ref.transpose() << std::endl;
}

BOOST_AUTO_TEST_CASE(test_talos)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  Data data(model), data_ref(model);

  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = std::make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1, package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(
    model, filename, COLLISION, geometry_model, package_paths, mesh_loader);

  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geometry_model, srdf_filename, false);

  GeometryData geometry_data(geometry_model), geometry_data_ref(geometry_model);

  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  Eigen::VectorXd q = randomConfiguration(model, -qmax, qmax);

  const bool res_ref = computeCollisions(model, data_ref, geometry_model, geometry_data_ref, q);
  const bool res = computeCollisions(model, data, geometry_model, geometry_data, q);

  BOOST_CHECK(res_ref == res);
  BOOST_CHECK(geometry_data_ref.collisionPairIndex == geometry_data.collisionPairIndex);

  for (size_t k = 0; k < geometry_model.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = geometry_model.collisionPairs[k];
    BOOST_CHECK(geometry_data_ref.oMg[cp.first] == geometry_data.oMg[cp.first]);
    BOOST_CHECK(geometry_data_ref.oMg[cp.second] == geometry_data.oMg[cp.second]);
    BOOST_REQUIRE_EQUAL(
      geometry_data_ref.collisionResults[k].getContacts().size(),
      geometry_data.collisionResults[k].getContacts().size());
    // This code is a workaround for https://github.com/coal-library/coal/issues/636
    for (size_t l = 0; l < geometry_data.collisionResults[k].getContacts().size(); ++l)
    {
      const auto & contact = geometry_data.collisionResults[k].getContacts()[l];
      const auto & contact_ref = geometry_data_ref.collisionResults[k].getContacts()[l];

      // If contact is not filled with NaN do the standard comparison
      if (contact.normal == contact.normal)
      {
        BOOST_CHECK(contact == contact_ref);
      }
      else
      {
        // Compare standard values
        BOOST_CHECK_EQUAL(contact.o1, contact_ref.o1);
        BOOST_CHECK_EQUAL(contact.o2, contact_ref.o2);
        BOOST_CHECK_EQUAL(contact.b1, contact_ref.b1);
        BOOST_CHECK_EQUAL(contact.b2, contact_ref.b2);
        BOOST_CHECK_EQUAL(contact.penetration_depth, contact_ref.penetration_depth);

        // Check all is set to NaN
        BOOST_CHECK(contact.normal != contact.normal);
        BOOST_CHECK(contact.pos != contact.pos);
        BOOST_CHECK(contact.nearest_points[0] != contact.nearest_points[0]);
        BOOST_CHECK(contact.nearest_points[1] != contact.nearest_points[1]);
        BOOST_CHECK(contact_ref.normal != contact_ref.normal);
        BOOST_CHECK(contact_ref.pos != contact_ref.pos);
        BOOST_CHECK(contact_ref.nearest_points[0] != contact_ref.nearest_points[0]);
        BOOST_CHECK(contact_ref.nearest_points[1] != contact_ref.nearest_points[1]);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_pool_talos_memory)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");

  pinocchio::Model * model_ptr = new Model();
  Model & model = *model_ptr;
  pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  Data data_ref(model);

  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = std::make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1, package_path);
  pinocchio::GeometryModel * geometry_model_ptr = new GeometryModel();
  GeometryModel & geometry_model = *geometry_model_ptr;
  pinocchio::urdf::buildGeom(model, filename, COLLISION, geometry_model, package_paths);

  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geometry_model, srdf_filename, false);

  typedef BroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> BroadPhaseManager;
  typedef BroadPhaseManagerPoolBase<BroadPhaseManager, double> BroadPhaseManagerPool;

  const size_t num_thread = (size_t)omp_get_max_threads();
  ;
  BroadPhaseManagerPool broadphase_manager_pool(model, geometry_model, num_thread);

  const Eigen::DenseIndex batch_size = 2048;
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  Eigen::MatrixXd q(model.nq, batch_size);
  for (Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    q.col(i) = randomConfiguration(model, -qmax, qmax);
  }

  delete model_ptr;
  delete geometry_model_ptr;

  typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
  VectorXb res(batch_size);
  computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res);
}

BOOST_AUTO_TEST_CASE(test_pool_talos)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  Data data_ref(model);

  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = std::make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1, package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(
    model, filename, COLLISION, geometry_model, package_paths, mesh_loader);

  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, geometry_model, srdf_filename, false);

  GeometryData geometry_data_ref(geometry_model);

  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  const Eigen::DenseIndex batch_size = 2048;
  const size_t num_thread = (size_t)omp_get_max_threads();

  Eigen::MatrixXd q(model.nq, batch_size);
  for (Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    q.col(i) = randomConfiguration(model, -qmax, qmax);
  }

  typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

  VectorXb res_ref(batch_size);
  res_ref.fill(false);
  for (Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    res_ref[i] = computeCollisions(model, data_ref, geometry_model, geometry_data_ref, q.col(i));
  }
  BOOST_CHECK(res_ref.sum() > 0);

  {
    VectorXb res(batch_size);
    res.fill(false);
    GeometryPool geometry_pool(model, geometry_model, num_thread);
    computeCollisionsInParallel(num_thread, geometry_pool, q, res);

    BOOST_CHECK(res == res_ref);
  }

  {
    typedef BroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> BroadPhaseManager;
    typedef BroadPhaseManagerPoolBase<BroadPhaseManager, double> BroadPhaseManagerPool;

    BroadPhaseManagerPool broadphase_manager_pool(model, geometry_model, num_thread);
    VectorXb res1(batch_size), res2(batch_size), res3(batch_size), res4(batch_size);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res1);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res2, true);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res3, true, true);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res4, false, true);

    BOOST_CHECK(res1 == res_ref);
    BOOST_CHECK(res2 == res_ref);

    for (Eigen::DenseIndex k = 0; k < batch_size; ++k)
    {
      if (res3[k])
        BOOST_CHECK(res_ref[k]);
      if (res4[k])
        BOOST_CHECK(res_ref[k]);
    }
  }

  {
    typedef TreeBroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager> BroadPhaseManager;
    typedef BroadPhaseManagerPoolBase<BroadPhaseManager, double> BroadPhaseManagerPool;

    BroadPhaseManagerPool broadphase_manager_pool(model, geometry_model, num_thread);
    VectorXb res1(batch_size), res2(batch_size), res3(batch_size), res4(batch_size);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res1);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res2, true);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res3, true, true);
    computeCollisionsInParallel(num_thread, broadphase_manager_pool, q, res4, false, true);

    BOOST_CHECK(res1 == res_ref);
    BOOST_CHECK(res2 == res_ref);

    for (Eigen::DenseIndex k = 0; k < batch_size; ++k)
    {
      if (res3[k])
        BOOST_CHECK(res_ref[k]);
      if (res4[k])
        BOOST_CHECK(res_ref[k]);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
