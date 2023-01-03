//
// Copyright (c) 2021-2023 INRIA
//

#include <iostream>

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/parallel/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_pool)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  Data data(model);
  
  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1,package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(model,filename,COLLISION,geometry_model,package_paths,mesh_loader);
  
  
  const int num_thread = omp_get_max_threads();
  GeometryPool pool(model,pinocchio::GeometryModel(),num_thread);
  
  pool.update(geometry_model);
  BOOST_CHECK(pool.geometry_model() == geometry_model);
  pool.update(GeometryData(geometry_model));
  
  pool.update(geometry_model,GeometryData(geometry_model));
}

BOOST_AUTO_TEST_CASE(test_talos)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  Data data(model), data_ref(model);
  
  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1,package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(model,filename,COLLISION,geometry_model,package_paths,mesh_loader);
  
  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geometry_model,srdf_filename,false);
  
  GeometryData geometry_data(geometry_model), geometry_data_ref(geometry_model);
  
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  Eigen::VectorXd q = randomConfiguration(model,-qmax,qmax);
  
  const bool res_ref = computeCollisions(model,data_ref,geometry_model,geometry_data_ref,q);
  const bool res = computeCollisions(model,data,geometry_model,geometry_data,q);
  
  BOOST_CHECK(res_ref == res);
  BOOST_CHECK(geometry_data_ref.collisionPairIndex == geometry_data.collisionPairIndex);
  
  for(size_t k = 0; k < geometry_model.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = geometry_model.collisionPairs[k];
    BOOST_CHECK(geometry_data_ref.oMg[cp.first] == geometry_data.oMg[cp.first]);
    BOOST_CHECK(geometry_data_ref.oMg[cp.second] == geometry_data.oMg[cp.second]);
    BOOST_CHECK(   geometry_data_ref.collisionResults[k].getContacts()
                == geometry_data.collisionResults[k].getContacts());
  }
}

BOOST_AUTO_TEST_CASE(test_pool_talos)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  Data data_ref(model);
  
  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1,package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(model,filename,COLLISION,geometry_model,package_paths,mesh_loader);
  
  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geometry_model,srdf_filename,false);
  
  GeometryData geometry_data_ref(geometry_model);
  
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  const Eigen::DenseIndex batch_size = 128;
  const int num_thread = omp_get_max_threads();

  Eigen::MatrixXd q(model.nq,batch_size);
  for(Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    q.col(i) = randomConfiguration(model,-qmax,qmax);
  }
  
  typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
  VectorXb res(batch_size); res.fill(false);
  VectorXb res_ref(batch_size); res_ref.fill(false);
  for(Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    res_ref[i] = computeCollisions(model,data_ref,geometry_model,geometry_data_ref,q.col(i));
  }
  
  GeometryPool pool(model,geometry_model,num_thread);
  computeCollisions(num_thread,pool,q,res);
  
  BOOST_CHECK(res == res_ref);
}

BOOST_AUTO_TEST_SUITE_END()
