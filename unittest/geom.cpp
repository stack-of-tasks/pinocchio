//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

typedef std::map <std::string, pinocchio::SE3> PositionsMap_t;
typedef std::map <std::string, pinocchio::SE3> JointPositionsMap_t;
typedef std::map <std::string, pinocchio::SE3> GeometryPositionsMap_t;
typedef std::map <std::pair < std::string , std::string >, fcl::DistanceResult > PairDistanceMap_t;
JointPositionsMap_t fillPinocchioJointPositions(const pinocchio::Model& model, const pinocchio::Data & data);
GeometryPositionsMap_t fillPinocchioGeometryPositions(const pinocchio::GeometryModel & geomModel,
                                                      const pinocchio::GeometryData & geomData);

std::vector<std::string> getBodiesList();

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( simple_boxes )
{
  using namespace pinocchio;
  Model model;
  GeometryModel geomModel;

  Model::JointIndex idx;
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar1_joint");
  model.addJointFrame(idx);
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
  model.addBodyFrame("planar1_body", idx, SE3::Identity());
  
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar2_joint");
  model.addJointFrame(idx);
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
  model.addBodyFrame("planar2_body", idx, SE3::Identity());
  
  boost::shared_ptr<fcl::Box> sample(new fcl::Box(1, 1, 1));
  Model::FrameIndex body_id_1 = model.getBodyId("planar1_body");
  Model::JointIndex joint_parent_1 = model.frames[body_id_1].parent;
  Model::JointIndex idx_geom1 = geomModel.addGeometryObject(GeometryObject("ff1_collision_object",
                                                                           model.getBodyId("planar1_body"),joint_parent_1,
                                                                           sample,SE3::Identity(), "", Eigen::Vector3d::Ones())
                                                            );
  geomModel.geometryObjects[idx_geom1].parentJoint = model.frames[body_id_1].parent;
  
  
  boost::shared_ptr<fcl::Box> sample2(new fcl::Box(1, 1, 1));
  Model::FrameIndex body_id_2 = model.getBodyId("planar2_body");
  Model::JointIndex joint_parent_2 = model.frames[body_id_2].parent;
  Model::JointIndex idx_geom2 = geomModel.addGeometryObject(GeometryObject("ff2_collision_object",
                                                                           model.getBodyId("planar2_body"),joint_parent_2,
                                                                           sample2,SE3::Identity(), "", Eigen::Vector3d::Ones()),
                                                            model);
  BOOST_CHECK(geomModel.geometryObjects[idx_geom2].parentJoint == model.frames[body_id_2].parent);
  
  boost::shared_ptr<fcl::Box> universe_body_geometry(new fcl::Box(1, 1, 1));
  model.addBodyFrame("universe_body", 0, SE3::Identity());
  Model::FrameIndex body_id_3 = model.getBodyId("universe_body");
  Model::JointIndex joint_parent_3 = model.frames[body_id_3].parent;
  SE3 universe_body_placement = SE3::Random();
  Model::JointIndex idx_geom3 = geomModel.addGeometryObject(GeometryObject("universe_collision_object",
                                                                           model.getBodyId("universe_body"),joint_parent_3,
                                                                           universe_body_geometry,universe_body_placement, "", Eigen::Vector3d::Ones()),
                                                            model);
  
  BOOST_CHECK(geomModel.geometryObjects[idx_geom3].parentJoint == model.frames[body_id_3].parent);

  geomModel.addAllCollisionPairs();
  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geomModel);

  BOOST_CHECK(CollisionPair(0,1) == geomModel.collisionPairs[0]);

  std::cout << "------ Model ------ " << std::endl;
  std::cout << model;
  std::cout << "------ Geom ------ " << std::endl;
  std::cout << geomModel;
  std::cout << "------ DataGeom ------ " << std::endl;
  std::cout << geomData;

  Eigen::VectorXd q(model.nq);
  q <<  0, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(geomData.oMg[idx_geom3].isApprox(universe_body_placement));
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == true);

  q <<  2, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == false);

  q <<  0.99, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == true);

  q <<  1.01, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == false);
}

BOOST_AUTO_TEST_CASE ( loading_model )
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;
  typedef pinocchio::Data Data;
  typedef pinocchio::GeometryData GeometryData;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > packageDirs;
  std::string meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  packageDirs.push_back(meshDir);

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );
  geomModel.addAllCollisionPairs();
  
  GeometryModel geomModelOther = pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );
  BOOST_CHECK(geomModelOther == geomModel);

  Data data(model);
  GeometryData geomData(geomModel);
  fcl::CollisionResult result;

  Eigen::VectorXd q(model.nq);
  q << 0, 0, 0.840252, 0, 0, 0, 1, 0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0, 0, -0.3490658,
       0.6981317, -0.3490658, 0, 0, 1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2, 0, 0, 0, 0,
       1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  pinocchio::Index idx = geomModel.findCollisionPair(CollisionPair(1,10));
  BOOST_CHECK(computeCollision(geomModel,geomData,idx) == false);
  
  fcl::DistanceResult distance_res = computeDistance(geomModel,geomData,idx);
  BOOST_CHECK(distance_res.min_distance > 0.);
}
  
BOOST_AUTO_TEST_CASE ( test_collisions )
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;
  typedef pinocchio::Data Data;
  typedef pinocchio::GeometryData GeometryData;
  
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > packageDirs;
  const std::string meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  packageDirs.push_back(meshDir);
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/srdf/romeo.srdf");
  
  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, packageDirs);
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geom_model,srdf_filename,false);
  
  Data data(model);
  GeometryData geom_data(geom_model);
  fcl::CollisionResult result;
  
  pinocchio::srdf::loadReferenceConfigurations(model,srdf_filename,false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

  BOOST_CHECK(computeCollisions(geom_model,geom_data) == false);
  
  {
    Data data(model);
    GeometryData geom_data(geom_model);
    BOOST_CHECK(computeCollisions(model,data,geom_model,geom_data,q) == false);
  }
}
  
BOOST_AUTO_TEST_CASE ( test_distances )
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;
  typedef pinocchio::Data Data;
  typedef pinocchio::GeometryData GeometryData;
  
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > packageDirs;
  const std::string meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  packageDirs.push_back(meshDir);
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/srdf/romeo.srdf");
  
  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model, packageDirs);
  geom_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geom_model,srdf_filename,false);
  
  Data data(model);
  GeometryData geom_data(geom_model);
  fcl::CollisionResult result;
  
  pinocchio::srdf::loadReferenceConfigurations(model,srdf_filename,false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];
  
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);
  
  BOOST_CHECK(computeDistances(geom_model,geom_data) < geom_model.collisionPairs.size());
  
  {
    Data data(model);
    GeometryData geom_data(geom_model);
    BOOST_CHECK(computeDistances(model,data,geom_model,geom_data,q) < geom_model.collisionPairs.size());
  }
}
  
BOOST_AUTO_TEST_CASE ( test_append_geom_models )
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > packageDirs;
  const std::string meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  packageDirs.push_back(meshDir);
  
  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geom_model1;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model1, packageDirs);
  
  GeometryModel geom_model2;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom_model2, packageDirs);
  
  appendGeometryModel(geom_model2,geom_model1);
  BOOST_CHECK(geom_model2.ngeoms == 2*geom_model1.ngeoms);
  
  {
    GeometryModel geom_model_empty;
    GeometryModel geom_model;
    BOOST_CHECK(geom_model_empty.ngeoms == 0);
    appendGeometryModel(geom_model,geom_model_empty);
    BOOST_CHECK(geom_model.ngeoms== 0);
  }
}
  
#if defined(PINOCCHIO_WITH_URDFDOM) && defined(PINOCCHIO_WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE (radius)
{
  std::vector < std::string > packageDirs;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::string meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots/");
  packageDirs.push_back(meshDir);

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  pinocchio::GeometryModel geom;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom, packageDirs);
  Data data(model);
  GeometryData geomData(geom);

  // Test that the algorithm does not crash
  pinocchio::computeBodyRadius(model, geom, geomData);
  BOOST_FOREACH( double radius, geomData.radius) BOOST_CHECK(radius>=0.);
}
#endif // if defined(PINOCCHIO_WITH_URDFDOM) && defined(PINOCCHIO_WITH_HPP_FCL)

BOOST_AUTO_TEST_SUITE_END ()
