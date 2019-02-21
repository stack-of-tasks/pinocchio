//
// Copyright (c) 2015-2018 CNRS
//

#include <iostream>
#include <iomanip>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <vector>
#include <boost/test/unit_test.hpp>

using namespace pinocchio;

typedef std::map <std::string, pinocchio::SE3> PositionsMap_t;
typedef std::map <std::string, pinocchio::SE3> JointPositionsMap_t;
typedef std::map <std::string, pinocchio::SE3> GeometryPositionsMap_t;
typedef std::map <std::pair < std::string , std::string >, hpp::fcl::DistanceResult > PairDistanceMap_t;
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
  
  boost::shared_ptr<hpp::fcl::Box> sample(new hpp::fcl::Box(1, 1, 1));
  Model::FrameIndex body_id_1 = model.getBodyId("planar1_body");
  Model::JointIndex joint_parent_1 = model.frames[body_id_1].parent;
  Model::JointIndex idx_geom1 = geomModel.addGeometryObject(GeometryObject("ff1_collision_object",
                                                                           model.getBodyId("planar1_body"),joint_parent_1,
                                                                           sample,SE3::Identity(), "", Eigen::Vector3d::Ones())
                                                            );
  geomModel.geometryObjects[idx_geom1].parentJoint = model.frames[body_id_1].parent;
  
  
  boost::shared_ptr<hpp::fcl::Box> sample2(new hpp::fcl::Box(1, 1, 1));
  Model::FrameIndex body_id_2 = model.getBodyId("planar2_body");
  Model::JointIndex joint_parent_2 = model.frames[body_id_2].parent;
  Model::JointIndex idx_geom2 = geomModel.addGeometryObject(GeometryObject("ff2_collision_object",
                                                                           model.getBodyId("planar2_body"),joint_parent_2,
                                                                           sample2,SE3::Identity(), "", Eigen::Vector3d::Ones()),
                                                            model);
  BOOST_CHECK(geomModel.geometryObjects[idx_geom2].parentJoint == model.frames[body_id_2].parent);

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

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  std::vector < std::string > packageDirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/romeo/";
  packageDirs.push_back(meshDir);

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );
  geomModel.addAllCollisionPairs();

  Data data(model);
  GeometryData geomData(geomModel);
  hpp::fcl::CollisionResult result;

  Eigen::VectorXd q(model.nq);
  q << 0, 0, 0.840252, 0, 0, 0, 1, 0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0, 0, -0.3490658,
       0.6981317, -0.3490658, 0, 0, 1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2, 0, 0, 0, 0,
       1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  pinocchio::Index idx = geomModel.findCollisionPair(CollisionPair(1,10));
  BOOST_CHECK(computeCollision(geomModel,geomData,idx) == false);
}


#if defined(PINOCCHIO_WITH_URDFDOM) && defined(PINOCCHIO_WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE (radius)
{
  std::vector < std::string > packageDirs;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/romeo/";
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

JointPositionsMap_t fillPinocchioJointPositions(const pinocchio::Model& model, const pinocchio::Data & data)
{
  JointPositionsMap_t result;
  for (pinocchio::Model::Index i = 0; i < (pinocchio::Model::Index)model.njoints; ++i)
  {
    result[model.names[i]] = data.oMi[i];
  }
  return result;
}

GeometryPositionsMap_t fillPinocchioGeometryPositions(const pinocchio::GeometryModel & geomModel,
                                                      const pinocchio::GeometryData & geomData)
{
  GeometryPositionsMap_t result;
  for (std::size_t i = 0; i < geomModel.ngeoms ; ++i)
  {
    result[geomModel.geometryObjects[i].name] = geomData.oMg[i];
  }
  return result;
}

std::vector<std::string> getBodiesList()
{
  std::vector<std::string> result;

  result.push_back( "CHEST_LINK0");
  result.push_back( "torso");
  result.push_back( "HEAD_LINK0");
  result.push_back( "HEAD_LINK1");
  result.push_back( "LARM_LINK0");
  result.push_back( "LARM_LINK1");
  result.push_back( "LARM_LINK2");
  result.push_back( "LARM_LINK3");
  result.push_back( "LARM_LINK4");
  result.push_back( "l_wrist");
  result.push_back( "LARM_LINK6");
  result.push_back( "LHAND_LINK0");
  result.push_back( "LHAND_LINK1");
  result.push_back( "LHAND_LINK2");
  result.push_back( "LHAND_LINK3");
  result.push_back( "LHAND_LINK4");
  result.push_back( "RARM_LINK0");
  result.push_back( "RARM_LINK1");
  result.push_back( "RARM_LINK2");
  result.push_back( "RARM_LINK3");
  result.push_back( "RARM_LINK4");
  result.push_back( "r_wrist");
  result.push_back( "RARM_LINK6");
  result.push_back( "RHAND_LINK0");
  result.push_back( "RHAND_LINK1");
  result.push_back( "RHAND_LINK2");
  result.push_back( "RHAND_LINK3");
  result.push_back( "RHAND_LINK4");
  result.push_back( "LLEG_LINK0");
  result.push_back( "LLEG_LINK1");
  result.push_back( "LLEG_LINK2");
  result.push_back( "LLEG_LINK3");
  result.push_back( "LLEG_LINK4");
  result.push_back( "l_ankle");
  result.push_back( "RLEG_LINK0");
  result.push_back( "RLEG_LINK1");
  result.push_back( "RLEG_LINK2");
  result.push_back( "RLEG_LINK3");
  result.push_back( "RLEG_LINK4");
  result.push_back( "r_ankle");

  return result;
}
