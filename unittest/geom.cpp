//
// Copyright (c) 2015-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

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

using namespace se3;

typedef std::map <std::string, se3::SE3> PositionsMap_t;
typedef std::map <std::string, se3::SE3> JointPositionsMap_t;
typedef std::map <std::string, se3::SE3> GeometryPositionsMap_t;
typedef std::map <std::pair < std::string , std::string >, fcl::DistanceResult > PairDistanceMap_t;
JointPositionsMap_t fillPinocchioJointPositions(const se3::Model& model, const se3::Data & data);
GeometryPositionsMap_t fillPinocchioGeometryPositions(const se3::GeometryModel & geomModel,
                                                      const se3::GeometryData & geomData);

std::vector<std::string> getBodiesList();

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( simple_boxes )
{
  using namespace se3;
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
  geomModel.addGeometryObject(GeometryObject("ff1_collision_object",
                                             model.getBodyId("planar1_body"),0,
                                             sample,SE3::Identity(), "", Eigen::Vector3d::Ones()),
                              model,true);
  
  boost::shared_ptr<fcl::Box> sample2(new fcl::Box(1, 1, 1));
  geomModel.addGeometryObject(GeometryObject("ff2_collision_object",
                                             model.getBodyId("planar2_body"),0,
                                             sample2,SE3::Identity(), "", Eigen::Vector3d::Ones()),
                              model,true);

  geomModel.addAllCollisionPairs();
  se3::Data data(model);
  se3::GeometryData geomData(geomModel);

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

  se3::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == true);

  q <<  2, 0, 1, 0,
        0, 0, 1, 0 ;

  se3::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == false);

  q <<  0.99, 0, 1, 0,
        0, 0, 1, 0 ;

  se3::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == true);

  q <<  1.01, 0, 1, 0,
        0, 0, 1, 0 ;

  se3::updateGeometryPlacements(model, data, geomModel, geomData, q);
  BOOST_CHECK(computeCollision(geomModel,geomData,0) == false);
}

BOOST_AUTO_TEST_CASE ( loading_model )
{
  typedef se3::Model Model;
  typedef se3::GeometryModel GeometryModel;
  typedef se3::Data Data;
  typedef se3::GeometryData GeometryData;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  std::vector < std::string > packageDirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/romeo/";
  packageDirs.push_back(meshDir);

  Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  GeometryModel geomModel;
  se3::urdf::buildGeom(model, filename, se3::COLLISION, geomModel, packageDirs );
  geomModel.addAllCollisionPairs();

  Data data(model);
  GeometryData geomData(geomModel);
  fcl::CollisionResult result;

  Eigen::VectorXd q(model.nq);
  q << 0, 0, 0.840252, 0, 0, 0, 1, 0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0, 0, -0.3490658,
       0.6981317, -0.3490658, 0, 0, 1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2, 0, 0, 0, 0,
       1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2 ;

  se3::updateGeometryPlacements(model, data, geomModel, geomData, q);
  se3::Index idx = geomModel.findCollisionPair(CollisionPair(1,10));
  BOOST_CHECK(computeCollision(geomModel,geomData,idx) == false);
}


#if defined(WITH_URDFDOM) && defined(WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE (radius)
{
  std::vector < std::string > packageDirs;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/romeo/";
  packageDirs.push_back(meshDir);

  se3::Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  se3::GeometryModel geom;
  se3::urdf::buildGeom(model, filename, se3::COLLISION, geom, packageDirs);
  Data data(model);
  GeometryData geomData(geom);

  // Test that the algorithm does not crash
  se3::computeBodyRadius(model, geom, geomData);
  BOOST_FOREACH( double radius, geomData.radius) BOOST_CHECK(radius>=0.);
}
#endif // if defined(WITH_URDFDOM) && defined(WITH_HPP_FCL)

BOOST_AUTO_TEST_SUITE_END ()

JointPositionsMap_t fillPinocchioJointPositions(const se3::Model& model, const se3::Data & data)
{
  JointPositionsMap_t result;
  for (se3::Model::Index i = 0; i < (se3::Model::Index)model.njoints; ++i)
  {
    result[model.names[i]] = data.oMi[i];
  }
  return result;
}

GeometryPositionsMap_t fillPinocchioGeometryPositions(const se3::GeometryModel & geomModel,
                                                      const se3::GeometryData & geomData)
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
