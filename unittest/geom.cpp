//
// Copyright (c) 2015-2016 CNRS
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

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <vector>

#ifdef WITH_HPP_MODEL_URDF
#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/urdf/util.hh>
#endif

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE GeomTest
#include <boost/test/unit_test.hpp>

using namespace se3;

typedef std::map <std::string, se3::SE3> PositionsMap_t;
typedef std::map <std::string, se3::SE3> JointPositionsMap_t;
typedef std::map <std::string, se3::SE3> GeometryPositionsMap_t;
typedef std::map <std::pair < std::string , std::string >, fcl::DistanceResult > PairDistanceMap_t;
JointPositionsMap_t fillPinocchioJointPositions(const se3::Model& model, const se3::Data & data);
GeometryPositionsMap_t fillPinocchioGeometryPositions(const se3::GeometryData & data_geom);
#ifdef WITH_HPP_MODEL_URDF
JointPositionsMap_t fillHppJointPositions(const hpp::model::HumanoidRobotPtr_t robot);
GeometryPositionsMap_t fillHppGeometryPositions(const hpp::model::HumanoidRobotPtr_t robot);
#endif
std::vector<std::string> getBodiesList();

#ifdef WITH_HPP_MODEL_URDF
class IsDistanceResultPair
{
typedef std::string string;
public:
IsDistanceResultPair( string body1, string body2): _body1(body1), _body2(body2){}

bool operator()(hpp::model::DistanceResult dist) const
{
  return (dist.innerObject->name() == _body1 && dist.outerObject->name() == _body2);
}
private:
string _body1;
string _body2;
};

struct Distance_t
{
  Distance_t () :d_ (0), x0_ (0), y0_ (0), z0_ (0), x1_ (0), y1_ (0), z1_ (0)
  {
  }
  Distance_t (double d, double x0, double y0, double z0,
        double x1, double y1, double z1) : d_ (d),
             x0_ (x0), y0_ (y0), z0_ (z0),
             x1_ (x1), y1_ (y1), z1_ (z1)
  {
  }
  Distance_t ( fcl::DistanceResult dr) : d_ (dr.min_distance),
   x0_ (dr.nearest_points [0][0]), y0_ (dr.nearest_points [0][1]), z0_ (dr.nearest_points [0][2]),
   x1_ (dr.nearest_points [1][0]), y1_ (dr.nearest_points [1][1]), z1_ (dr.nearest_points [1][2])
  {
  }
  Distance_t (const Distance_t& other) :
    d_ (other.d_), x0_ (other.x0_), y0_ (other.y0_), z0_ (other.z0_),
    x1_ (other.x1_), y1_ (other.y1_), z1_ (other.z1_)
  {
  }
  Distance_t& swap ()
  {
    double tmp;
    tmp = x1_; x1_ = x0_; x0_ = tmp;
    tmp = y1_; y1_ = y0_; y0_ = tmp;
    tmp = z1_; z1_ = z0_; z0_ = tmp;
    return *this;
  }
  void checkClose (const Distance_t& other)
  {
    BOOST_CHECK_MESSAGE (fabs (d_ - other.d_ ) < 1e-5, "values d are " << d_ << " and " << other.d_);
    BOOST_CHECK_MESSAGE (fabs (x0_ - other.x0_ ) < 1e-5, "values x0 are " << x0_ << " and " << other.x0_);
    BOOST_CHECK_MESSAGE (fabs (y0_ - other.y0_ ) < 1e-5, "values y0 are " << y0_ << " and " << other.y0_);
    BOOST_CHECK_MESSAGE (fabs (z0_ - other.z0_ ) < 1e-5, "values z0 are " << z0_ << " and " << other.z0_);
    BOOST_CHECK_MESSAGE (fabs (x1_ - other.x1_ ) < 1e-5, "values x1 are " << x1_ << " and " << other.x1_);
    BOOST_CHECK_MESSAGE (fabs (y1_ - other.y1_ ) < 1e-5, "values y1 are " << y1_ << " and " << other.y1_);
    BOOST_CHECK_MESSAGE (fabs (z1_ - other.z1_ ) < 1e-5, "values z1 are " << z1_ << " and " << other.z1_);
  }
  // Distance and closest points on bodies.
  double d_, x0_, y0_, z0_, x1_, y1_, z1_;
}; // struct Distance_t

void loadHumanoidPathPlanerModel (const hpp::model::HumanoidRobotPtr_t& robot,
            const std::string& rootJointType,
            const std::string& package,
            const std::string& modelName,
            const std::string& urdfSuffix)
{
  hpp::model::urdf::Parser urdfParser (rootJointType, robot);
  urdfParser.prefix ("");

  std::string urdfPath = "package://" + package + "/urdf/"
    + modelName + urdfSuffix + ".urdf";

  // Build robot model from URDF.
  urdfParser.parse (urdfPath);

}

#endif


BOOST_AUTO_TEST_SUITE ( GeomTest )

BOOST_AUTO_TEST_CASE ( GeomNoFcl )
{

}

BOOST_AUTO_TEST_CASE ( simple_boxes )
{
  using namespace se3;
  Model model;
  GeometryModel model_geom;

  Model::JointIndex idx;
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar1_joint");
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"planar1_body");
  
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar2_joint");
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity(),"planar2_body");
  
  boost::shared_ptr<fcl::Box> sample(new fcl::Box(1));
  model_geom.addGeometryObject(model, model.getBodyId("planar1_body"),sample, SE3::Identity(),  "ff1_collision_object", "");
  
  boost::shared_ptr<fcl::Box> sample2(new fcl::Box(1));
  model_geom.addGeometryObject(model, model.getBodyId("planar2_body"),sample2, SE3::Identity(),  "ff2_collision_object", "");

  se3::Data data(model);
  se3::GeometryData data_geom(model_geom);

  std::cout << "------ Model ------ " << std::endl;
  std::cout << model;
  std::cout << "------ Geom ------ " << std::endl;
  std::cout << model_geom;
  std::cout << "------ DataGeom ------ " << std::endl;
  std::cout << data_geom;
  BOOST_CHECK(data_geom.computeCollision(CollisionPair(0,1)).fcl_collision_result.isCollision() == true);

  Eigen::VectorXd q(model.nq);
  q <<  2, 0, 0,
        0, 0, 0 ;

  se3::updateGeometryPlacements(model, data, model_geom, data_geom, q);
  std::cout << data_geom;
  BOOST_CHECK(data_geom.computeCollision(CollisionPair(0,1)).fcl_collision_result.isCollision() == false);

  q <<  0.99, 0, 0,
        0, 0, 0 ;

  se3::updateGeometryPlacements(model, data, model_geom, data_geom, q);
  std::cout << data_geom;
  BOOST_CHECK(data_geom.computeCollision(CollisionPair(0,1)).fcl_collision_result.isCollision() == true);

  q <<  1.01, 0, 0,
        0, 0, 0 ;

  se3::updateGeometryPlacements(model, data, model_geom, data_geom, q);
  std::cout << data_geom;
  BOOST_CHECK(data_geom.computeCollision(CollisionPair(0,1)).fcl_collision_result.isCollision() == false);
}

BOOST_AUTO_TEST_CASE ( loading_model )
{
  typedef se3::Model Model;
  typedef se3::GeometryModel GeometryModel;
  typedef se3::Data Data;
  typedef se3::GeometryData GeometryData;


  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::vector < std::string > package_dirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  package_dirs.push_back(meshDir);

  Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  GeometryModel geometry_model = se3::urdf::buildGeom(model, filename, package_dirs, se3::COLLISION);

  Data data(model);
  GeometryData geometry_data(geometry_model);

  Eigen::VectorXd q(model.nq);
  q << 0, 0, 0.840252, 0, 0, 0, 1, 0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0, 0, -0.3490658,
       0.6981317, -0.3490658, 0, 0, 1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2, 0, 0, 0, 0,
       1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2 ;

  se3::updateGeometryPlacements(model, data, geometry_model, geometry_data, q);
  BOOST_CHECK(geometry_data.computeCollision(CollisionPair(1,10)).fcl_collision_result.isCollision() == false);
}


#if defined(WITH_URDFDOM) && defined(WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE (radius)
{
  // Building the model in pinocchio and compute kinematics/geometry for configuration q_pino
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::vector < std::string > package_dirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  package_dirs.push_back(meshDir);

  se3::Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  se3::GeometryModel geom = se3::urdf::buildGeom(model, filename, package_dirs, se3::COLLISION);
  Data data(model);
  GeometryData geomData(geom);

  // Test that the algorithm does not crash
  se3::computeBodyRadius(model, geom, geomData);
  BOOST_FOREACH( double radius, geomData.radius) BOOST_CHECK(radius>=0.);

#ifdef WITH_HPP_MODEL_URDF
  /// *************  HPP  ************* /// 
  /// ********************************* ///
  using hpp::model::JointVector_t;
  using hpp::model::Joint;
  using hpp::model::Body;
  using hpp::model::BodyPtr_t;

  hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("romeo");
  loadHumanoidPathPlanerModel(humanoidRobot, "freeflyer",
              "romeo_pinocchio", "romeo",
              "");

  BOOST_CHECK_MESSAGE(model.nq == humanoidRobot->configSize () , "Pinocchio model & HPP model config sizes are not the same ");

  /// **********  COMPARISON  ********* /// 
  /// ********************************* ///

  JointVector_t joints = humanoidRobot->getJointVector ();
  for (JointVector_t::iterator it = joints.begin (); it != joints.end (); ++it)
  {
    BodyPtr_t body = (*it)->linkedBody ();
    if (body) {
      double radius_hpp = body->radius();
      std::string bodyName = body->name();
      if(bodyName != "base_link")
      {
        double radius_pino = geomData.radius[model.getFrameParent(bodyName)];
        BOOST_CHECK_MESSAGE(radius_hpp - radius_pino < 1e-6, "Radius of body " << bodyName << " are not equals between hpp and pinocchio");
      }

    }
  }
#endif // WITH_HPP_MODEL_URDF
}
#endif // if defined(WITH_URDFDOM) && defined(WITH_HPP_FCL)

#ifdef WITH_HPP_MODEL_URDF
BOOST_AUTO_TEST_CASE ( romeo_joints_meshes_positions )
{
  typedef se3::Model Model;
  typedef se3::GeometryModel GeometryModel;
  typedef se3::Data Data;
  typedef se3::GeometryData GeometryData;
  using hpp::model::Device;
  using hpp::model::Joint;
  using hpp::model::Body;
  typedef hpp::model::ObjectVector_t ObjectVector_t;
  typedef hpp::model::JointVector_t JointVector_t;
  typedef std::vector<double> vector_t;



  /// **********  Pinocchio  ********** /// 
  /// ********************************* ///

  // Building the model in pinocchio and compute kinematics/geometry for configuration q_pino
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::vector < std::string > package_dirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  package_dirs.push_back(meshDir);

  se3::Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  se3::GeometryModel geom = se3::urdf::buildGeom(model, filename, package_dirs, se3::COLLISION);
  std::cout << model << std::endl;


  Data data(model);
  GeometryData data_geom(geom);

  // Configuration to be tested
  

  Eigen::VectorXd q_pino(Eigen::VectorXd::Random(model.nq));
  q_pino.segment<4>(3) /= q_pino.segment<4>(3).norm();

  Eigen::VectorXd q_hpp(q_pino);
  Eigen::Vector4d quaternion;
  quaternion <<  q_pino[6], q_pino[3], q_pino[4], q_pino[5];
  q_hpp.segment<4>(3) = quaternion ;

  BOOST_CHECK_MESSAGE(q_pino.size() == model.nq , "wrong config size" );

  se3::updateGeometryPlacements(model, data, geom, data_geom, q_pino);


  /// *************  HPP  ************* /// 
  /// ********************************* ///


  hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("romeo");
  loadHumanoidPathPlanerModel(humanoidRobot, "freeflyer",
              "romeo_pinocchio", "romeo",
              "");

  BOOST_CHECK_MESSAGE(model.nq == humanoidRobot->configSize () , "Pinocchio model & HPP model config sizes are not the same ");

  humanoidRobot->currentConfiguration (q_hpp);
  humanoidRobot->computeForwardKinematics ();



  /// **********  COMPARISON  ********* /// 
  /// ********************************* ///
  // retrieve all joint and geometry objects positions
  JointPositionsMap_t joints_pin  = fillPinocchioJointPositions(model, data);
  JointPositionsMap_t joints_hpp  = fillHppJointPositions(humanoidRobot);
  GeometryPositionsMap_t geom_pin = fillPinocchioGeometryPositions(data_geom);
  GeometryPositionsMap_t geom_hpp = fillHppGeometryPositions(humanoidRobot);


  std::map <std::string, se3::SE3>::iterator it_hpp;
  std::map <std::string, se3::SE3>::iterator it_pin;

  // Comparing position of joints
  se3::SE3 ff_pin = joints_pin["root_joint"];
  se3::SE3 ff_hpp = joints_hpp["base_joint_SO3"];
  BOOST_CHECK_MESSAGE(ff_pin.isApprox(ff_hpp) , "ff_hpp and ff_pin are not ==");

  for (it_pin = joints_pin.begin(); it_pin != joints_pin.end();
        ++it_pin)
  {
    it_hpp = joints_hpp.find(it_pin->first);
    if (it_hpp != joints_hpp.end())
    {
      BOOST_CHECK_MESSAGE(it_pin->second.isApprox(it_hpp->second) , "joint positions are not equal");
      // se3::Motion nu = se3::log6(it_pin->second.inverse() * it_hpp->second);
    }
  }

  // Comparing position of geometry objects
  for (it_pin = geom_pin.begin(); it_pin != geom_pin.end();
        ++it_pin)
  {
    it_hpp = geom_hpp.find(it_pin->first);
    if (it_hpp != geom_hpp.end())
    {
      BOOST_CHECK_MESSAGE(it_pin->second.isApprox(it_hpp->second) , "geometry objects positions are not equal");
    }
  }


}

BOOST_AUTO_TEST_CASE ( hrp2_mesh_distance)
{
  typedef se3::Model Model;
  typedef se3::GeometryModel GeometryModel;
  typedef se3::Data Data;
  typedef se3::GeometryData GeometryData;
  using hpp::model::Device;
  using hpp::model::Joint;
  using hpp::model::Body;
  typedef hpp::model::ObjectVector_t ObjectVector_t;
  typedef hpp::model::JointVector_t JointVector_t;
  typedef std::vector<double> vector_t;



  /// **********  Pinocchio  ********** /// 
  /// ********************************* /// 

  // Building the model in pinocchio and compute kinematics/geometry for configuration q_pino
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::vector < std::string > package_dirs;
  std::string meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  package_dirs.push_back(meshDir);

  se3::Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(),model);
  se3::GeometryModel geom = se3::urdf::buildGeom(model, filename, package_dirs, se3::COLLISION);
  std::cout << model << std::endl;


  Data data(model);
  GeometryData data_geom(geom);

  // Configuration to be tested
  

  Eigen::VectorXd q_pino(Eigen::VectorXd::Random(model.nq));
  q_pino.segment<4>(3) /= q_pino.segment<4>(3).norm();

  Eigen::VectorXd q_hpp(q_pino);
  Eigen::Vector4d quaternion;
  quaternion <<  q_pino[6], q_pino[3], q_pino[4], q_pino[5];
  q_hpp.segment<4>(3) = quaternion ;

  BOOST_CHECK_MESSAGE(q_pino.size() == model.nq , "wrong config size");

  se3::updateGeometryPlacements(model, data, geom, data_geom, q_pino);


  /// *************  HPP  ************* /// 
  /// ********************************* ///


  hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("romeo");
  loadHumanoidPathPlanerModel(humanoidRobot, "freeflyer",
              "romeo_pinocchio", "romeo",
              "");

  BOOST_CHECK_MESSAGE(model.nq == humanoidRobot->configSize () , "Pinocchio model & HPP model config sizes are not the same ");

  humanoidRobot->currentConfiguration (q_hpp);
  humanoidRobot->computeForwardKinematics ();


  humanoidRobot->computeDistances ();

  /// **********  COMPARISON  ********* /// 
  /// ********************************* ///

  const hpp::model::DistanceResults_t& distances(humanoidRobot->distanceResults ());
  std::vector<std::string> bodies = getBodiesList();

  for (std::vector<std::string>::iterator i = bodies.begin(); i != bodies.end(); ++i)
  {
    for (std::vector<std::string>::iterator j = bodies.begin(); j != bodies.end(); ++j)
    {
      std::string body1 (*i); std::string inner1(body1 + "_0");
      std::string body2 (*j); std::string inner2(body2 + "_0");
      hpp::model::DistanceResults_t::const_iterator it = std::find_if ( distances.begin(),
                                                                    distances.end(),
                                                                    IsDistanceResultPair(inner1, inner2)
                                                                    );
      if(it != distances .end())
      {
        Distance_t distance_hpp(it->fcl);


        std::cout << "comparison between " << body1 << " and " << body2 << std::endl;

        se3::DistanceResult dist_pin
          = data_geom.computeDistance( CollisionPair(geom.getGeometryId(body1),
                                                     geom.getGeometryId(body2)) );

        Distance_t distance_pin(dist_pin.fcl_distance_result);
        distance_hpp.checkClose(distance_pin);
      }
    }
  }

}
#endif
BOOST_AUTO_TEST_SUITE_END ()

JointPositionsMap_t fillPinocchioJointPositions(const se3::Model& model, const se3::Data & data)
{
  JointPositionsMap_t result;
  for (se3::Model::Index i = 0; i < (se3::Model::Index)model.njoint; ++i)
  {
    result[model.getJointName(i)] = data.oMi[i];
  }
  return result;
}

GeometryPositionsMap_t fillPinocchioGeometryPositions(const se3::GeometryData & data_geom)
{
  GeometryPositionsMap_t result;
  for (std::size_t i = 0; i < data_geom.model_geom.ngeoms ; ++i)
  {
    result[data_geom.model_geom.getGeometryName(i)] = data_geom.oMg[i];
  }
  return result;
}

#ifdef WITH_HPP_MODEL_URDF
JointPositionsMap_t fillHppJointPositions(const hpp::model::HumanoidRobotPtr_t robot)
{
  JointPositionsMap_t result;
  const hpp::model::JointVector_t& joints(robot->getJointVector ());
  
  for (hpp::model::JointVector_t::const_iterator it = joints.begin (); it != joints.end (); ++it)
  {
    if((*it)->configSize() != 0) // Retrieving joints that are not anchors
    {
      result[(*it)->name()] = se3::toPinocchioSE3((*it)->currentTransformation() * (*it)->linkInJointFrame());
    }
  }
  return result;  

}

GeometryPositionsMap_t fillHppGeometryPositions(const hpp::model::HumanoidRobotPtr_t robot)
{
  GeometryPositionsMap_t result;

  const hpp::model::JointVector_t& joints(robot->getJointVector ());
  // retrieving only the positions of link that have collision geometry
  for (hpp::model::JointVector_t::const_iterator it = joints.begin (); it != joints.end (); ++it)
  {
    if((*it)->configSize() != 0) // Retrieving body attached to joints that are not anchors
    {
      hpp::model::BodyPtr_t body = (*it)->linkedBody ();
      if(body)
      {
        const hpp::model::ObjectVector_t& ov (body->innerObjects (hpp::model::COLLISION));
        for (hpp::model::ObjectVector_t::const_iterator itObj = ov.begin ();itObj != ov.end (); itObj ++)
        {
          result[body->name()] = se3::toPinocchioSE3((*itObj)->fcl ()->getTransform ());
        }
      }
    }
  }

  return result;
}

#endif

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
