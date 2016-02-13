//
// Copyright (c) 2015 - 2016 CNRS
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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/simulation/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/collisions.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"


#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/parser/urdf-with-geometry.hpp"
#ifdef WITH_HPP_MODEL_URDF
  #include <hpp/util/debug.hh>
  #include <hpp/model/device.hh>
  #include <hpp/model/body.hh>
  #include <hpp/model/collision-object.hh>
  #include <hpp/model/joint.hh>
  #include <hpp/model/urdf/util.hh>
#endif

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main()
{
  using namespace Eigen;
  using namespace se3;

  StackTicToc timer(StackTicToc::US);
  #ifdef NDEBUG
  const unsigned int NBT = 1000*100;
  const unsigned int NBD = 1000; // for heavy tests, like computeDistances()
  #else
    const unsigned int NBT = 1;
    const unsigned int NBD = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    


  std::string romeo_filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::string romeo_meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  std::pair < Model, GeometryModel > romeo = se3::urdf::buildModelAndGeom(romeo_filename, romeo_meshDir, se3::JointModelFreeFlyer());
  se3::Model romeo_model = romeo.first;
  se3::GeometryModel romeo_model_geom = romeo.second;
  Data romeo_data(romeo_model);
  GeometryData romeo_data_geom(romeo_data, romeo_model_geom);


  std::vector<VectorXd> qs_romeo     (NBT);
  std::vector<VectorXd> qdots_romeo  (NBT);
  std::vector<VectorXd> qddots_romeo (NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo[i]     = Eigen::VectorXd::Random(romeo_model.nq);
    qs_romeo[i].segment<4>(3) /= qs_romeo[i].segment<4>(3).norm();
    qdots_romeo[i]  = Eigen::VectorXd::Random(romeo_model.nv);
    qddots_romeo[i] = Eigen::VectorXd::Random(romeo_model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(romeo_model,romeo_data,qs_romeo[_smooth]);
  }
  double geom_time = timer.toc(StackTicToc::US)/NBT;

  timer.tic();
  SMOOTH(NBT)
  {
    updateGeometryPlacements(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth]);
  }
  double update_col_time = timer.toc(StackTicToc::US)/NBT - geom_time;
  std::cout << "Update Collision Geometry < false > = \t" << update_col_time << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    updateGeometryPlacements(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth]);
    for (std::vector<se3::GeometryData::CollisionPair_t>::iterator it = romeo_data_geom.collision_pairs.begin(); it != romeo_data_geom.collision_pairs.end(); ++it)
    {
      romeo_data_geom.computeCollision(it->first, it->second);
    }
  }
  double collideTime = timer.toc(StackTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision test between two geometry objects (mean time) = \t" << collideTime / romeo_data_geom.nCollisionPairs
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    computeCollisions(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth], true);
  }
  double is_colliding_time = timer.toc(StackTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision Test : robot in collision? = \t" << is_colliding_time
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    computeDistances(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth]);
  }
  double computeDistancesTime = timer.toc(StackTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Compute distance between two geometry objects (mean time) = \t" << computeDistancesTime / romeo_data_geom.nCollisionPairs
            << " " << StackTicToc::unitName(StackTicToc::US) << " " << romeo_data_geom.nCollisionPairs << " col pairs" << std::endl;



#ifdef WITH_HPP_MODEL_URDF



  std::vector<VectorXd> qs_romeo_pino     (NBT); 
  std::vector<VectorXd> qdots_romeo_pino  (NBT); 
  std::vector<VectorXd> qddots_romeo_pino (NBT); 
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo_pino[i]     = Eigen::VectorXd::Random(romeo_model.nq);
    qs_romeo_pino[i].segment<4>(3) /= qs_romeo_pino[i].segment<4>(3).norm();
    qdots_romeo_pino[i]  = Eigen::VectorXd::Random(romeo_model.nv);
    qddots_romeo_pino[i] = Eigen::VectorXd::Random(romeo_model.nv);
  }
  std::vector<VectorXd> qs_romeo_hpp     (qs_romeo_pino);
  std::vector<VectorXd> qdots_romeo_hpp  (qdots_romeo_pino);
  std::vector<VectorXd> qddots_romeo_hpp (qddots_romeo_pino);

  for (size_t i = 0; i < NBT; ++i)
  {
    Vector4d quaternion;
    quaternion <<  qs_romeo_pino[i][6], qs_romeo_pino[i][3], qs_romeo_pino[i][4], qs_romeo_pino[i][5];
    qs_romeo_hpp[i].segment<4>(3) = quaternion ;
  }




//   /// *************  HPP  ************* /// 
//   /// ********************************* ///


hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("romeo");
  hpp::model::urdf::loadHumanoidModel(humanoidRobot, "freeflyer",
              "romeo_pinocchio", "romeo",
              "", "");



  timer.tic();
  SMOOTH(NBT)
  {
    updateCollisionGeometry(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo_pino[_smooth], true);
  }
  double compute_forward_kinematics_time = timer.toc(StackTicToc::US)/NBT;
  std::cout << "Update Collision Geometry < true > (K) = \t" << compute_forward_kinematics_time << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    humanoidRobot->currentConfiguration (qs_romeo_hpp[_smooth]);
    humanoidRobot->computeForwardKinematics ();
  }
  double compute_forward_kinematics_time_hpp = timer.toc(StackTicToc::US)/NBT;
  std::cout << "HPP - Compute Forward Kinematics (K) = \t" << compute_forward_kinematics_time_hpp
            << StackTicToc::unitName(StackTicToc::US) << std::endl;
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeCollisions(romeo_data_geom, true);
  }
  double is_romeo_colliding_time_pino = timer.toc(StackTicToc::US)/NBT;
  std::cout << "Pinocchio - Collision Test : robot in collision? (G) = \t" << is_romeo_colliding_time_pino
            << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    humanoidRobot->collisionTest();
  }
  double is_romeo_colliding_time_hpp = timer.toc(StackTicToc::US)/NBT;
  std::cout << "HPP - Collision Test : robot in collision? (G)= \t" << is_romeo_colliding_time_hpp
            << StackTicToc::unitName(StackTicToc::US) << std::endl;
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeCollisions(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo_pino[_smooth], true);
  }
  is_romeo_colliding_time_pino = timer.toc(StackTicToc::US)/NBT;
  std::cout << "Pinocchio - Collision Test : update + robot in collision? (K+G)= \t" << is_romeo_colliding_time_pino
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    humanoidRobot->currentConfiguration (qs_romeo_hpp[_smooth]);
    humanoidRobot->computeForwardKinematics ();
    humanoidRobot->collisionTest();
  }
  is_romeo_colliding_time_hpp = timer.toc(StackTicToc::US)/NBT;
  std::cout << "HPP - Collision Test : update + robot in collision? (K+G) = \t" << is_romeo_colliding_time_hpp
            << StackTicToc::unitName(StackTicToc::US) << std::endl;



  timer.tic();
  SMOOTH(NBT)
  {
    computeDistances(romeo_data_geom);
  }
  computeDistancesTime = timer.toc(StackTicToc::US)/NBT ;
  std::cout << "Pinocchio - Compute distances (D) " << romeo_data_geom.nCollisionPairs << " col pairs\t" << computeDistancesTime 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    humanoidRobot->computeDistances ();
  }
  double hpp_compute_distances = timer.toc(StackTicToc::US)/NBT ;
  std::cout << "HPP - Compute distances (D) " << humanoidRobot->distanceResults().size() << " col pairs\t" << hpp_compute_distances 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeDistances(romeo_model, romeo_data, romeo_model_geom, romeo_data_geom, qs_romeo_pino[_smooth]);
  }
  computeDistancesTime = timer.toc(StackTicToc::US)/NBT ;
  std::cout << "Pinocchio - Update + Compute distances (K+D) " << romeo_data_geom.nCollisionPairs << " col pairs\t" << computeDistancesTime 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    humanoidRobot->currentConfiguration (qs_romeo_hpp[_smooth]);
    humanoidRobot->computeForwardKinematics ();
    humanoidRobot->computeDistances ();
  }
  hpp_compute_distances = timer.toc(StackTicToc::US)/NBT ;
  std::cout << "HPP - Update + Compute distances (K+D) " << humanoidRobot->distanceResults().size() << " col pairs\t" << hpp_compute_distances 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

#endif

  return 0;
}
