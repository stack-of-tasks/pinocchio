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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main()
{
  using namespace Eigen;
  using namespace se3;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const unsigned int NBT = 1000*100;
  const unsigned int NBD = 1000; // for heavy tests, like computeDistances()
  #else
    const unsigned int NBT = 1;
    const unsigned int NBD = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
  
  std::string romeo_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo.urdf";
  std::vector < std::string > package_dirs;
  std::string romeo_meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  package_dirs.push_back(romeo_meshDir);

  se3::Model model;
  se3::urdf::buildModel(romeo_filename, se3::JointModelFreeFlyer(),model);
  se3::GeometryModel geom_model; se3::urdf::buildGeom(model, romeo_filename, COLLISION, geom_model, package_dirs);
#ifdef WITH_HPP_FCL  
  geom_model.addAllCollisionPairs();
#endif // WITH_HPP_FCL
   
  Data data(model);
  GeometryData geom_data(geom_model);


  std::vector<VectorXd> qs_romeo     (NBT);
  std::vector<VectorXd> qdots_romeo  (NBT);
  std::vector<VectorXd> qddots_romeo (NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo[i]     = Eigen::VectorXd::Random(model.nq);
    qs_romeo[i].segment<4>(3) /= qs_romeo[i].segment<4>(3).norm();
    qdots_romeo[i]  = Eigen::VectorXd::Random(model.nv);
    qddots_romeo[i] = Eigen::VectorXd::Random(model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model,data,qs_romeo[_smooth]);
  }
  double geom_time = timer.toc(PinocchioTicToc::US)/NBT;

  timer.tic();
  SMOOTH(NBT)
  {
    updateGeometryPlacements(model,data,geom_model,geom_data,qs_romeo[_smooth]);
  }
  double update_col_time = timer.toc(PinocchioTicToc::US)/NBT - geom_time;
  std::cout << "Update Collision Geometry < false > = \t" << update_col_time << " " << PinocchioTicToc::unitName(PinocchioTicToc::US) << std::endl;

#ifdef WITH_HPP_FCL
  timer.tic();
  SMOOTH(NBT)
  {
    updateGeometryPlacements(model,data,geom_model,geom_data,qs_romeo[_smooth]);
    for (std::vector<se3::CollisionPair>::iterator it = geom_model.collisionPairs.begin(); it != geom_model.collisionPairs.end(); ++it)
    {
      computeCollision(geom_model,geom_data,std::size_t(it-geom_model.collisionPairs.begin()));
    }
  }
  double collideTime = timer.toc(PinocchioTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision test between two geometry objects (mean time) = \t" << collideTime / double(geom_model.collisionPairs.size())
            << PinocchioTicToc::unitName(PinocchioTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    computeCollisions(geom_model,geom_data, true);
  }
  double is_colliding_time = timer.toc(PinocchioTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision Test : robot in collision? = \t" << is_colliding_time
            << PinocchioTicToc::unitName(PinocchioTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBD)
  {
    computeDistances(model,data,geom_model,geom_data,qs_romeo[_smooth]);
  }
  double computeDistancesTime = timer.toc(PinocchioTicToc::US)/NBD - (update_col_time + geom_time);
  std::cout << "Compute distance between two geometry objects (mean time) = \t" << computeDistancesTime / double(geom_model.collisionPairs.size())
            << " " << PinocchioTicToc::unitName(PinocchioTicToc::US) << " " << geom_model.collisionPairs.size() << " col pairs" << std::endl;

#endif // WITH_HPP_FCL
  return 0;
}
