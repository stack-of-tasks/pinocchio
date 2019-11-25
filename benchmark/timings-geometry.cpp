//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
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
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const unsigned int NBT = 1000*100;
  const unsigned int NBD = 1000; // for heavy tests, like computeDistances()
  #else
    const unsigned int NBT = 1;
    const unsigned int NBD = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
  
  std::string romeo_filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector < std::string > package_dirs;
  std::string romeo_meshDir  = PINOCCHIO_MODEL_DIR + std::string("/others/robots");
  package_dirs.push_back(romeo_meshDir);

  pinocchio::Model model;
  pinocchio::urdf::buildModel(romeo_filename, pinocchio::JointModelFreeFlyer(),model);
  pinocchio::GeometryModel geom_model; pinocchio::urdf::buildGeom(model, romeo_filename, COLLISION, geom_model, package_dirs);
#ifdef PINOCCHIO_WITH_HPP_FCL  
  geom_model.addAllCollisionPairs();
#endif // PINOCCHIO_WITH_HPP_FCL
   
  Data data(model);
  GeometryData geom_data(geom_model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);


  std::vector<VectorXd> qs_romeo     (NBT);
  std::vector<VectorXd> qdots_romeo  (NBT);
  std::vector<VectorXd> qddots_romeo (NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo[i]     = randomConfiguration(model,-qmax,qmax);
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

#ifdef PINOCCHIO_WITH_HPP_FCL
  timer.tic();
  SMOOTH(NBT)
  {
    updateGeometryPlacements(model,data,geom_model,geom_data,qs_romeo[_smooth]);
    for (std::vector<pinocchio::CollisionPair>::iterator it = geom_model.collisionPairs.begin(); it != geom_model.collisionPairs.end(); ++it)
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

#endif // PINOCCHIO_WITH_HPP_FCL
  
  return 0;
}
