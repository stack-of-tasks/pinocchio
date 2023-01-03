//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/algorithm/parallel/rnea.hpp"
#include "pinocchio/algorithm/parallel/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include "pinocchio/algorithm/parallel/geometry.hpp"
#endif

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"


int main(int /*argc*/, const char ** /*argv*/)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
//  typedef Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> MatrixXb;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const int NBT = 4000;
 
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
  
  const int BATCH_SIZE = 256;
  const int NUM_THREADS = omp_get_max_threads();
    
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "name = " << model.name << std::endl;
  
#ifdef PINOCCHIO_WITH_HPP_FCL
  const std::string package_path = PINOCCHIO_MODEL_DIR;
  hpp::fcl::MeshLoaderPtr mesh_loader = make_shared<hpp::fcl::CachedMeshLoader>();
  const std::string srdf_filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  std::vector<std::string> package_paths(1,package_path);
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(model,filename,COLLISION,geometry_model,package_paths,mesh_loader);
  
  geometry_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model,geometry_model,srdf_filename,false);
  
  GeometryData geometry_data(geometry_model);
  {
    int num_active_collision_pairs = 0;
    for(size_t k = 0; k < geometry_data.activeCollisionPairs.size(); ++k)
    {
      if(geometry_data.activeCollisionPairs[k])
        num_active_collision_pairs++;
    }
    std::cout << "active collision pairs = " << num_active_collision_pairs << std::endl;
  }
#endif
  
  std::cout << "--" << std::endl;
  std::cout << "NUM_THREADS: " << NUM_THREADS << std::endl;
  std::cout << "BATCH_SIZE: " << BATCH_SIZE << std::endl;
  std::cout << "--" << std::endl;
  
  pinocchio::Data data(model);
  const VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  MatrixXd qs(model.nq,BATCH_SIZE);
  MatrixXd vs(model.nv,BATCH_SIZE);
  MatrixXd as(model.nv,BATCH_SIZE);
  MatrixXd taus(model.nv,BATCH_SIZE);
  MatrixXd res(model.nv,BATCH_SIZE);
  
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) q_vec(NBT);
  for(size_t i=0; i < NBT; ++i)
  {
    q_vec[i] = randomConfiguration(model,-qmax,qmax);
  }
  
  for(Eigen::DenseIndex i=0; i < BATCH_SIZE; ++i)
  {
    qs.col(i) = randomConfiguration(model,-qmax,qmax);
    vs.col(i) = Eigen::VectorXd::Random(model.nv);
    as.col(i) = Eigen::VectorXd::Random(model.nv);
    taus.col(i) = Eigen::VectorXd::Random(model.nv);
  }
  
  ModelPool pool(model,NUM_THREADS);
  
  timer.tic();
  SMOOTH(NBT)
  {
    for(Eigen::DenseIndex i = 0; i < BATCH_SIZE; ++i)
      res.col(i) = rnea(model,data,qs.col(i),vs.col(i),as.col(i));
  }
  std::cout << "mean RNEA = \t\t\t\t"; timer.toc(std::cout,NBT*BATCH_SIZE);

  for(int num_threads = 1; num_threads <= NUM_THREADS; ++num_threads)
  {
    timer.tic();
    SMOOTH(NBT)
    {
      rnea(num_threads,pool,qs,vs,as,res);
    }
    double elapsed = timer.toc()/(NBT*BATCH_SIZE);
    std::stringstream ss;
    ss << "mean RNEA pool (";
    ss << num_threads;
    ss << " threads) = \t\t";
    ss << elapsed << " us" << std::endl;
    std::cout << ss.str();
  }
  
  std::cout << "--" << std::endl;
  
  timer.tic();
  SMOOTH(NBT)
  {
    for(Eigen::DenseIndex i = 0; i < BATCH_SIZE; ++i)
      res.col(i) = aba(model,data,qs.col(i),vs.col(i),taus.col(i));
  }
  std::cout << "mean ABA = \t\t\t\t"; timer.toc(std::cout,NBT*BATCH_SIZE);

  for(int num_threads = 1; num_threads <= NUM_THREADS; ++num_threads)
  {
    timer.tic();
    SMOOTH(NBT)
    {
      aba(num_threads,pool,qs,vs,taus,res);
    }
    double elapsed = timer.toc()/(NBT*BATCH_SIZE);
    std::stringstream ss;
    ss << "mean ABA pool (";
    ss << num_threads;
    ss << " threads) = \t\t";
    ss << elapsed << " us" << std::endl;
    std::cout << ss.str();
  }
  
#ifdef PINOCCHIO_WITH_HPP_FCL
  std::cout << "--" << std::endl;
  const int NBT_COLLISION = math::max(NBT,1);
  timer.tic();
  SMOOTH((size_t)NBT_COLLISION)
  {
    computeCollisions(model,data,geometry_model,geometry_data,q_vec[_smooth]);
  }
  std::cout << "non parallel collision = \t\t\t"; timer.toc(std::cout,NBT_COLLISION);

  for(int num_threads = 1; num_threads <= NUM_THREADS; ++num_threads)
  {
    timer.tic();
    SMOOTH((size_t)NBT_COLLISION)
    {
      computeCollisions(num_threads,model,data,geometry_model,geometry_data,q_vec[_smooth]);
    }
    double elapsed = timer.toc()/(NBT_COLLISION);
    std::stringstream ss;
    ss << "parallel collision (";
    ss << num_threads;
    ss << " threads) = \t\t";
    ss << elapsed << " us" << std::endl;
    std::cout << ss.str();
  }
  
  std::cout << "--" << std::endl;
  GeometryPool geometry_pool(model,geometry_model,NUM_THREADS);
  VectorXb collision_res(BATCH_SIZE);
  collision_res.fill(false);
  
  timer.tic();
  SMOOTH((size_t)(NBT_COLLISION/BATCH_SIZE))
  {
    for(Eigen::DenseIndex i = 0; i < BATCH_SIZE; ++i)
      computeCollisions(model,data,geometry_model,geometry_data,qs.col(i));
  }
  std::cout << "non parallel collision = \t\t\t"; timer.toc(std::cout,NBT_COLLISION);

  for(int num_threads = 1; num_threads <= NUM_THREADS; ++num_threads)
  {
    timer.tic();
    SMOOTH((size_t)(NBT_COLLISION/BATCH_SIZE))
    {
      computeCollisions(num_threads,geometry_pool,qs,collision_res);
    }
    double elapsed = timer.toc()/(NBT_COLLISION);
    std::stringstream ss;
    ss << "pool parallel collision (";
    ss << num_threads;
    ss << " threads) = \t\t";
    ss << elapsed << " us" << std::endl;
    std::cout << ss.str();
  }
#endif

  std::cout << "--" << std::endl;
  return 0;
}
