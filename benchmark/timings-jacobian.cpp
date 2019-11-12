//
// Copyright (c) 2015-2018 CNRS
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const int NBT = 1000*100;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif

  pinocchio::Model model;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if(argc>1) filename = argv[1];

  bool with_ff = true;
  if(argc>2)
  {
    const std::string ff_option = argv[2];
    if(ff_option == "-no-ff")
      with_ff = false;
  }

  if( filename == "HS")
    pinocchio::buildModels::humanoidRandom(model,true);
  else
    if(with_ff)
      pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
    else
      pinocchio::urdf::buildModel(filename,model);
  std::cout << "nq = " << model.nq << std::endl;

  pinocchio::Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  //VectorXd q = randomConfiguration(model);

  pinocchio::Data::Matrix6x J(6,model.nv);
  J.setZero();
  pinocchio::Model::JointIndex JOINT_ID = (Model::JointIndex)(model.njoints-1);

  std::vector<VectorXd> qs(NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs[i] = randomConfiguration(model,-qmax,qmax);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model,data,qs[_smooth]);
  }
  std::cout << "Zero Order Kinematics = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeJointJacobian(model,data,qs[_smooth],JOINT_ID,J);
  }
  std::cout << "computeJointJacobian = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeJointJacobians(model,data,qs[_smooth]);
  }
  std::cout << "computeJointJacobians(q) = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeJointJacobians(model,data);
  }
  std::cout << "computeJointJacobians() = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    getJointJacobian(model,data,JOINT_ID,LOCAL,J);
  }
  std::cout << "getJointJacobian(LOCAL) = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    getJointJacobian(model,data,JOINT_ID,WORLD,J);
  }
  std::cout << "getJointJacobian(WORLD) = \t"; timer.toc(std::cout,NBT);

  std::cout << "--" << std::endl;
  return 0;
}
