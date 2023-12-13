//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/pv_solver.hpp"
#include <iostream>

#include "pinocchio/utils/timer.hpp"

int main()
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
    const int NBT = 1000*100;
  #else
    const int NBT = 1;
  #endif

  // Build model
  Model model;

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;

  std::string filename = "/simple_humanoid.urdf";
  std::vector<std::string> link_names  = {"RLEG_LINK6", "LLEG_LINK6", "RARM_LINK6", "LARM_LINK6"};

  std::vector<int> contact_types = {6,6};
  std::vector<int> contact_links = {0,1};

  pinocchio::urdf::buildModel(PINOCCHIO_MODEL_DIR + filename,JointModelFreeFlyer(),model);



  for (size_t i = 0; i < contact_links.size(); i++)
  {
    const JointIndex joint_id = model.frames[model.getFrameId(link_names[contact_links[i]])].parentJoint;

    assert (contact_types[i] == 6 || contact_types[i] == 3 && "only 3 or 6 contacts allowed");

    RigidConstraintModel ci;

    if (contact_types[i] == 6)
      ci = RigidConstraintModel(CONTACT_6D,model,joint_id,LOCAL);
    else
      ci = RigidConstraintModel(CONTACT_3D,model,joint_id,LOCAL);

    contact_models.push_back(ci);
    contact_datas.push_back(RigidConstraintData(ci));
  }

      

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qs(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qdots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qddots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) taus(NBT);

  Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  for(size_t i=0;i<NBT;++i)
  {
    qs[i]     = randomConfiguration(model,-qmax,qmax);
    qdots[i]  = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }



  double mu = 1e-6;
  int max_iter = 3;

  ProximalSettings prox_settings;
  prox_settings.max_iter = max_iter;
  prox_settings.mu = mu;
  prox_settings.absolute_accuracy = 1e-10;

 
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas,prox_settings);
  }
  std::cout << timer.toc()/NBT << ", ";  

  return 0;
}


