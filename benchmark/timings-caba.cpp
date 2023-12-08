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

  // Build model
  Model model;

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
    buildModels::humanoidRandom(model,true);
  else
    if(with_ff)
      pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  //      pinocchio::urdf::buildModel(filename,JointModelRX(),model);
    else
      pinocchio::urdf::buildModel(filename,model);

  const std::string RA = "gripper_right_fingertip_3_link";
  const JointIndex RA_id = model.frames[model.getFrameId(RA)].parent;
  const std::string LA = "gripper_left_fingertip_3_link";
  const JointIndex LA_id = model.frames[model.getFrameId(LA)].parent;
  const std::string RF = "leg_right_6_link";
  const JointIndex RF_id = model.frames[model.getFrameId(RF)].parent;
  const std::string LF = "leg_left_6_link";
  const JointIndex LF_id = model.frames[model.getFrameId(LF)].parent;

  RigidConstraintModel ci_RF_6D(CONTACT_6D,model,RF_id,LOCAL);
  RigidConstraintModel ci_RF_3D(CONTACT_3D,model,RF_id,LOCAL);

  RigidConstraintModel ci_LF_6D(CONTACT_6D,model,LF_id,LOCAL);
  RigidConstraintModel ci_LF_3D(CONTACT_3D,model,LF_id,LOCAL);

  RigidConstraintModel ci_RA_3D(CONTACT_3D,model,RA_id,LOCAL);
  RigidConstraintModel ci_LA_3D(CONTACT_3D,model,LA_id,LOCAL);

  RigidConstraintModel ci_RA_6D(CONTACT_6D,model,RA_id,LOCAL);
  RigidConstraintModel ci_LA_6D(CONTACT_6D,model,LA_id,LOCAL);

  // Define contact infos structure
  static const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_empty;
  cholesky::ContactCholeskyDecomposition contact_chol_empty(model,contact_models_empty);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
  contact_models_6D.push_back(ci_RF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D;
  contact_data_6D.push_back(RigidConstraintData(ci_RF_6D));

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D6D;

  int num_cons;
  std::cout << "Enter number of contacts: ";
  std::cin >> num_cons;

  if (num_cons > 0)
  { 
    contact_models_6D6D.push_back(ci_RF_6D);
    contact_data_6D6D.push_back(RigidConstraintData(ci_RF_6D));
  }
  if (num_cons > 1)
  {
    contact_models_6D6D.push_back(ci_LF_6D);
    contact_data_6D6D.push_back(RigidConstraintData(ci_LF_6D));
  }
  if (num_cons > 2)
  {
    contact_models_6D6D.push_back(ci_RA_6D);
    contact_data_6D6D.push_back(RigidConstraintData(ci_RA_6D));
  }
  if (num_cons > 3)
  {
    contact_models_6D6D.push_back(ci_LA_6D);
    contact_data_6D6D.push_back(RigidConstraintData(ci_LA_6D));
  }

  const double mu = 1e-7;
  int max_iter = 2;
  std::cout << "Enter max_iter: ";
  std::cin >> max_iter;

  ProximalSettings prox_settings;
  prox_settings.max_iter = max_iter;
  prox_settings.mu = mu;
  prox_settings.absolute_accuracy = 1e-10;

  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;

  Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qs(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qdots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qddots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) taus(NBT);

  static const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;

  for(size_t i=0;i<NBT;++i)
  {
    qs[i]     = randomConfiguration(model,-qmax,qmax);
    qdots[i]  = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    minimal::aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA = \t\t"; timer.toc(std::cout,NBT);


  initConstraintDynamics(model,data,contact_models_6D6D);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models_6D6D,contact_data_6D6D, prox_settings);
  }
  std::cout << "constraintDynamics {6D,6D} = \t\t"; timer.toc(std::cout,NBT);
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models_6D6D,contact_data_6D6D,prox_settings);
  }
  std::cout << "contact ABA {6D,6D} = \t\t"; timer.toc(std::cout,NBT);


  initPvSolver(model,data,contact_models_6D6D);
  Data::PvSettings pv_settings;
  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.use_early = false;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models_6D6D,contact_data_6D6D, prox_settings, pv_settings);
  }
  std::cout << "proxPV {6D,6D} = \t\t"; timer.toc(std::cout,NBT);

  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.use_early = true;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models_6D6D,contact_data_6D6D, prox_settings, pv_settings);
  }
  std::cout << "cABA {6D,6D} = \t\t"; timer.toc(std::cout,NBT);


  std::cout << "--" << std::endl;

  return 0;
}

