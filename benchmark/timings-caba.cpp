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

int run_solvers(pinocchio::Model model, PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models, 
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_datas, int max_iter)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
    const int NBT = 1000*100;
  #else
    const int NBT = 1;
  #endif

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

  timer.tic();
  SMOOTH(NBT)
  {
    minimal::aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA = \t\t"; timer.toc(std::cout,NBT);

  double mu = 1e-5;

  ProximalSettings prox_settings;
  prox_settings.max_iter = max_iter;
  prox_settings.mu = mu;
  prox_settings.absolute_accuracy = 1e-10;

  initConstraintDynamics(model,data,contact_models);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings);
  }
  std::cout << "constraintDynamics {6D,6D} = \t\t"; timer.toc(std::cout,NBT);
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas,prox_settings);
  }
  std::cout << "contact ABA {6D,6D} = \t\t"; timer.toc(std::cout,NBT);


  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.use_early = false;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings, pv_settings);
  }
  std::cout << "proxPV {6D,6D} = \t\t"; timer.toc(std::cout,NBT);

  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.use_early = true;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings, pv_settings);
  }
  std::cout << "cABA {6D,6D} = \t\t"; timer.toc(std::cout,NBT);


  std::cout << "--" << std::endl;

  return 0;
}

int benchmark_contacts(std::string filename, std::vector<std::string> link_names, std::vector<size_t> contact_links, std::vector<int> contact_types, bool with_ff = true, int max_iter = 3)
{

  using namespace Eigen;
  using namespace pinocchio;

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;

  Model model;

  if(with_ff)
    pinocchio::urdf::buildModel(PINOCCHIO_MODEL_DIR + filename,JointModelFreeFlyer(),model);
  else
    pinocchio::urdf::buildModel(PINOCCHIO_MODEL_DIR + filename,model);

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

  run_solvers(model, contact_models, contact_datas, max_iter);

  return 0;

}

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
#ifndef NDEBUG
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

  std::string simple_humanoid = "simple_humanoid.urdf";
  std::vector<std::string> simple_humanoid_links = {"RLEG_LINK6", "LLEG_LINK6", "RARM_LINK6", "LARM_LINK6"};

  std::string talos_filename = "/example-robot-data/robots/talos_data/robots/talos_full_v2.urdf";
  std::vector<std::string> talos_links = {"leg_right_6_link", "leg_left_6_link", "gripper_right_fingertip_3_link", "gripper_left_fingertip_3_link", "leg_right_5_link", "leg_left_5_link"};

  std::string solo_filename = "/example-robot-data/robots/solo_description/robots/solo12.urdf";
  std::vector<std::string> solo_links = {"FL_LOWER_LEG", "FR_LOWER_LEG", "HL_LOWER_LEG", "HR_LOWER_LEG"};

  std::string iiwa_filename = "/iiwa2.urdf";
  std::vector<std::string> iiwa_links = {"iiwa_link_7"};
  
  // std::vector<int> link_names = {}
  // benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6});

  benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6}, true, 1);
  benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6}, true, 3);
  benchmark_contacts(talos_filename, talos_links, {0, 1, 2, 3}, {6, 6, 6, 6}, true, 1);
  benchmark_contacts(talos_filename, talos_links, {0, 1, 2, 3}, {6, 6, 6, 6}, true, 3);

  std::cout << "\n\n Solo \n\n";
  benchmark_contacts(solo_filename, solo_links, {0, 1}, {3,3}, true, 1);
  benchmark_contacts(solo_filename, solo_links, {0, 1}, {3,3}, true, 3);
  benchmark_contacts(solo_filename, solo_links, {0, 1, 2, 3}, {3,3,3,3}, true, 1);
  benchmark_contacts(solo_filename, solo_links, {0, 1, 2, 3}, {3,3,3,3}, true, 3);

  std::cout << "\n\n Iiwa \n\n";
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {3}, true, 1);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {3}, true, 3);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {6}, true, 1);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {6}, true, 3);
  
  // const std::string RA = "gripper_right_fingertip_3_link";
  // const JointIndex RA_id = model.frames[model.getFrameId(RA)].parentJoint;
  // const std::string LA = "gripper_left_fingertip_3_link";
  // const JointIndex LA_id = model.frames[model.getFrameId(LA)].parentJoint;
  // const std::string RF = "leg_right_6_link";
  // const JointIndex RF_id = model.frames[model.getFrameId(RF)].parentJoint;
  // const std::string LF = "leg_left_6_link";
  // const JointIndex LF_id = model.frames[model.getFrameId(LF)].parentJoint;

  // RigidConstraintModel ci_RF_6D(CONTACT_6D,model,RF_id,LOCAL);
  // RigidConstraintModel ci_RF_3D(CONTACT_3D,model,RF_id,LOCAL);

  // RigidConstraintModel ci_LF_6D(CONTACT_6D,model,LF_id,LOCAL);
  // RigidConstraintModel ci_LF_3D(CONTACT_3D,model,LF_id,LOCAL);

  // RigidConstraintModel ci_RA_3D(CONTACT_3D,model,RA_id,LOCAL);
  // RigidConstraintModel ci_LA_3D(CONTACT_3D,model,LA_id,LOCAL);

  // RigidConstraintModel ci_RA_6D(CONTACT_6D,model,RA_id,LOCAL);
  // RigidConstraintModel ci_LA_6D(CONTACT_6D,model,LA_id,LOCAL);

  // // Define contact infos structure
  // static const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  // static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_empty;
  // cholesky::ContactCholeskyDecomposition contact_chol_empty(model,contact_models_empty);

  // PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
  // contact_models_6D.push_back(ci_RF_6D);
  // PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D;
  // contact_data_6D.push_back(RigidConstraintData(ci_RF_6D));

  // PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
  // PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D6D;

  // int num_cons;
  // std::cout << "Enter number of contacts: ";
  // std::cin >> num_cons;

  // if (num_cons > 0)
  // { 
  //   contact_models_6D6D.push_back(ci_RF_6D);
  //   contact_data_6D6D.push_back(RigidConstraintData(ci_RF_6D));
  // }
  // if (num_cons > 1)
  // {
  //   contact_models_6D6D.push_back(ci_LF_6D);
  //   contact_data_6D6D.push_back(RigidConstraintData(ci_LF_6D));
  // }
  // if (num_cons > 2)
  // {
  //   contact_models_6D6D.push_back(ci_RA_6D);
  //   contact_data_6D6D.push_back(RigidConstraintData(ci_RA_6D));
  // }
  // if (num_cons > 3)
  // {
  //   contact_models_6D6D.push_back(ci_LA_6D);
  //   contact_data_6D6D.push_back(RigidConstraintData(ci_LA_6D));
  // }

  // const double mu = 1e-7;
  // int max_iter = 2;
  // std::cout << "Enter max_iter: ";
  // std::cin >> max_iter;

  // std::cout << "nq = " << model.nq << std::endl;
  // std::cout << "nv = " << model.nv << std::endl;
  // std::cout << "--" << std::endl;

  // Data data(model);
  // VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  // run_solvers(model, contact_models_6D6D, contact_data_6D6D, 5);


  return 0;
}

