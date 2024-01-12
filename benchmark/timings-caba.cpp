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
  std::cout << model.nv << ", " << timer.toc()/NBT << ", ";  


  double mu = 1e-1;

  ProximalSettings prox_settings;
  prox_settings.max_iter = max_iter;
  prox_settings.mu = mu;
  prox_settings.absolute_accuracy = 1e-10;
  prox_settings.relative_accuracy = 1e-18;

  initConstraintDynamics(model,data,contact_models);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings);
  }
  std::cout << timer.toc()/NBT << ", ";  

  timer.tic();
  SMOOTH(NBT)
  {
    proxLTLs(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models, contact_datas, prox_settings);
  }
  std::cout << timer.toc()/NBT << ", ";  
 
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas,prox_settings);
  }
  std::cout << timer.toc()/NBT << ", ";  


  initPvSolver(model,data,contact_models);
  Data::PvSettings pv_settings;
  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.absolute_accuracy = 1e-10;
  pv_settings.use_early = false;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings, pv_settings);
  }
  std::cout << timer.toc()/NBT << ", ";  

  pv_settings.mu = mu;
  pv_settings.max_iter = max_iter;
  pv_settings.use_early = true;
  timer.tic();
  SMOOTH(NBT)
  {
    pv(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_models,contact_datas, prox_settings, pv_settings);
  }
  std::cout << timer.toc()/NBT << "\n"; 

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

  std::string atlas_filename = "/atlas_with_srs.urdf";
  std::vector<std::string> atlas_links = {"l_foot", "r_foot", "sr_left_thumb_distal", "sr_left_index_finger_distal", "sr_left_middle_finger_distal", "sr_left_ring_finger_distal", "sr_left_little_finger_distal", "sr_right_thumb_distal", "sr_right_index_finger_distal", "sr_right_middle_finger_distal", "sr_right_ring_finger_distal", "sr_right_little_finger_distal"};
  
  std::vector<std::string> chain_names;
  std::vector<std::string> chain_links;

  // for (int i = 6; i < 101; i++)
  // {
  //   std::string chain_name = "/example-robot-data/robots/chain_urdf_files/chain" + std::to_string(i) + ".urdf";
  //   chain_names.push_back(chain_name);
  //   chain_links.push_back("link" + std::to_string(i));
  //   // benchmark_contacts(chain_name, chain_links, {0}, {6}, false, 1);
  //   // benchmark_contacts(chain_name, chain_links, {0}, {6}, false, 3);
  //   // benchmark_contacts(chain_name, chain_links, {0}, {6}, false, 10);
  //   chain_links.pop_back();
  // }

  // Benchmark the binary trees
  for (int i = 1; i < 35; i++)
  {
    std::string chain_name = "/example-robot-data/robots/binary_tree_urdfs/tree" + std::to_string(i*3) + ".urdf";
    std::vector<size_t> link_numbers;
    std::vector<int> link_types;
    size_t counter = 0;
    for (int j = i/2 + 1;  j <= i; j++)
    {
      chain_links.push_back("link" + std::to_string(j*3));
      link_numbers.push_back(counter);
      counter += 1;
      link_types.push_back(3);
    }
    // chain_links.push_back("link" + std::to_string(i));
    // benchmark_contacts(chain_name, chain_links, link_numbers, link_types, false, 1);
    benchmark_contacts(chain_name, chain_links, link_numbers, link_types, false, 3);
    // benchmark_contacts(chain_name, chain_links, {0}, {6}, false, 3);
    // benchmark_contacts(chain_name, chain_links, {0}, {6}, false, 10);
    // chain_links.pop_back();
    chain_links.clear();
  }
  // std::vector<int> link_names = {}
  // benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6});

  benchmark_contacts(atlas_filename, atlas_links, {0}, {6}, true, 1);
   benchmark_contacts(atlas_filename, atlas_links, {0}, {6}, true, 3);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1}, {6, 6}, true, 1);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1}, {6, 6}, true, 3);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1, 2, 3, 4, 5, 6}, {6, 6, 3, 3, 3, 3, 3}, true, 1);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1, 2, 3, 4, 5, 6}, {6, 6, 3, 3, 3, 3, 3}, true, 3);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, {6, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, true, 1);
  benchmark_contacts(atlas_filename, atlas_links, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, {6, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, true, 3);
  std::cout << "\n\n";

  benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6}, true, 1);
  benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6}, true, 3);
  benchmark_contacts(talos_filename, talos_links, {0, 1}, {6, 6}, true, 10);
  benchmark_contacts(talos_filename, talos_links, {0, 1, 2, 3}, {6, 6, 6, 6}, true, 1);
  benchmark_contacts(talos_filename, talos_links, {0, 1, 2, 3}, {6, 6, 6, 6}, true, 3);
  benchmark_contacts(talos_filename, talos_links, {0, 1, 2, 3}, {6, 6, 6, 6}, true, 10);

  std::cout << "\n\n Solo \n\n";
  benchmark_contacts(solo_filename, solo_links, {0, 1}, {3,3}, true, 1);
  benchmark_contacts(solo_filename, solo_links, {0, 1}, {3,3}, true, 3);
  benchmark_contacts(solo_filename, solo_links, {0, 1, 2, 3}, {3,3,3,3}, true, 1);
  benchmark_contacts(solo_filename, solo_links, {0, 1, 2, 3}, {3,3,3,3}, true, 3);


  std::cout << "\n\n Iiwa \n\n";
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {3}, false, 1);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {3}, false, 3);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {6}, false, 1);
  benchmark_contacts(iiwa_filename, iiwa_links, {0}, {6}, false, 3);



  return 0;
}

