//
// Copyright (c) 2019-2020 LAAS-CNRS INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
#ifdef NDEBUG
  const int NBT = 1000 * 100;
#else
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant) " << std::endl;
#endif

  // Build model
  Model model;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if (argc > 1)
    filename = argv[1];
  bool with_ff = true;

  if (argc > 2)
  {
    const std::string ff_option = argv[2];
    if (ff_option == "-no-ff")
      with_ff = false;
  }

  if (filename == "HS")
    buildModels::humanoidRandom(model, true);
  else if (with_ff)
    pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  //      pinocchio::urdf::buildModel(filename,JointModelRX(),model);
  else
    pinocchio::urdf::buildModel(filename, model);

  const std::string RA = "RARM_LINK6";
  const JointIndex RA_id = model.frames[model.getFrameId(RA)].parentJoint;
  const std::string LA = "LARM_LINK6";
  const JointIndex LA_id = model.frames[model.getFrameId(LA)].parentJoint;
  const std::string RF = "RLEG_LINK6";
  const JointIndex RF_id = model.frames[model.getFrameId(RF)].parentJoint;
  const std::string LF = "LLEG_LINK6";
  const JointIndex LF_id = model.frames[model.getFrameId(LF)].parentJoint;

  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, RF_id, LOCAL);
  RigidConstraintData cd_RF_6D(ci_RF_6D);
  // RigidConstraintModel ci_RF_3D(CONTACT_3D,model.getJointId(RF),WORLD);

  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, LF_id, LOCAL);
  RigidConstraintData cd_LF_6D(ci_LF_6D);
  // RigidConstraintModel ci_LF_3D(CONTACT_3D,model.getJointId(LF),WORLD);

  // RigidConstraintModel ci_RA_3D(CONTACT_3D,model.getJointId(RA),WORLD);
  // RigidConstraintModel ci_LA_3D(CONTACT_3D,model.getJointId(LA),WORLD);

  // Define contact infos structure
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_empty;

  ContactCholeskyDecomposition contact_chol_empty(model, contact_models_empty);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D;
  contact_models_6D.push_back(ci_RF_6D);
  contact_data_6D.push_back(cd_RF_6D);

  ContactCholeskyDecomposition contact_chol_6D(model, contact_models_6D);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
  contact_models_6D6D.push_back(ci_RF_6D);
  contact_models_6D6D.push_back(ci_LF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D6D;
  contact_data_6D6D.push_back(cd_RF_6D);
  contact_data_6D6D.push_back(cd_LF_6D);

  ContactCholeskyDecomposition contact_chol_6D6D(model, contact_models_6D6D);

  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;

  Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qs(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qdots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qddots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) taus(NBT);

  for (size_t i = 0; i < NBT; ++i)
  {
    qs[i] = randomConfiguration(model, -qmax, qmax);
    qdots[i] = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    computeABADerivatives(model, data, qs[_smooth], qdots[_smooth], taus[_smooth]);
  }
  std::cout << "ABA derivatives= \t\t\t";
  timer.toc(std::cout, NBT);

  double total_time = 0;
  initConstraintDynamics(model, data, contact_models_empty);
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_empty,
      contact_data_empty);
    timer.tic();
    computeConstraintDynamicsDerivatives(model, data, contact_models_empty, contact_data_empty);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "constraintDynamicsDerivs {} = \t\t" << (total_time / NBT) << std::endl;

  total_time = 0;
  initConstraintDynamics(model, data, contact_models_6D);
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D, contact_data_6D);
    timer.tic();
    computeConstraintDynamicsDerivatives(model, data, contact_models_6D, contact_data_6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "constraintDynamicsDerivs {6D} = \t\t" << (total_time / NBT) << std::endl;

  total_time = 0;
  initConstraintDynamics(model, data, contact_models_6D6D);
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D6D,
      contact_data_6D6D);
    timer.tic();
    computeConstraintDynamicsDerivatives(model, data, contact_models_6D6D, contact_data_6D6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "constraintDynamicsDerivs {6D,6D} = \t" << (total_time / NBT) << std::endl;
  return 0;
}
