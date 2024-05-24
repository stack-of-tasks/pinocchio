//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
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
  const std::string LA = "LARM_LINK6";
  const std::string RF = "RLEG_LINK6";
  const std::string LF = "LLEG_LINK6";

  RigidConstraintModel ci_RF_6D(CONTACT_6D, model.getFrameId(RF), WORLD);
  RigidConstraintModel ci_RF_3D(CONTACT_3D, model.getFrameId(RF), WORLD);

  RigidConstraintModel ci_LF_6D(CONTACT_6D, model.getFrameId(LF), WORLD);
  RigidConstraintModel ci_LF_3D(CONTACT_3D, model.getFrameId(LF), WORLD);

  RigidConstraintModel ci_RA_3D(CONTACT_3D, model.getFrameId(RA), WORLD);
  RigidConstraintModel ci_LA_3D(CONTACT_3D, model.getFrameId(LA), WORLD);

  // Define contact infos structure
  static const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_empty;
  cholesky::ContactCholeskyDecomposition contact_chol_empty(model, contact_models_empty);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
  contact_models_6D.push_back(ci_RF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D;
  contact_data_6D.push_back(RigidConstraintData(ci_RF_6D));
  cholesky::ContactCholeskyDecomposition contact_chol_6D(model, contact_models_6D);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
  contact_models_6D6D.push_back(ci_RF_6D);
  contact_models_6D6D.push_back(ci_LF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D6D;
  contact_data_6D6D.push_back(RigidConstraintData(ci_RF_6D));
  contact_data_6D6D.push_back(RigidConstraintData(ci_LF_6D));
  cholesky::ContactCholeskyDecomposition contact_chol_6D6D(model, contact_models_6D6D);

  ProximalSettings prox_settings;
  prox_settings.max_iter = 10;
  prox_settings.mu = 1e8;

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

  for (size_t i = 0; i < NBT; ++i)
  {
    qs[i] = randomConfiguration(model, -qmax, qmax);
    qdots[i] = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
  }
  Eigen::ArrayXd r_coeffs = (Eigen::ArrayXd::Random(NBT) + 1.) / 2.;

  initConstraintDynamics(model, data, contact_models_empty);
  timer.tic();
  SMOOTH(NBT)
  {
    impulseDynamics(
      model, data, qs[_smooth], qdots[_smooth], contact_models_empty, contact_data_empty,
      r_coeffs[(Eigen::Index)_smooth]);
  }
  std::cout << "impulseDynamics {} = \t\t";
  timer.toc(std::cout, NBT);

  initConstraintDynamics(model, data, contact_models_6D);
  timer.tic();
  SMOOTH(NBT)
  {
    impulseDynamics(
      model, data, qs[_smooth], qdots[_smooth], contact_models_6D, contact_data_6D,
      r_coeffs[(Eigen::Index)_smooth]);
  }
  std::cout << "impulseDynamics {6D} = \t\t";
  timer.toc(std::cout, NBT);

  initConstraintDynamics(model, data, contact_models_6D6D);
  timer.tic();
  SMOOTH(NBT)
  {
    impulseDynamics(
      model, data, qs[_smooth], qdots[_smooth], contact_models_6D6D, contact_data_6D6D,
      r_coeffs[(Eigen::Index)_smooth]);
  }
  std::cout << "impulseDynamics {6D,6D} = \t\t";
  timer.toc(std::cout, NBT);

  Eigen::MatrixXd J(Eigen::MatrixXd::Zero(12, model.nv));
  timer.tic();
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
    getFrameJacobian(model, data, ci_RF_6D.frame_id, ci_RF_6D.reference_frame, J.middleRows<6>(0));
    getFrameJacobian(model, data, ci_LF_6D.frame_id, ci_LF_6D.reference_frame, J.middleRows<6>(6));
    impulseDynamics(model, data, qdots[_smooth], J, r_coeffs[(Eigen::Index)_smooth]);
  }
  std::cout << "constrained impulseDynamics {6D,6D} = \t\t";
  timer.toc(std::cout, NBT);

  std::cout << "--" << std::endl;

  return 0;
}
