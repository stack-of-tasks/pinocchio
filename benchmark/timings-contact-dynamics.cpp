//
// Copyright (c) 2019-2024 INRIA
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
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/pv.hpp"

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
  const JointIndex RA_id = model.frames[model.getFrameId(RA)].parent;
  const std::string LA = "LARM_LINK6";
  const JointIndex LA_id = model.frames[model.getFrameId(LA)].parent;
  const std::string RF = "RLEG_LINK6";
  const JointIndex RF_id = model.frames[model.getFrameId(RF)].parent;
  const std::string LF = "LLEG_LINK6";
  const JointIndex LF_id = model.frames[model.getFrameId(LF)].parent;

  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, RF_id, LOCAL);
  RigidConstraintModel ci_RF_3D(CONTACT_3D, model, RF_id, LOCAL);

  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, LF_id, LOCAL);
  RigidConstraintModel ci_LF_3D(CONTACT_3D, model, LF_id, LOCAL);

  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, RA_id, LOCAL);
  RigidConstraintModel ci_LA_3D(CONTACT_3D, model, LA_id, LOCAL);

  // Define contact infos structure
  static const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_empty;
  static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_empty;
  ContactCholeskyDecomposition contact_chol_empty(model, contact_models_empty);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D;
  contact_models_6D.push_back(ci_RF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D;
  contact_data_6D.push_back(RigidConstraintData(ci_RF_6D));
  ContactCholeskyDecomposition contact_chol_6D(model, contact_models_6D);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models_6D6D;
  contact_models_6D6D.push_back(ci_RF_6D);
  contact_models_6D6D.push_back(ci_LF_6D);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data_6D6D;
  contact_data_6D6D.push_back(RigidConstraintData(ci_RF_6D));
  contact_data_6D6D.push_back(RigidConstraintData(ci_LF_6D));
  ContactCholeskyDecomposition contact_chol_6D6D(model, contact_models_6D6D);

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
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::WORLD);
  }
  std::cout << "ABA = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_empty,
      contact_data_empty);
  }
  std::cout << "contact ABA = \t\t";
  timer.toc(std::cout, NBT);

  initPvSolver(model, data, contact_models_empty);
  timer.tic();
  SMOOTH(NBT)
  {
    pv(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_empty,
      contact_data_empty, prox_settings);
  }
  std::cout << "PV = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    constrainedABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_empty,
      contact_data_empty, prox_settings);
  }
  std::cout << "constrainedABA = \t\t";
  timer.toc(std::cout, NBT);

  double total_time = 0;
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
    timer.tic();
    cholesky::decompose(model, data);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Sparse Cholesky = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    timer.tic();
    contact_chol_empty.compute(model, data, contact_models_empty, contact_data_empty);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  total_time = 0;
  MatrixXd H_inverse(contact_chol_empty.size(), contact_chol_empty.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    contact_chol_empty.compute(model, data, contact_models_empty, contact_data_empty);
    timer.tic();
    contact_chol_empty.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  initConstraintDynamics(model, data, contact_models_empty);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_empty,
      contact_data_empty);
  }
  std::cout << "constraintDynamics {} = \t\t";
  timer.toc(std::cout, NBT);

  std::cout << "--" << std::endl;
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    timer.tic();
    contact_chol_6D.compute(model, data, contact_models_6D, contact_data_6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  total_time = 0;
  H_inverse.resize(contact_chol_6D.size(), contact_chol_6D.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    contact_chol_6D.compute(model, data, contact_models_6D, contact_data_6D);
    timer.tic();
    contact_chol_6D.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  MatrixXd J(contact_chol_6D.constraintDim(), model.nv);
  J.setZero();
  MatrixXd MJtJ_inv(
    model.nv + contact_chol_6D.constraintDim(), model.nv + contact_chol_6D.constraintDim());
  MJtJ_inv.setZero();

  VectorXd gamma(contact_chol_6D.constraintDim());
  gamma.setZero();

  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    timer.tic();
    getJointJacobian(model, data, ci_RF_6D.joint1_id, ci_RF_6D.reference_frame, J.middleRows<6>(0));
    total_time += timer.toc(timer.DEFAULT_UNIT);

    forwardDynamics(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);

    timer.tic();
    cholesky::decompose(model, data);
    getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "KKTContactDynamicMatrixInverse {6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  initConstraintDynamics(model, data, contact_models_6D);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D, contact_data_6D);
  }
  std::cout << "constraintDynamics {6D} = \t\t";
  timer.toc(std::cout, NBT);
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D, contact_data_6D,
      prox_settings);
  }
  std::cout << "contact ABA {6D} = \t\t";
  timer.toc(std::cout, NBT);

  initPvSolver(model, data, contact_models_6D);
  timer.tic();
  SMOOTH(NBT)
  {
    pv(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D, contact_data_6D,
      prox_settings);
  }
  std::cout << "PV = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    constrainedABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D, contact_data_6D,
      prox_settings);
  }
  std::cout << "constrainedABA = \t\t";
  timer.toc(std::cout, NBT);
  std::cout << "--" << std::endl;

  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    timer.tic();
    contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {6D,6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  total_time = 0;
  H_inverse.resize(contact_chol_6D6D.size(), contact_chol_6D6D.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
    timer.tic();
    contact_chol_6D6D.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {6D,6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  J.resize(contact_chol_6D6D.constraintDim(), model.nv);
  J.setZero();
  MJtJ_inv.resize(
    model.nv + contact_chol_6D6D.constraintDim(), model.nv + contact_chol_6D6D.constraintDim());
  MJtJ_inv.setZero();

  gamma.resize(contact_chol_6D6D.constraintDim());
  gamma.setZero();

  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    getJointJacobian(model, data, ci_RF_6D.joint1_id, ci_RF_6D.reference_frame, J.middleRows<6>(0));
    getJointJacobian(model, data, ci_LF_6D.joint1_id, ci_LF_6D.reference_frame, J.middleRows<6>(6));

    forwardDynamics(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);

    timer.tic();
    cholesky::decompose(model, data);
    getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "KKTContactDynamicMatrixInverse {6D,6D} = \t\t" << (total_time / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  initConstraintDynamics(model, data, contact_models_6D6D);
  timer.tic();
  SMOOTH(NBT)
  {
    constraintDynamics(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D6D,
      contact_data_6D6D);
  }
  std::cout << "constraintDynamics {6D,6D} = \t\t";
  timer.toc(std::cout, NBT);
  timer.tic();
  SMOOTH(NBT)
  {
    contactABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D6D,
      contact_data_6D6D, prox_settings);
  }
  std::cout << "contact ABA {6D,6D} = \t\t";
  timer.toc(std::cout, NBT);

  initPvSolver(model, data, contact_models_6D6D);
  timer.tic();
  SMOOTH(NBT)
  {
    pv(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D6D,
      contact_data_6D6D, prox_settings);
  }
  std::cout << "PV = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    constrainedABA(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], contact_models_6D6D,
      contact_data_6D6D, prox_settings);
  }
  std::cout << "constrainedABA = \t\t";
  timer.toc(std::cout, NBT);

  J.setZero();
  timer.tic();
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
    getJointJacobian(model, data, ci_RF_6D.joint1_id, ci_RF_6D.reference_frame, J.middleRows<6>(0));
    getJointJacobian(model, data, ci_LF_6D.joint1_id, ci_LF_6D.reference_frame, J.middleRows<6>(6));
    forwardDynamics(model, data, taus[_smooth], J, gamma);
  }
  std::cout << "forwardDynamics {6D,6D} = \t\t";
  timer.toc(std::cout, NBT);

  std::cout << "--" << std::endl;

  return 0;
}
