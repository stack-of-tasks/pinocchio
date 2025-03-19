//
// Copyright (c) 2019-2025 INRIA
//

#include "model-fixture.hpp"

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

#include <benchmark/benchmark.h>

#include <iostream>

struct ContactFixture : ModelFixture
{
  void SetUp(benchmark::State & st)
  {
    ModelFixture::SetUp(st);

    const std::string RF = "RLEG_LINK6";
    const auto RF_id = model.frames[model.getFrameId(RF)].parentJoint;
    const std::string LF = "LLEG_LINK6";
    const auto LF_id = model.frames[model.getFrameId(LF)].parentJoint;

    ci_RF_6D = std::make_unique<pinocchio::RigidConstraintModel>(
      pinocchio::CONTACT_6D, model, RF_id, pinocchio::LOCAL);
    ci_LF_6D = std::make_unique<pinocchio::RigidConstraintModel>(
      pinocchio::CONTACT_6D, model, LF_id, pinocchio::LOCAL);

    contact_chol_empty = pinocchio::ContactCholeskyDecomposition(model, contact_models_empty);

    contact_models_6D.clear();
    contact_models_6D.push_back(*ci_RF_6D);

    contact_data_6D.clear();
    contact_data_6D.push_back(pinocchio::RigidConstraintData(*ci_RF_6D));

    contact_chol_6D = pinocchio::ContactCholeskyDecomposition(model, contact_models_6D);

    contact_models_6D6D.clear();
    contact_models_6D6D.push_back(*ci_RF_6D);
    contact_models_6D6D.push_back(*ci_LF_6D);

    contact_data_6D6D.clear();
    contact_data_6D6D.push_back(pinocchio::RigidConstraintData(*ci_RF_6D));
    contact_data_6D6D.push_back(pinocchio::RigidConstraintData(*ci_LF_6D));

    contact_chol_6D6D = pinocchio::ContactCholeskyDecomposition(model, contact_models_6D6D);

    prox_settings.max_iter = 10;
    prox_settings.mu = 1e8;

    num_constraints = 12;

    col_major_square_matrices = Eigen::MatrixXd::Random(num_constraints, num_constraints)
                                + Eigen::MatrixXd::Identity(num_constraints, num_constraints);
  }

  void TearDown(benchmark::State & st)
  {
    ModelFixture::TearDown(st);
  }

  std::unique_ptr<pinocchio::RigidConstraintModel> ci_RF_6D;
  std::unique_ptr<pinocchio::RigidConstraintModel> ci_LF_6D;

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_data_empty;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models_6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_data_6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models_6D6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_data_6D6D;

  pinocchio::ContactCholeskyDecomposition contact_chol_empty;
  pinocchio::ContactCholeskyDecomposition contact_chol_6D;
  pinocchio::ContactCholeskyDecomposition contact_chol_6D6D;

  pinocchio::ProximalSettings prox_settings;

  int num_constraints = 12;

  Eigen::MatrixXd col_major_square_matrices;
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// ABA_WORLD

PINOCCHIO_DONT_INLINE static void abaWorldCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau)
{
  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(ContactFixture, ABA_WORLD)(benchmark::State & st)
{
  for (auto _ : st)
  {
    abaWorldCall(model, data, q, v, tau);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, ABA_WORLD)->Apply(CustomArguments);

// CHOLESKY_DECOMPOSE

PINOCCHIO_DONT_INLINE static void
choleskyDecomposeCall(const pinocchio::Model & model, pinocchio::Data & data)
{
  pinocchio::cholesky::decompose(model, data);
}
BENCHMARK_DEFINE_F(ContactFixture, CHOLESKY_DECOMPOSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    choleskyDecomposeCall(model, data);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CHOLESKY_DECOMPOSE)->Apply(CustomArguments);

// CONTACT_ABA_EMPTY

PINOCCHIO_DONT_INLINE static void contactABACall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data)
{
  pinocchio::contactABA(model, data, q, v, tau, contact_models, contact_data);
}
BENCHMARK_DEFINE_F(ContactFixture, CONTACT_ABA_EMPTY)(benchmark::State & st)
{
  for (auto _ : st)
  {
    contactABACall(model, data, q, v, tau, contact_models_empty, contact_data_empty);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_ABA_EMPTY)->Apply(CustomArguments);

// PV_EMPTY

PINOCCHIO_DONT_INLINE static void pvCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data,
  pinocchio::ProximalSettings & prox_settings)
{
  pinocchio::pv(model, data, q, v, tau, contact_models, contact_data, prox_settings);
}
BENCHMARK_DEFINE_F(ContactFixture, PV_EMPTY)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_empty);
  for (auto _ : st)
  {
    pvCall(model, data, q, v, tau, contact_models_empty, contact_data_empty, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, PV_EMPTY)->Apply(CustomArguments);

// CONSTRAINED_ABA_EMPTY

PINOCCHIO_DONT_INLINE static void constrainedABACall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data,
  pinocchio::ProximalSettings & prox_settings)
{
  pinocchio::constrainedABA(model, data, q, v, tau, contact_models, contact_data, prox_settings);
}
BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINED_ABA_EMPTY)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_empty);
  for (auto _ : st)
  {
    constrainedABACall(
      model, data, q, v, tau, contact_models_empty, contact_data_empty, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINED_ABA_EMPTY)->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_EMPTY

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionComputeCall(
  pinocchio::ContactCholeskyDecomposition & contact,
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data)
{
  contact.compute(model, data, contact_models, contact_data);
}

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_EMPTY)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  for (auto _ : st)
  {
    contactCholeskyDecompositionComputeCall(
      contact_chol_empty, model, data, contact_models_empty, contact_data_empty);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_EMPTY)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_EMPTY

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionInverseCall(
  pinocchio::ContactCholeskyDecomposition & contact, Eigen::MatrixXd & H_inverse)
{
  contact.inverse(H_inverse);
}

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_EMPTY)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_empty.compute(model, data, contact_models_empty, contact_data_empty);
  Eigen::MatrixXd H_inverse(contact_chol_empty.size(), contact_chol_empty.size());
  for (auto _ : st)
  {
    contactCholeskyDecompositionInverseCall(contact_chol_empty, H_inverse);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_EMPTY)
  ->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_EMPTY

PINOCCHIO_DONT_INLINE static void constraintDynamicsCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data)
{
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models, contact_data);
}
BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_EMPTY)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_empty);
  for (auto _ : st)
  {
    constraintDynamicsCall(model, data, q, v, tau, contact_models_empty, contact_data_empty);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_EMPTY)->Apply(CustomArguments);

// CONTACT_ABA_6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_ABA_6D)(benchmark::State & st)
{
  for (auto _ : st)
  {
    contactABACall(model, data, q, v, tau, contact_models_6D, contact_data_6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_ABA_6D)->Apply(CustomArguments);

// PV_6D

BENCHMARK_DEFINE_F(ContactFixture, PV_6D)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_6D);
  for (auto _ : st)
  {
    pvCall(model, data, q, v, tau, contact_models_6D, contact_data_6D, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, PV_6D)->Apply(CustomArguments);

// CONSTRAINED_ABA_6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINED_ABA_6D)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_6D);
  for (auto _ : st)
  {
    constrainedABACall(model, data, q, v, tau, contact_models_6D, contact_data_6D, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINED_ABA_6D)->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D)(benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  for (auto _ : st)
  {
    contactCholeskyDecompositionComputeCall(
      contact_chol_6D, model, data, contact_models_6D, contact_data_6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D)(benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_6D.compute(model, data, contact_models_6D, contact_data_6D);
  Eigen::MatrixXd H_inverse(contact_chol_6D.size(), contact_chol_6D.size());
  for (auto _ : st)
  {
    contactCholeskyDecompositionInverseCall(contact_chol_6D, H_inverse);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D)
  ->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D);
  for (auto _ : st)
  {
    constraintDynamicsCall(model, data, q, v, tau, contact_models_6D, contact_data_6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_6D)->Apply(CustomArguments);

// GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D

PINOCCHIO_DONT_INLINE static void getKKTContactDynamicMatrixInverseCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::MatrixXd & J,
  const Eigen::MatrixXd & MJtJ_inv)
{
  pinocchio::cholesky::decompose(model, data);
  pinocchio::getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
}

BENCHMARK_DEFINE_F(ContactFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D)(benchmark::State & st)
{
  Eigen::MatrixXd J(contact_chol_6D.constraintDim(), model.nv);
  J.setZero();

  Eigen::MatrixXd MJtJ_inv(
    model.nv + contact_chol_6D.constraintDim(), model.nv + contact_chol_6D.constraintDim());
  MJtJ_inv.setZero();

  Eigen::VectorXd gamma(contact_chol_6D.constraintDim());
  gamma.setZero();

  pinocchio::computeAllTerms(model, data, q, v);
  pinocchio::getJointJacobian(
    model, data, ci_RF_6D->joint1_id, ci_RF_6D->reference_frame, J.middleRows<6>(0));
  pinocchio::forwardDynamics(model, data, q, v, tau, J, gamma);

  for (auto _ : st)
  {
    getKKTContactDynamicMatrixInverseCall(model, data, J, MJtJ_inv);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D)
  ->Apply(CustomArguments);

// CONTACT_ABA_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_ABA_6D6D)(benchmark::State & st)
{
  for (auto _ : st)
  {
    contactABACall(model, data, q, v, tau, contact_models_6D6D, contact_data_6D6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_ABA_6D6D)->Apply(CustomArguments);

// PV_6D6D

BENCHMARK_DEFINE_F(ContactFixture, PV_6D6D)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_6D6D);
  for (auto _ : st)
  {
    pvCall(model, data, q, v, tau, contact_models_6D6D, contact_data_6D6D, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, PV_6D6D)->Apply(CustomArguments);

// CONSTRAINED_ABA_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINED_ABA_6D6D)(benchmark::State & st)
{
  pinocchio::initPvSolver(model, data, contact_models_6D6D);
  for (auto _ : st)
  {
    constrainedABACall(
      model, data, q, v, tau, contact_models_6D6D, contact_data_6D6D, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINED_ABA_6D6D)->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D6D)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  for (auto _ : st)
  {
    contactCholeskyDecompositionComputeCall(
      contact_chol_6D6D, model, data, contact_models_6D6D, contact_data_6D6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE_6D6D)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D6D)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
  Eigen::MatrixXd H_inverse(contact_chol_6D6D.size(), contact_chol_6D6D.size());
  for (auto _ : st)
  {
    contactCholeskyDecompositionInverseCall(contact_chol_6D6D, H_inverse);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE_6D6D)
  ->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_6D6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D6D);
  for (auto _ : st)
  {
    constraintDynamicsCall(model, data, q, v, tau, contact_models_6D6D, contact_data_6D6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_6D6D)->Apply(CustomArguments);

// GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D6D

BENCHMARK_DEFINE_F(ContactFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D6D)(
  benchmark::State & st)
{
  Eigen::MatrixXd J(contact_chol_6D6D.constraintDim(), model.nv);
  J.setZero();

  Eigen::MatrixXd MJtJ_inv(
    model.nv + contact_chol_6D6D.constraintDim(), model.nv + contact_chol_6D6D.constraintDim());
  MJtJ_inv.setZero();

  Eigen::VectorXd gamma(contact_chol_6D6D.constraintDim());
  gamma.setZero();

  computeAllTerms(model, data, q, v);
  getJointJacobian(model, data, ci_RF_6D->joint1_id, ci_RF_6D->reference_frame, J.middleRows<6>(0));
  getJointJacobian(model, data, ci_LF_6D->joint1_id, ci_LF_6D->reference_frame, J.middleRows<6>(6));
  forwardDynamics(model, data, q, v, tau, J, gamma);

  for (auto _ : st)
  {
    getKKTContactDynamicMatrixInverseCall(model, data, J, MJtJ_inv);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE_6D6D)
  ->Apply(CustomArguments);

// FORWARD_DYNAMICS_6D6D

PINOCCHIO_DONT_INLINE static void forwardDynamisCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::RigidConstraintModel & c1,
  const pinocchio::RigidConstraintModel & c2,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  Eigen::MatrixXd & J,
  const Eigen::VectorXd & gamma)
{
  pinocchio::computeAllTerms(model, data, q, v);
  pinocchio::getJointJacobian(model, data, c1.joint1_id, c1.reference_frame, J.middleRows<6>(0));
  pinocchio::getJointJacobian(model, data, c2.joint1_id, c2.reference_frame, J.middleRows<6>(6));
  pinocchio::forwardDynamics(model, data, tau, J, gamma);
}

BENCHMARK_DEFINE_F(ContactFixture, FORWARD_DYNAMICS_6D6D)(benchmark::State & st)
{
  Eigen::MatrixXd J(contact_chol_6D6D.constraintDim(), model.nv);
  J.setZero();

  Eigen::VectorXd gamma(contact_chol_6D6D.constraintDim());
  gamma.setZero();

  initConstraintDynamics(model, data, contact_models_6D6D);
  for (auto _ : st)
  {
    forwardDynamisCall(model, data, *ci_RF_6D, *ci_LF_6D, q, v, tau, J, gamma);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, FORWARD_DYNAMICS_6D6D)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN();
