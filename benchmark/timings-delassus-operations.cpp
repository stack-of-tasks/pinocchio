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
#include "pinocchio/algorithm/delassus.hpp"

#include <benchmark/benchmark.h>

#include <iostream>

struct DelassusFixture : ModelFixture
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

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models_6D6D;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) contact_data_6D6D;

  pinocchio::ContactCholeskyDecomposition contact_chol_6D6D;

  pinocchio::ProximalSettings prox_settings;

  int num_constraints = 12;

  Eigen::MatrixXd col_major_square_matrices;
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionComputeCall(
  pinocchio::ContactCholeskyDecomposition & contact,
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel)
    & contact_models_6D6D,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data_6D6D)
{
  contact.compute(model, data, contact_models_6D6D, contact_data_6D6D);
}

BENCHMARK_DEFINE_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE)(benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  for (auto _ : st)
  {
    contactCholeskyDecompositionComputeCall(
      contact_chol_6D6D, model, data, contact_models_6D6D, contact_data_6D6D);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_COMPUTE)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_INVERSE

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionInverseCall(
  pinocchio::ContactCholeskyDecomposition & contact, Eigen::MatrixXd & H_inverse)
{
  contact.inverse(H_inverse);
}

BENCHMARK_DEFINE_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE)(benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
  Eigen::MatrixXd H_inverse(contact_chol_6D6D.size(), contact_chol_6D6D.size());
  for (auto _ : st)
  {
    contactCholeskyDecompositionInverseCall(contact_chol_6D6D, H_inverse);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_INVERSE)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_UPDATE_DAMPING

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionUpdateDampingCall(
  pinocchio::ContactCholeskyDecomposition & contact, double damping)
{
  contact.getDelassusCholeskyExpression().updateDamping(damping);
}

BENCHMARK_DEFINE_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_UPDATE_DAMPING)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
  for (auto _ : st)
  {
    contactCholeskyDecompositionUpdateDampingCall(contact_chol_6D6D, 1.);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_UPDATE_DAMPING)
  ->Apply(CustomArguments);

// CONTACT_CHOLESKY_DECOMPOSITION_SOLVE_IN_PLACE

PINOCCHIO_DONT_INLINE static void contactCholeskyDecompositionSolveInPlaceCall(
  pinocchio::ContactCholeskyDecomposition & contact, const Eigen::VectorXd & rhs_vector)
{
  contact.getDelassusCholeskyExpression().solveInPlace(rhs_vector);
}

BENCHMARK_DEFINE_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_SOLVE_IN_PLACE)(
  benchmark::State & st)
{
  pinocchio::computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D);
  Eigen::VectorXd rhs_vector = Eigen::VectorXd::Random(num_constraints);
  for (auto _ : st)
  {
    contactCholeskyDecompositionSolveInPlaceCall(contact_chol_6D6D, rhs_vector);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, CONTACT_CHOLESKY_DECOMPOSITION_SOLVE_IN_PLACE)
  ->Apply(CustomArguments);

// COLUMN_MAJOR_LLT_COMPUTE

PINOCCHIO_DONT_INLINE static void
columnMajorLLTComputeCall(Eigen::LLT<Eigen::MatrixXd> & llt, const Eigen::MatrixXd & m)
{
  llt.compute(m);
}

BENCHMARK_DEFINE_F(DelassusFixture, COLUMN_MAJOR_LLT_COMPUTE)(benchmark::State & st)
{
  Eigen::LLT<Eigen::MatrixXd> col_major_llt(num_constraints);
  for (auto _ : st)
  {
    columnMajorLLTComputeCall(col_major_llt, col_major_square_matrices);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, COLUMN_MAJOR_LLT_COMPUTE)->Apply(CustomArguments);

// COLUMN_MAJOR_LLT_SOLVE_IN_PLACE

PINOCCHIO_DONT_INLINE static void
columnMajorLLTSolveInPlaceCall(Eigen::LLT<Eigen::MatrixXd> & llt, const Eigen::VectorXd & v)
{
  llt.solveInPlace(v);
}

BENCHMARK_DEFINE_F(DelassusFixture, COLUMN_MAJOR_LLT_SOLVE_IN_PLACE)(benchmark::State & st)
{
  Eigen::LLT<Eigen::MatrixXd> col_major_llt(num_constraints);
  Eigen::VectorXd rhs_vector = Eigen::VectorXd::Random(num_constraints);
  col_major_llt.compute(col_major_square_matrices);
  for (auto _ : st)
  {
    columnMajorLLTSolveInPlaceCall(col_major_llt, rhs_vector);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, COLUMN_MAJOR_LLT_SOLVE_IN_PLACE)->Apply(CustomArguments);

// GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE

PINOCCHIO_DONT_INLINE static void getKKTContactDynamicMatrixInverseCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::MatrixXd & J,
  const Eigen::MatrixXd & MJtJ_inv)
{
  pinocchio::cholesky::decompose(model, data);
  pinocchio::getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
}

BENCHMARK_DEFINE_F(DelassusFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE)(benchmark::State & st)
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
BENCHMARK_REGISTER_F(DelassusFixture, GET_KKT_CONTACT_DYNAMIC_MATRIX_INVERSE)
  ->Apply(CustomArguments);

// COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE

PINOCCHIO_DONT_INLINE static void computeDampedDelassusMatrixInverseCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data,
  const Eigen::MatrixXd & damped_delassus_inverse)
{
  pinocchio::computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, damped_delassus_inverse, 1e-6);
}

BENCHMARK_DEFINE_F(DelassusFixture, COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE)(benchmark::State & st)
{
  Eigen::MatrixXd H_inverse(contact_chol_6D6D.size(), contact_chol_6D6D.size());

  computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D, 1e-6);
  contact_chol_6D6D.inverse(H_inverse);

  Eigen::MatrixXd damped_delassus_inverse;
  damped_delassus_inverse.resize(
    contact_chol_6D6D.constraintDim(), contact_chol_6D6D.constraintDim());

  initPvDelassus(model, data, contact_models_6D6D); // Allocate memory

  for (auto _ : st)
  {
    computeDampedDelassusMatrixInverseCall(
      model, data, q, contact_models_6D6D, contact_data_6D6D, damped_delassus_inverse);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE)
  ->Apply(CustomArguments);

// COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE_NO_SCALE_NO_PV

PINOCCHIO_DONT_INLINE static void computeDampedDelassusMatrixInverseNoScaleNoPvCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data,
  const Eigen::MatrixXd & damped_delassus_inverse)
{
  pinocchio::computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, damped_delassus_inverse, 1e-6, false, false);
}

BENCHMARK_DEFINE_F(DelassusFixture, COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE_NO_SCALE_NO_PV)(
  benchmark::State & st)
{
  Eigen::MatrixXd H_inverse(contact_chol_6D6D.size(), contact_chol_6D6D.size());

  computeAllTerms(model, data, q, v);
  contact_chol_6D6D.compute(model, data, contact_models_6D6D, contact_data_6D6D, 1e-6);
  contact_chol_6D6D.inverse(H_inverse);

  Eigen::MatrixXd damped_delassus_inverse;
  damped_delassus_inverse.resize(
    contact_chol_6D6D.constraintDim(), contact_chol_6D6D.constraintDim());

  initPvDelassus(model, data, contact_models_6D6D); // Allocate memory

  for (auto _ : st)
  {
    computeDampedDelassusMatrixInverseNoScaleNoPvCall(
      model, data, q, contact_models_6D6D, contact_data_6D6D, damped_delassus_inverse);
  }
}
BENCHMARK_REGISTER_F(DelassusFixture, COMPUTE_DAMPED_DELASSUS_MATRIX_INVERSE_NO_SCALE_NO_PV)
  ->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN();
