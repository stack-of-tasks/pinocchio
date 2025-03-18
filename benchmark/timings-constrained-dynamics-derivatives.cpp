//
// Copyright (c) 2019-2025 LAAS-CNRS INRIA
//
//
#include "model-fixture.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

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
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// COMPUTE_ABA_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeABADerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau)
{
  pinocchio::computeABADerivatives(model, data, q, v, tau);
}
BENCHMARK_DEFINE_F(ContactFixture, COMPUTE_ABA_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeABADerivativesCall(model, data, q, v, tau);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, COMPUTE_ABA_DERIVATIVES)->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_DERIVATIVES_EMPTY

PINOCCHIO_DONT_INLINE static void constraintDynamicsDerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data)
{
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);
}
BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_EMPTY)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_empty);
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models_empty, contact_data_empty);
  for (auto _ : st)
  {
    constraintDynamicsDerivativesCall(model, data, contact_models_empty, contact_data_empty);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_EMPTY)->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_DERIVATIVES_6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D);
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models_6D, contact_data_6D);
  for (auto _ : st)
  {
    constraintDynamicsDerivativesCall(model, data, contact_models_6D, contact_data_6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_6D)->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_DERIVATIVES_6D6D

BENCHMARK_DEFINE_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_6D6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D6D);
  pinocchio::constraintDynamics(model, data, q, v, tau, contact_models_6D6D, contact_data_6D6D);
  for (auto _ : st)
  {
    constraintDynamicsDerivativesCall(model, data, contact_models_6D6D, contact_data_6D6D);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_6D6D)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN();
