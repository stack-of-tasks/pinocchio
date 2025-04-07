//
// Copyright (c) 2025 CNRS INRIA
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
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
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

    r_coeff = (Eigen::ArrayXd::Random(1)[0] + 1.) / 2.;

    prox_settings.max_iter = 10;
    prox_settings.mu = 1e8;
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

  double r_coeff;

  pinocchio::ProximalSettings prox_settings;
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// IMPULSE_DYNAMICS_EMPTY

PINOCCHIO_DONT_INLINE static void impulseDynamicsCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) & contact_models,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_data,
  double r_coeff,
  const pinocchio::ProximalSettings & prox_settings)
{
  pinocchio::impulseDynamics(
    model, data, q, v, contact_models, contact_data, r_coeff, prox_settings);
}
BENCHMARK_DEFINE_F(ContactFixture, IMPULSE_DYNAMICS_EMPTY)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_empty);
  for (auto _ : st)
  {
    impulseDynamicsCall(
      model, data, q, v, contact_models_empty, contact_data_empty, r_coeff, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, IMPULSE_DYNAMICS_EMPTY)->Apply(CustomArguments);

// IMPULSE_DYNAMICS_6D

BENCHMARK_DEFINE_F(ContactFixture, IMPULSE_DYNAMICS_6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D);
  for (auto _ : st)
  {
    impulseDynamicsCall(
      model, data, q, v, contact_models_6D, contact_data_6D, r_coeff, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, IMPULSE_DYNAMICS_6D)->Apply(CustomArguments);

// IMPULSE_DYNAMICS_6D6D

BENCHMARK_DEFINE_F(ContactFixture, IMPULSE_DYNAMICS_6D6D)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, contact_models_6D6D);
  for (auto _ : st)
  {
    impulseDynamicsCall(
      model, data, q, v, contact_models_6D6D, contact_data_6D6D, r_coeff, prox_settings);
  }
}
BENCHMARK_REGISTER_F(ContactFixture, IMPULSE_DYNAMICS_6D6D)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN();
