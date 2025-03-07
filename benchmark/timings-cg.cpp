//
// Copyright (c) 2018 CNRS
// Copyright (c) 2020-2024 INRIA
//

#include "model-fixture.hpp"

#include "pinocchio/codegen/cppadcg.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/codegen/code-generator-algo.hpp"

#include <iostream>

struct CGFixture : ModelFixture
{
  void SetUp(benchmark::State & st)
  {
    ModelFixture::SetUp(st);
  }

  void TearDown(benchmark::State & st)
  {
    ModelFixture::TearDown(st);
  }

  static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel)
    CONTACT_MODELS_6D6D;
  static PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData)
    CONTACT_DATAS_6D6D;

  // Initialize all as a global variable to avoid long running time
  static std::unique_ptr<pinocchio::CodeGenRNEA<double>> RNEA_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenABA<double>> ABA_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenCRBA<double>> CRBA_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenMinv<double>> MINV_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenRNEADerivatives<double>> RNEA_DERIVATIVES_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenABADerivatives<double>> ABA_DERIVATIVES_CODE_GEN;
  static std::unique_ptr<pinocchio::CodeGenConstraintDynamicsDerivatives<double>>
    CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN;

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    ModelFixture::GlobalSetUp(extra_args);

    const std::string RF = "RLEG_ANKLE_R";
    const std::string LF = "LLEG_ANKLE_R";
    pinocchio::RigidConstraintModel ci_RF(
      pinocchio::CONTACT_6D, ModelFixture::MODEL, ModelFixture::MODEL.getFrameId(RF),
      pinocchio::ReferenceFrame::LOCAL);
    pinocchio::RigidConstraintModel ci_LF(
      pinocchio::CONTACT_6D, ModelFixture::MODEL, ModelFixture::MODEL.getFrameId(LF),
      pinocchio::ReferenceFrame::LOCAL);
    CONTACT_MODELS_6D6D.push_back(ci_RF);
    CONTACT_MODELS_6D6D.push_back(ci_LF);

    pinocchio::RigidConstraintData cd_RF(ci_RF);
    pinocchio::RigidConstraintData cd_LF(ci_LF);
    CONTACT_DATAS_6D6D.push_back(cd_RF);
    CONTACT_DATAS_6D6D.push_back(cd_LF);

    RNEA_CODE_GEN = std::make_unique<pinocchio::CodeGenRNEA<double>>(ModelFixture::MODEL);
    ABA_CODE_GEN = std::make_unique<pinocchio::CodeGenABA<double>>(ModelFixture::MODEL);
    CRBA_CODE_GEN = std::make_unique<pinocchio::CodeGenCRBA<double>>(ModelFixture::MODEL);
    MINV_CODE_GEN = std::make_unique<pinocchio::CodeGenMinv<double>>(ModelFixture::MODEL);
    RNEA_DERIVATIVES_CODE_GEN =
      std::make_unique<pinocchio::CodeGenRNEADerivatives<double>>(ModelFixture::MODEL);
    ABA_DERIVATIVES_CODE_GEN =
      std::make_unique<pinocchio::CodeGenABADerivatives<double>>(ModelFixture::MODEL);
    CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN =
      std::make_unique<pinocchio::CodeGenConstraintDynamicsDerivatives<double>>(
        ModelFixture::MODEL, CONTACT_MODELS_6D6D);

    RNEA_CODE_GEN->initLib();
    RNEA_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    ABA_CODE_GEN->initLib();
    ABA_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    CRBA_CODE_GEN->initLib();
    CRBA_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    MINV_CODE_GEN->initLib();
    MINV_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    RNEA_DERIVATIVES_CODE_GEN->initLib();
    RNEA_DERIVATIVES_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    ABA_DERIVATIVES_CODE_GEN->initLib();
    ABA_DERIVATIVES_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
    CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN->initLib();
    CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN->loadLib(true, PINOCCHIO_CXX_COMPILER);
  }
};

PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel)
CGFixture::CONTACT_MODELS_6D6D;
PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData)
CGFixture::CONTACT_DATAS_6D6D;
std::unique_ptr<pinocchio::CodeGenRNEA<double>> CGFixture::RNEA_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenABA<double>> CGFixture::ABA_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenCRBA<double>> CGFixture::CRBA_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenMinv<double>> CGFixture::MINV_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenRNEADerivatives<double>> CGFixture::RNEA_DERIVATIVES_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenABADerivatives<double>> CGFixture::ABA_DERIVATIVES_CODE_GEN;
std::unique_ptr<pinocchio::CodeGenConstraintDynamicsDerivatives<double>>
  CGFixture::CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN;

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// RNEA

PINOCCHIO_DONT_INLINE static void rneaCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::rnea(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(CGFixture, RNEA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    rneaCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, RNEA)->Apply(CustomArguments);

// RNEA_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, RNEA_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    RNEA_CODE_GEN->evalFunction(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, RNEA_CODE_GEN)->Apply(CustomArguments);

// RNEA_JACOBIAN_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, RNEA_JACOBIAN_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    RNEA_CODE_GEN->evalJacobian(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, RNEA_JACOBIAN_CODE_GEN)->Apply(CustomArguments);

// RNEA_DERIVATIVES_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, RNEA_DERIVATIVES_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    RNEA_DERIVATIVES_CODE_GEN->evalFunction(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, RNEA_DERIVATIVES_CODE_GEN)->Apply(CustomArguments);

// CRBA

PINOCCHIO_DONT_INLINE static void
crbaWorldCall(const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(CGFixture, CRBA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    crbaWorldCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(CGFixture, CRBA)->Apply(CustomArguments);

// CRBA_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, CRBA_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    CRBA_CODE_GEN->evalFunction(q);
  }
}
BENCHMARK_REGISTER_F(CGFixture, CRBA_CODE_GEN)->Apply(CustomArguments);

// COMPUTE_M_INVERSE

PINOCCHIO_DONT_INLINE static void computeMinverseQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeMinverse(model, data, q);
}
BENCHMARK_DEFINE_F(CGFixture, COMPUTE_M_INVERSE)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeMinverseQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(CGFixture, COMPUTE_M_INVERSE)->Apply(CustomArguments);

// COMPUTE_M_INVERSE_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, COMPUTE_M_INVERSE_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    MINV_CODE_GEN->evalFunction(q);
  }
}
BENCHMARK_REGISTER_F(CGFixture, COMPUTE_M_INVERSE_CODE_GEN)->Apply(CustomArguments);

// ABA

PINOCCHIO_DONT_INLINE static void abaWorldCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::aba(model, data, q, v, a, pinocchio::Convention::WORLD);
}
BENCHMARK_DEFINE_F(CGFixture, ABA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    abaWorldCall(model, data, q, v, tau);
  }
}
BENCHMARK_REGISTER_F(CGFixture, ABA)->Apply(CustomArguments);

// ABA_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, ABA_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    ABA_CODE_GEN->evalFunction(q, v, tau);
  }
}
BENCHMARK_REGISTER_F(CGFixture, ABA_CODE_GEN)->Apply(CustomArguments);

// ABA_JACOBIAN_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, ABA_JACOBIAN_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    ABA_CODE_GEN->evalJacobian(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, ABA_JACOBIAN_CODE_GEN)->Apply(CustomArguments);

// ABA_DERIVATIVES_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, ABA_DERIVATIVES_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    ABA_DERIVATIVES_CODE_GEN->evalFunction(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, ABA_DERIVATIVES_CODE_GEN)->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_DERIVATIVES

PINOCCHIO_DONT_INLINE void constraintDynamicsDerivativeCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel)
    & contact_models_6d6d,
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) & contact_datas_6d6d)
{
  pinocchio::constraintDynamics(model, data, q, v, a, contact_models_6d6d, contact_datas_6d6d);
  pinocchio::computeConstraintDynamicsDerivatives(
    model, data, contact_models_6d6d, contact_datas_6d6d);
}
BENCHMARK_DEFINE_F(CGFixture, CONSTRAINT_DYNAMICS_DERIVATIVES)(benchmark::State & st)
{
  pinocchio::initConstraintDynamics(model, data, CONTACT_MODELS_6D6D);
  for (auto _ : st)
  {
    constraintDynamicsDerivativeCall(model, data, q, v, a, CONTACT_MODELS_6D6D, CONTACT_DATAS_6D6D);
  }
}
BENCHMARK_REGISTER_F(CGFixture, CONSTRAINT_DYNAMICS_DERIVATIVES)->Apply(CustomArguments);

// CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN

BENCHMARK_DEFINE_F(CGFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN->evalFunction(q, v, a);
  }
}
BENCHMARK_REGISTER_F(CGFixture, CONSTRAINT_DYNAMICS_DERIVATIVES_CODE_GEN)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(CGFixture::GlobalSetUp);
