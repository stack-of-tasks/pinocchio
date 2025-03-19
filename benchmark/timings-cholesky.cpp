//
// Copyright (c) 2018-2025 CNRS
//

#include "model-fixture.hpp"

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

struct CholeskyFixture : ModelFixture
{
  void SetUp(benchmark::State & st)
  {
    ModelFixture::SetUp(st);
    lhs.setZero(model.nv);
    rhs.setRandom(model.nv);
    A.setZero(model.nv, model.nv);
    B.setRandom(model.nv, model.nv);
    Minv.setZero(model.nv, model.nv);
  }

  void TearDown(benchmark::State & st)
  {
    ModelFixture::TearDown(st);
  }

  Eigen::VectorXd lhs;
  Eigen::VectorXd rhs;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd Minv;

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    ModelFixture::GlobalSetUp(extra_args);
  }
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// choleskyDecompose

PINOCCHIO_DONT_INLINE static void
choleskyDecomposeCall(const pinocchio::Model & model, pinocchio::Data & data)
{
  pinocchio::cholesky::decompose(model, data);
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_DECOMPOSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    choleskyDecomposeCall(model, data);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_DECOMPOSE)->Apply(CustomArguments);

// Dense choleskyDecompose

PINOCCHIO_DONT_INLINE static void
denseCholeksyDecomposeCall(Eigen::LDLT<Eigen::MatrixXd> & M_ldlt, const pinocchio::Data & data)
{
  M_ldlt.compute(data.M);
}
BENCHMARK_DEFINE_F(CholeskyFixture, DENSE_CHOLESKY_DECOMPOSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::LDLT<Eigen::MatrixXd> M_ldlt(data.M);
  for (auto _ : st)
  {
    denseCholeksyDecomposeCall(M_ldlt, data);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, DENSE_CHOLESKY_DECOMPOSE)->Apply(CustomArguments);

// CHOLESKY_SOLVE

PINOCCHIO_DONT_INLINE static void choleskySolveCall(
  const pinocchio::Model & model, const pinocchio::Data & data, Eigen::VectorXd & rhs)
{
  pinocchio::cholesky::solve(model, data, rhs);
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_SOLVE)(benchmark::State & st)
{
  for (auto _ : st)
  {
    choleskySolveCall(model, data, rhs);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_SOLVE)->Apply(CustomArguments);

// CHOLESKY_UDUtv

PINOCCHIO_DONT_INLINE static void choleskyUDUtvCall(
  const pinocchio::Model & model, const pinocchio::Data & data, Eigen::VectorXd & rhs)
{
  pinocchio::cholesky::UDUtv(model, data, rhs);
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_UDUtv)(benchmark::State & st)
{
  for (auto _ : st)
  {
    choleskyUDUtvCall(model, data, rhs);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_UDUtv)->Apply(CustomArguments);

// CHOLESKY_COMPUTE_M_INV

PINOCCHIO_DONT_INLINE static void choleskyComputeMInvCall(
  const pinocchio::Model & model, const pinocchio::Data & data, Eigen::MatrixXd & Minv)
{
  pinocchio::cholesky::computeMinv(model, data, Minv);
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_COMPUTE_M_INV)(benchmark::State & st)
{
  for (auto _ : st)
  {
    choleskyComputeMInvCall(model, data, Minv);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_COMPUTE_M_INV)->Apply(CustomArguments);

// CHOLESKY_SOLVE_COLUMN

PINOCCHIO_DONT_INLINE static void choleskySolveColumnCall(
  const pinocchio::Model & model, const pinocchio::Data & data, Eigen::MatrixXd & Minv)
{
  pinocchio::cholesky::solve(model, data, Minv.col(10));
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_SOLVE_COLUMN)(benchmark::State & st)
{
  pinocchio::cholesky::computeMinv(model, data, Minv);
  for (auto _ : st)
  {
    choleskySolveColumnCall(model, data, Minv);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_SOLVE_COLUMN)->Apply(CustomArguments);

// CHOLESKY_M_INV_MULT_V

PINOCCHIO_DONT_INLINE static void
choleskyMInvMultV(const Eigen::MatrixXd & Minv, const Eigen::VectorXd & rhs, Eigen::VectorXd & lhs)
{
  lhs.noalias() = Minv * rhs;
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_M_INV_MULT_V)(benchmark::State & st)
{
  pinocchio::cholesky::computeMinv(model, data, Minv);
  for (auto _ : st)
  {
    choleskyMInvMultV(Minv, rhs, lhs);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_M_INV_MULT_V)->Apply(CustomArguments);

// CHOLESKY_M_INV_MULT_B

PINOCCHIO_DONT_INLINE static void
choleskyMInvMultB(const Eigen::MatrixXd & Minv, const Eigen::MatrixXd & B, Eigen::MatrixXd & A)
{
  A.noalias() = Minv * B;
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_M_INV_MULT_B)(benchmark::State & st)
{
  pinocchio::cholesky::computeMinv(model, data, Minv);
  for (auto _ : st)
  {
    choleskyMInvMultB(Minv, B, A);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_M_INV_MULT_B)->Apply(CustomArguments);

// CHOLESKY_DENSE_M_INVERSE

PINOCCHIO_DONT_INLINE static void
choleskyDenseMInverse(const Eigen::MatrixXd & M, Eigen::MatrixXd & A)
{
  A.noalias() = M.inverse();
}
BENCHMARK_DEFINE_F(CholeskyFixture, CHOLESKY_DENSE_M_INVERSE)(benchmark::State & st)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  for (auto _ : st)
  {
    choleskyDenseMInverse(data.M, A);
  }
}
BENCHMARK_REGISTER_F(CholeskyFixture, CHOLESKY_DENSE_M_INVERSE)->Apply(CustomArguments);

// computeMinverseQ

PINOCCHIO_DONT_INLINE static void computeMinverseQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeMinverse(model, data, q);
}
BENCHMARK_DEFINE_F(ModelFixture, COMPUTE_M_INVERSE_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeMinverseQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(ModelFixture, COMPUTE_M_INVERSE_Q)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(CholeskyFixture::GlobalSetUp);
