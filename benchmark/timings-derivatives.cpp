//
// Copyright (c) 2018-2025 CNRS INRIA
//

#include "model-fixture.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>

typedef pinocchio::Data::Tensor3x Tensor3x;

template<typename Matrix1, typename Matrix2, typename Matrix3>
void rnea_fd(
  const pinocchio::Model & model,
  pinocchio::Data & data_fd,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const Eigen::MatrixBase<Matrix1> & _drnea_dq,
  const Eigen::MatrixBase<Matrix2> & _drnea_dv,
  const Eigen::MatrixBase<Matrix3> & _drnea_da)
{
  Matrix1 & drnea_dq = PINOCCHIO_EIGEN_CONST_CAST(Matrix1, _drnea_dq);
  Matrix2 & drnea_dv = PINOCCHIO_EIGEN_CONST_CAST(Matrix2, _drnea_dv);
  Matrix3 & drnea_da = PINOCCHIO_EIGEN_CONST_CAST(Matrix3, _drnea_da);

  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;

  VectorXd tau0 = rnea(model, data_fd, q, v, a);

  // dRNEA/dq
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    integrate(model, q, v_eps, q_plus);
    tau_plus = rnea(model, data_fd, q_plus, v, a);

    drnea_dq.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }

  // dRNEA/dv
  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    tau_plus = rnea(model, data_fd, q, v_plus, a);

    drnea_dv.col(k) = (tau_plus - tau0) / alpha;
    v_plus[k] -= alpha;
  }

  // dRNEA/da
  drnea_da = pinocchio::crba(model, data_fd, q, pinocchio::Convention::WORLD);
  drnea_da.template triangularView<Eigen::StrictlyLower>() =
    drnea_da.transpose().template triangularView<Eigen::StrictlyLower>();
}

void aba_fd(
  const pinocchio::Model & model,
  pinocchio::Data & data_fd,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  Eigen::MatrixXd & daba_dq,
  Eigen::MatrixXd & daba_dv,
  pinocchio::Data::RowMatrixXs & daba_dtau)
{
  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;

  VectorXd a0 = pinocchio::aba(model, data_fd, q, v, tau, pinocchio::Convention::LOCAL);

  // dABA/dq
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    a_plus = pinocchio::aba(model, data_fd, q_plus, v, tau, pinocchio::Convention::LOCAL);

    daba_dq.col(k) = (a_plus - a0) / alpha;
    v_eps[k] -= alpha;
  }

  // dABA/dv
  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = pinocchio::aba(model, data_fd, q, v_plus, tau, pinocchio::Convention::LOCAL);

    daba_dv.col(k) = (a_plus - a0) / alpha;
    v_plus[k] -= alpha;
  }

  // dABA/dtau
  daba_dtau = computeMinverse(model, data_fd, q);
}

struct DerivativesFixture : ModelFixture
{
  void SetUp(benchmark::State & st)
  {
    ModelFixture::SetUp(st);
    drnea_dq.setZero(model.nv, model.nv);
    drnea_dv.setZero(model.nv, model.nv);
    drnea_da.setZero(model.nv, model.nv);

    daba_dq.setZero(model.nv, model.nv);
    daba_dv.setZero(model.nv, model.nv);
    daba_dtau.setZero(model.nv, model.nv);

    dtau2_dq = Tensor3x(model.nv, model.nv, model.nv);
    dtau2_dv = Tensor3x(model.nv, model.nv, model.nv);
    dtau2_dqv = Tensor3x(model.nv, model.nv, model.nv);
    dtau_dadq = Tensor3x(model.nv, model.nv, model.nv);
    dtau2_dq.setZero();
    dtau2_dv.setZero();
    dtau2_dqv.setZero();
    dtau_dadq.setZero();
  }

  PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) drnea_dq;
  PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) drnea_dv;
  Eigen::MatrixXd drnea_da;

  Eigen::MatrixXd daba_dq;
  Eigen::MatrixXd daba_dv;
  pinocchio::Data::RowMatrixXs daba_dtau;

  Tensor3x dtau2_dq;
  Tensor3x dtau2_dv;
  Tensor3x dtau2_dqv;
  Tensor3x dtau_dadq;

  void TearDown(benchmark::State & st)
  {
    ModelFixture::TearDown(st);
  }

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    ModelFixture::GlobalSetUp(extra_args);
  }
};

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// FORWARD_KINEMATICS_Q_V_A

PINOCCHIO_DONT_INLINE static void forwardKinematicsQVACall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::forwardKinematics(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(DerivativesFixture, FORWARD_KINEMATICS_Q_V_A)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQVACall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, FORWARD_KINEMATICS_Q_V_A)->Apply(CustomArguments);

// FORWARD_KINEMATICS_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeForwardKinematicsDerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(DerivativesFixture, FORWARD_KINEMATICS_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeForwardKinematicsDerivativesCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, FORWARD_KINEMATICS_DERIVATIVES)->Apply(CustomArguments);

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
BENCHMARK_DEFINE_F(DerivativesFixture, RNEA)(benchmark::State & st)
{
  for (auto _ : st)
  {
    rneaCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, RNEA)->Apply(CustomArguments);

// COMPUTE_RNEA_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeRNEADerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) & drnea_dq,
  const PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) & drnea_dv,
  const Eigen::MatrixXd & drnea_da)
{
  pinocchio::computeRNEADerivatives(model, data, q, v, a, drnea_dq, drnea_dv, drnea_da);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_RNEA_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeRNEADerivativesCall(model, data, q, v, a, drnea_dq, drnea_dv, drnea_da);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_RNEA_DERIVATIVES)->Apply(CustomArguments);

// COMUTE_RNEA_SECOND_ORDER_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeRNEASecondOrderDerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const Tensor3x & dtau2_dq,
  const Tensor3x & dtau2_dv,
  const Tensor3x & dtau2_dqv,
  const Tensor3x & dtau_dadq)
{
  ComputeRNEASecondOrderDerivatives(model, data, q, v, a, dtau2_dq, dtau2_dv, dtau2_dqv, dtau_dadq);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMUTE_RNEA_SECOND_ORDER_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeRNEASecondOrderDerivativesCall(
      model, data, q, v, a, dtau2_dq, dtau2_dv, dtau2_dqv, dtau_dadq);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMUTE_RNEA_SECOND_ORDER_DERIVATIVES)
  ->Apply(CustomArguments);

// COMPUTE_RNEA_FD_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeRNEAFDDerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) & drnea_dq,
  const PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(Eigen::MatrixXd) & drnea_dv,
  const Eigen::MatrixXd & drnea_da)
{
  rnea_fd(model, data, q, v, a, drnea_dq, drnea_dv, drnea_da);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_RNEA_FD_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeRNEAFDDerivativesCall(model, data, q, v, a, drnea_dq, drnea_dv, drnea_da);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_RNEA_FD_DERIVATIVES)->Apply(CustomArguments);

// ABA_LOCAL

PINOCCHIO_DONT_INLINE static void abaLocalCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau)
{
  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::LOCAL);
}
BENCHMARK_DEFINE_F(DerivativesFixture, ABA_LOCAL)(benchmark::State & st)
{
  for (auto _ : st)
  {
    abaLocalCall(model, data, q, v, tau);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, ABA_LOCAL)->Apply(CustomArguments);

// COMPUTE_ABA_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeABADerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  const Eigen::MatrixXd & daba_dq,
  const Eigen::MatrixXd & daba_dv,
  const pinocchio::Data::RowMatrixXs & daba_dtau)
{
  computeABADerivatives(model, data, q, v, tau, daba_dq, daba_dv, daba_dtau);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_ABA_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeABADerivativesCall(model, data, q, v, tau, daba_dq, daba_dv, daba_dtau);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_ABA_DERIVATIVES)->Apply(CustomArguments);

// COMPUTE_ABA_DERIVATIVES_NO_Q_V_TAU

PINOCCHIO_DONT_INLINE static void computeABADerivativesNoQVTauCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::MatrixXd & daba_dq,
  const Eigen::MatrixXd & daba_dv,
  const pinocchio::Data::RowMatrixXs & daba_dtau)
{
  computeABADerivatives(model, data, daba_dq, daba_dv, daba_dtau);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_ABA_DERIVATIVES_NO_Q_V_TAU)(benchmark::State & st)
{
  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    computeABADerivativesNoQVTauCall(model, data, daba_dq, daba_dv, daba_dtau);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_ABA_DERIVATIVES_NO_Q_V_TAU)
  ->Apply(CustomArguments);

// COMPUTE_ABA_FD_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeABAFDDerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  Eigen::MatrixXd & daba_dq,
  Eigen::MatrixXd & daba_dv,
  pinocchio::Data::RowMatrixXs & daba_dtau)
{
  aba_fd(model, data, q, v, tau, daba_dq, daba_dv, daba_dtau);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_ABA_FD_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeABAFDDerivativesCall(model, data, q, v, tau, daba_dq, daba_dv, daba_dtau);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_ABA_FD_DERIVATIVES)->Apply(CustomArguments);

// COMPUTE_M_INVERSE_Q

PINOCCHIO_DONT_INLINE static void computeMinverseQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeMinverse(model, data, q);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_M_INVERSE_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeMinverseQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_M_INVERSE_Q)->Apply(CustomArguments);

// COMPUTE_M_INVERSE

PINOCCHIO_DONT_INLINE static void
computeMinverseCall(const pinocchio::Model & model, pinocchio::Data & data)
{
  pinocchio::computeMinverse(model, data);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_M_INVERSE)(benchmark::State & st)
{
  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
  for (auto _ : st)
  {
    computeMinverseCall(model, data);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_M_INVERSE)->Apply(CustomArguments);

// COMPUTE_CHOLESKY_M_INVERSE

PINOCCHIO_DONT_INLINE static void computeCholeskyMinverseCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  Eigen::MatrixXd & Minv)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  pinocchio::cholesky::decompose(model, data);
  pinocchio::cholesky::computeMinv(model, data, Minv);
}
BENCHMARK_DEFINE_F(DerivativesFixture, COMPUTE_CHOLESKY_M_INVERSE)(benchmark::State & st)
{
  Eigen::MatrixXd Minv(model.nv, model.nv);
  Minv.setZero();
  for (auto _ : st)
  {
    computeCholeskyMinverseCall(model, data, q, Minv);
  }
}
BENCHMARK_REGISTER_F(DerivativesFixture, COMPUTE_CHOLESKY_M_INVERSE)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(DerivativesFixture::GlobalSetUp);
