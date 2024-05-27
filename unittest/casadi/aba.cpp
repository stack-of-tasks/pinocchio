//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <casadi/casadi.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_aba)
{
  typedef double Scalar;
  typedef casadi::SX ADScalar;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  typedef Model::ConfigVectorType ConfigVector;
  typedef Model::TangentVectorType TangentVector;
  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector tau(TangentVector::Random(model.nv));

  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  casadi::SX cs_v_int = casadi::SX::sym("v_inc", model.nv);
  ConfigVectorAD q_ad(model.nq), v_int_ad(model.nv), q_int_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
  v_int_ad =
    Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_v_int).data(), model.nv, 1);

  pinocchio::integrate(ad_model, q_ad, v_int_ad, q_int_ad);
  casadi::SX cs_q_int(model.nq, 1);
  pinocchio::casadi::copy(q_int_ad, cs_q_int);
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_int_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_int_vec.data(), model.nv, 1).setZero();

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);

  casadi::SX cs_tau = casadi::SX::sym("tau", model.nv);
  TangentVectorAD tau_ad(model.nv);
  tau_ad =
    Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

  // ABA
  pinocchio::aba(ad_model, ad_data, q_int_ad, v_ad, tau_ad, pinocchio::Convention::WORLD);
  casadi::SX cs_ddq(model.nv, 1);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    cs_ddq(k) = ad_data.ddq[k];
  casadi::Function eval_aba(
    "eval_aba", casadi::SXVector{cs_q, cs_v_int, cs_v, cs_tau}, casadi::SXVector{cs_ddq});

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(), model.nv, 1) = v;

  std::vector<double> tau_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(tau_vec.data(), model.nv, 1) = tau;

  casadi::DM ddq_res = eval_aba(casadi::DMVector{q_vec, v_int_vec, v_vec, tau_vec})[0];
  Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(
    static_cast<std::vector<double>>(ddq_res).data(), model.nv, 1);

  BOOST_CHECK(ddq_mat.isApprox(data.ddq));

  // compute references
  Data::MatrixXs ddq_dq_ref(model.nv, model.nv), ddq_dv_ref(model.nv, model.nv),
    ddq_dtau_ref(model.nv, model.nv);
  ddq_dq_ref.setZero();
  ddq_dv_ref.setZero();
  ddq_dtau_ref.setZero();

  pinocchio::computeABADerivatives(model, data, q, v, tau, ddq_dq_ref, ddq_dv_ref, ddq_dtau_ref);
  ddq_dtau_ref.triangularView<Eigen::StrictlyLower>() =
    ddq_dtau_ref.transpose().triangularView<Eigen::StrictlyLower>();

  // check with respect to q+dq
  casadi::SX ddq_dq = jacobian(cs_ddq, cs_v_int);
  casadi::Function eval_ddq_dq(
    "eval_ddq_dq", casadi::SXVector{cs_q, cs_v_int, cs_v, cs_tau}, casadi::SXVector{ddq_dq});

  casadi::DM ddq_dq_res = eval_ddq_dq(casadi::DMVector{q_vec, v_int_vec, v_vec, tau_vec})[0];
  std::vector<double> ddq_dq_vec(static_cast<std::vector<double>>(ddq_dq_res));
  BOOST_CHECK(
    Eigen::Map<Data::MatrixXs>(ddq_dq_vec.data(), model.nv, model.nv).isApprox(ddq_dq_ref));

  // check with respect to v+dv
  casadi::SX ddq_dv = jacobian(cs_ddq, cs_v);
  casadi::Function eval_ddq_dv(
    "eval_ddq_dv", casadi::SXVector{cs_q, cs_v_int, cs_v, cs_tau}, casadi::SXVector{ddq_dv});

  casadi::DM ddq_dv_res = eval_ddq_dv(casadi::DMVector{q_vec, v_int_vec, v_vec, tau_vec})[0];
  std::vector<double> ddq_dv_vec(static_cast<std::vector<double>>(ddq_dv_res));
  BOOST_CHECK(
    Eigen::Map<Data::MatrixXs>(ddq_dv_vec.data(), model.nv, model.nv).isApprox(ddq_dv_ref));

  // check with respect to a+da
  casadi::SX ddq_dtau = jacobian(cs_ddq, cs_tau);
  casadi::Function eval_ddq_da(
    "eval_ddq_da", casadi::SXVector{cs_q, cs_v_int, cs_v, cs_tau}, casadi::SXVector{ddq_dtau});

  casadi::DM ddq_dtau_res = eval_ddq_da(casadi::DMVector{q_vec, v_int_vec, v_vec, tau_vec})[0];
  std::vector<double> ddq_dtau_vec(static_cast<std::vector<double>>(ddq_dtau_res));
  BOOST_CHECK(
    Eigen::Map<Data::MatrixXs>(ddq_dtau_vec.data(), model.nv, model.nv).isApprox(ddq_dtau_ref));

  // call ABA derivatives in Casadi
  casadi::SX cs_ddq_dq(model.nv, model.nv);
  casadi::SX cs_ddq_dv(model.nv, model.nv);
  casadi::SX cs_ddq_dtau(model.nv, model.nv);

  computeABADerivatives(ad_model, ad_data, q_ad, v_ad, tau_ad);
  ad_data.Minv.triangularView<Eigen::StrictlyLower>() =
    ad_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::casadi::copy(ad_data.ddq_dq, cs_ddq_dq);
  pinocchio::casadi::copy(ad_data.ddq_dv, cs_ddq_dv);
  pinocchio::casadi::copy(ad_data.Minv, cs_ddq_dtau);

  casadi::Function eval_aba_derivatives_dq(
    "eval_aba_derivatives_dq", casadi::SXVector{cs_q, cs_v, cs_tau}, casadi::SXVector{cs_ddq_dq});

  casadi::DM ddq_dq_res_direct =
    eval_aba_derivatives_dq(casadi::DMVector{q_vec, v_vec, tau_vec})[0];
  Data::MatrixXs ddq_dq_res_direct_map = Eigen::Map<Data::MatrixXs>(
    static_cast<std::vector<double>>(ddq_dq_res_direct).data(), model.nv, model.nv);
  BOOST_CHECK(ddq_dq_ref.isApprox(ddq_dq_res_direct_map));

  casadi::Function eval_aba_derivatives_dv(
    "eval_aba_derivatives_dv", casadi::SXVector{cs_q, cs_v, cs_tau}, casadi::SXVector{cs_ddq_dv});

  casadi::DM ddq_dv_res_direct =
    eval_aba_derivatives_dv(casadi::DMVector{q_vec, v_vec, tau_vec})[0];
  Data::MatrixXs ddq_dv_res_direct_map = Eigen::Map<Data::MatrixXs>(
    static_cast<std::vector<double>>(ddq_dv_res_direct).data(), model.nv, model.nv);
  BOOST_CHECK(ddq_dv_ref.isApprox(ddq_dv_res_direct_map));

  casadi::Function eval_aba_derivatives_dtau(
    "eval_aba_derivatives_dtau", casadi::SXVector{cs_q, cs_v, cs_tau},
    casadi::SXVector{cs_ddq_dtau});

  casadi::DM ddq_dtau_res_direct =
    eval_aba_derivatives_dtau(casadi::DMVector{q_vec, v_vec, tau_vec})[0];
  Data::MatrixXs ddq_dtau_res_direct_map = Eigen::Map<Data::MatrixXs>(
    static_cast<std::vector<double>>(ddq_dtau_res_direct).data(), model.nv, model.nv);
  BOOST_CHECK(ddq_dtau_ref.isApprox(ddq_dtau_res_direct_map));
}

BOOST_AUTO_TEST_CASE(test_aba_casadi_algo)
{
  typedef double Scalar;
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef pinocchio::DataTpl<Scalar> Data;
  typedef typename Model::ConfigVectorType ConfigVector;
  typedef typename Model::TangentVectorType TangentVector;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector tau(TangentVector::Random(model.nv));

  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
  pinocchio::computeABADerivatives(model, data, q, v, tau);
  data.Minv.triangularView<Eigen::StrictlyLower>() =
    data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::casadi::AutoDiffABA<Scalar> ad_casadi(model);
  ad_casadi.initLib();
  ad_casadi.loadLib();

  ad_casadi.evalFunction(q, v, tau);
  ad_casadi.evalJacobian(q, v, tau);
  BOOST_CHECK(ad_casadi.ddq.isApprox(data.ddq));
  BOOST_CHECK(ad_casadi.ddq_dq.isApprox(data.ddq_dq));
  BOOST_CHECK(ad_casadi.ddq_dv.isApprox(data.ddq_dv));
  BOOST_CHECK(ad_casadi.ddq_dtau.isApprox(data.Minv));
}

BOOST_AUTO_TEST_SUITE_END()
