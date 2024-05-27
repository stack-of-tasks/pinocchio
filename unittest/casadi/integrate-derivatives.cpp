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

BOOST_AUTO_TEST_CASE(test_integrate)
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
  TangentVector a(TangentVector::Random(model.nv));

  typedef ADModel::ConfigVectorType ConfigVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  pinocchio::rnea(model, data, q, v, a);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  casadi::SX cs_v_int = casadi::SX::sym("v_inc", model.nv);

  ConfigVectorAD q_ad(model.nq), v_int_ad(model.nv), q_int_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);
  v_int_ad =
    Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_v_int).data(), model.nv, 1);

  pinocchio::integrate(ad_model, q_ad, v_int_ad, q_int_ad);
  casadi::SX cs_q_int(model.nq, 1);
  pinocchio::casadi::copy(q_int_ad, cs_q_int);

  casadi::Function eval_integrate(
    "eval_integrate", casadi::SXVector{cs_q, cs_v_int}, casadi::SXVector{cs_q_int});
  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_int_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_int_vec.data(), model.nv, 1).setZero();
  casadi::DM q_int_res = eval_integrate(casadi::DMVector{q_vec, v_int_vec})[0];

  Data::ConfigVectorType q_int_vec = Eigen::Map<Data::TangentVectorType>(
    static_cast<std::vector<double>>(q_int_res).data(), model.nq, 1);

  ConfigVector q_plus(model.nq);
  pinocchio::integrate(model, q, TangentVector::Zero(model.nv), q_plus);

  BOOST_CHECK(q_plus.isApprox(q_int_vec));
}

BOOST_AUTO_TEST_SUITE_END()
