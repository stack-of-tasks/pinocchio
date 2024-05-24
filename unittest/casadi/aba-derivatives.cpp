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

BOOST_AUTO_TEST_CASE(test_aba_derivatives_casadi_algo)
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
  pinocchio::Data data(model);

  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector tau(TangentVector::Random(model.nv));

  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
  pinocchio::computeABADerivatives(model, data, q, v, tau);
  data.Minv.triangularView<Eigen::StrictlyLower>() =
    data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::casadi::AutoDiffABADerivatives<Scalar> ad_casadi(model);
  ad_casadi.initLib();
  ad_casadi.loadLib();

  ad_casadi.evalFunction(q, v, tau);

  BOOST_CHECK(ad_casadi.ddq.isApprox(data.ddq));
  BOOST_CHECK(ad_casadi.ddq_dq.isApprox(data.ddq_dq));
  BOOST_CHECK(ad_casadi.ddq_dv.isApprox(data.ddq_dv));
  BOOST_CHECK(ad_casadi.ddq_dtau.isApprox(data.Minv));
}

BOOST_AUTO_TEST_SUITE_END()
