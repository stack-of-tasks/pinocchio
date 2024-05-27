//
// Copyright (c) 2017-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rnea_derivatives_SO)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  // check with only q non-zero
  Data::Tensor3x dtau2_dq(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dv(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dqdv(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dadq(model.nv, model.nv, model.nv);
  dtau2_dq.setZero();
  dtau2_dv.setZero();
  dtau2_dqdv.setZero();
  dtau2_dadq.setZero();

  ComputeRNEASecondOrderDerivatives(
    model, data, q, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv), dtau2_dq, dtau2_dv,
    dtau2_dqdv, dtau2_dadq);

  Data::Tensor3x dtau2_dq_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dv_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dqdv_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dadq_fd(model.nv, model.nv, model.nv);
  dtau2_dq_fd.setZero();
  dtau2_dv_fd.setZero();
  dtau2_dqdv_fd.setZero();
  dtau2_dadq_fd.setZero();

  MatrixXd drnea_dq_plus(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd drnea_dv_plus(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd drnea_da_plus(MatrixXd::Zero(model.nv, model.nv));

  MatrixXd temp1(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd temp2(MatrixXd::Zero(model.nv, model.nv));

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd v_plus(model.nv);

  MatrixXd rnea_partial_dq(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd rnea_partial_dv(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd rnea_partial_da(MatrixXd::Zero(model.nv, model.nv));

  computeRNEADerivatives(
    model, data, q, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv), rnea_partial_dq,
    rnea_partial_dv, rnea_partial_da);

  const double alpha = 1e-7;

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(
      model, data_fd, q_plus, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv), drnea_dq_plus,
      drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    for (int ii = 0; ii < model.nv; ii++)
    {
      for (int jj = 0; jj < model.nv; jj++)
      {
        dtau2_dq_fd(jj, ii, k) = temp1(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }

  Map<VectorXd> mq(dtau2_dq.data(), dtau2_dq.size());
  Map<VectorXd> mq_fd(dtau2_dq_fd.data(), dtau2_dq_fd.size());
  BOOST_CHECK(mq.isApprox(mq_fd, sqrt(alpha)));

  // Check with q and a non zero
  dtau2_dq.setZero();
  dtau2_dv.setZero();
  dtau2_dqdv.setZero();
  dtau2_dadq.setZero();
  ComputeRNEASecondOrderDerivatives(
    model, data, q, VectorXd::Zero(model.nv), a, dtau2_dq, dtau2_dv, dtau2_dqdv, dtau2_dadq);

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  rnea_partial_da.setZero();

  dtau2_dq_fd.setZero();
  dtau2_dadq_fd.setZero();
  drnea_dq_plus.setZero();
  drnea_dv_plus.setZero();
  drnea_da_plus.setZero();

  computeRNEADerivatives(
    model, data, q, VectorXd::Zero(model.nv), a, rnea_partial_dq, rnea_partial_dv, rnea_partial_da);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(
      model, data_fd, q_plus, VectorXd::Zero(model.nv), a, drnea_dq_plus, drnea_dv_plus,
      drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    temp2 = (drnea_da_plus - rnea_partial_da) / alpha;
    temp2.triangularView<Eigen::StrictlyLower>() =
      temp2.transpose().triangularView<Eigen::StrictlyLower>();
    for (int ii = 0; ii < model.nv; ii++)
    {
      for (int jj = 0; jj < model.nv; jj++)
      {
        dtau2_dq_fd(jj, ii, k) = temp1(jj, ii);
        dtau2_dadq_fd(jj, ii, k) = temp2(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }
  Map<VectorXd> maq(dtau2_dadq.data(), dtau2_dadq.size());
  Map<VectorXd> maq_fd(dtau2_dadq_fd.data(), dtau2_dadq_fd.size());

  BOOST_CHECK(mq.isApprox(mq_fd, sqrt(alpha)));
  BOOST_CHECK(maq.isApprox(maq_fd, sqrt(alpha)));

  // Check with q,v and a non zero
  dtau2_dq.setZero();
  dtau2_dv.setZero();
  dtau2_dqdv.setZero();
  dtau2_dadq.setZero();
  ComputeRNEASecondOrderDerivatives(
    model, data, q, v, a, dtau2_dq, dtau2_dv, dtau2_dqdv, dtau2_dadq);

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  rnea_partial_da.setZero();
  computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq, rnea_partial_dv, rnea_partial_da);

  dtau2_dq_fd.setZero();
  dtau2_dadq_fd.setZero();
  drnea_dq_plus.setZero();
  drnea_dv_plus.setZero();
  drnea_da_plus.setZero();

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(
      model, data_fd, q_plus, v, a, drnea_dq_plus, drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    temp2 = (drnea_da_plus - rnea_partial_da) / alpha;
    temp2.triangularView<Eigen::StrictlyLower>() =
      temp2.transpose().triangularView<Eigen::StrictlyLower>();
    for (int ii = 0; ii < model.nv; ii++)
    {
      for (int jj = 0; jj < model.nv; jj++)
      {
        dtau2_dq_fd(jj, ii, k) = temp1(jj, ii);
        dtau2_dadq_fd(jj, ii, k) = temp2(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }
  dtau2_dv_fd.setZero();
  dtau2_dqdv_fd.setZero();
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    v_plus = v + v_eps;
    computeRNEADerivatives(
      model, data_fd, q, v_plus, a, drnea_dq_plus, drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dv_plus - rnea_partial_dv) / alpha;
    temp2 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    for (int ii = 0; ii < model.nv; ii++)
    {
      for (int jj = 0; jj < model.nv; jj++)
      {
        dtau2_dv_fd(jj, ii, k) = temp1(jj, ii);
        dtau2_dqdv_fd(jj, ii, k) = temp2(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }
  Map<VectorXd> mv(dtau2_dv.data(), dtau2_dv.size());
  Map<VectorXd> mv_fd(dtau2_dv_fd.data(), dtau2_dv_fd.size());

  Map<VectorXd> mqv(dtau2_dqdv.data(), dtau2_dqdv.size());
  Map<VectorXd> mqv_fd(dtau2_dqdv_fd.data(), dtau2_dqdv_fd.size());

  BOOST_CHECK(mq.isApprox(mq_fd, sqrt(alpha)));
  BOOST_CHECK(maq.isApprox(maq_fd, sqrt(alpha)));
  BOOST_CHECK(mv.isApprox(mv_fd, sqrt(alpha)));
  BOOST_CHECK(mqv.isApprox(mqv_fd, sqrt(alpha)));

  Data data2(model);
  ComputeRNEASecondOrderDerivatives(model, data2, q, v, a);

  Map<VectorXd> mq2(data2.d2tau_dqdq.data(), (data2.d2tau_dqdq).size());
  Map<VectorXd> mv2(data2.d2tau_dvdv.data(), (data2.d2tau_dvdv).size());
  Map<VectorXd> mqv2(data2.d2tau_dqdv.data(), (data2.d2tau_dqdv).size());
  Map<VectorXd> maq2(data2.d2tau_dadq.data(), (data2.d2tau_dadq).size());

  BOOST_CHECK(mq.isApprox(mq2, sqrt(alpha)));
  BOOST_CHECK(mv.isApprox(mv2, sqrt(alpha)));
  BOOST_CHECK(mqv.isApprox(mqv2, sqrt(alpha)));
  BOOST_CHECK(maq.isApprox(maq2, sqrt(alpha)));
}

BOOST_AUTO_TEST_SUITE_END()
