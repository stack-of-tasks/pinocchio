//
// Copyright (c) 2018-2024 INRIA
//

#include "pinocchio/multibody.hpp"

#include "pinocchio/algorithm/aba-second-order-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

namespace
{
  inline void symmetrizeUpper(Eigen::MatrixXd & m)
  {
    m.triangularView<Eigen::StrictlyLower>() = m.transpose().triangularView<Eigen::StrictlyLower>();
  }
} // namespace

BOOST_AUTO_TEST_CASE(test_aba_derivatives_SO)
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
  VectorXd tau(VectorXd::Random(model.nv));

  // 1) Analytical second-order ABA derivatives.
  Data::Tensor3x d2ddq_dqdq(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dvdv(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dqdv(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dtaudq(model.nv, model.nv, model.nv);
  d2ddq_dqdq.setZero();
  d2ddq_dvdv.setZero();
  d2ddq_dqdv.setZero();
  d2ddq_dtaudq.setZero();

  ComputeABASecondOrderDerivatives(
    model, data, q, v, tau, d2ddq_dqdq, d2ddq_dvdv, d2ddq_dqdv, d2ddq_dtaudq);

  // 2) Finite-difference reference, computed by perturbing q and v and re-running
  //    the first-order ABA derivatives.
  Data::Tensor3x d2ddq_dqdq_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dvdv_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dqdv_fd(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dtaudq_fd(model.nv, model.nv, model.nv);
  d2ddq_dqdq_fd.setZero();
  d2ddq_dvdv_fd.setZero();
  d2ddq_dqdv_fd.setZero();
  d2ddq_dtaudq_fd.setZero();

  MatrixXd ddq_dq_base(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd ddq_dv_base(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd ddq_dtau_base(MatrixXd::Zero(model.nv, model.nv));

  computeABADerivatives(model, data, q, v, tau, ddq_dq_base, ddq_dv_base, ddq_dtau_base);
  symmetrizeUpper(ddq_dtau_base);

  MatrixXd ddq_dq_plus(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd ddq_dv_plus(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd ddq_dtau_plus(MatrixXd::Zero(model.nv, model.nv));

  MatrixXd temp_dq(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd temp_dv(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd temp_dtau(MatrixXd::Zero(model.nv, model.nv));

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd v_plus(model.nv);

  const double alpha = 1e-7;

  // 2a) Perturb q (configuration). This populates d2ddq_dqdq_fd and
  //     d2ddq_dtaudq_fd.
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeABADerivatives(model, data_fd, q_plus, v, tau, ddq_dq_plus, ddq_dv_plus, ddq_dtau_plus);
    symmetrizeUpper(ddq_dtau_plus);

    temp_dq = (ddq_dq_plus - ddq_dq_base) / alpha;
    temp_dtau = (ddq_dtau_plus - ddq_dtau_base) / alpha;

    for (int ii = 0; ii < model.nv; ++ii)
    {
      for (int jj = 0; jj < model.nv; ++jj)
      {
        d2ddq_dqdq_fd(jj, ii, k) = temp_dq(jj, ii);
        d2ddq_dtaudq_fd(jj, ii, k) = temp_dtau(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }

  // 2b) Perturb v. This populates d2ddq_dvdv_fd and d2ddq_dqdv_fd.
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    v_plus = v + v_eps;
    computeABADerivatives(model, data_fd, q, v_plus, tau, ddq_dq_plus, ddq_dv_plus, ddq_dtau_plus);

    temp_dv = (ddq_dv_plus - ddq_dv_base) / alpha;
    temp_dq = (ddq_dq_plus - ddq_dq_base) / alpha;

    for (int ii = 0; ii < model.nv; ++ii)
    {
      for (int jj = 0; jj < model.nv; ++jj)
      {
        d2ddq_dvdv_fd(jj, ii, k) = temp_dv(jj, ii);
        d2ddq_dqdv_fd(jj, ii, k) = temp_dq(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }

  // 3) Compare analytic against finite-difference reference.
  Map<VectorXd> mqq(d2ddq_dqdq.data(), d2ddq_dqdq.size());
  Map<VectorXd> mvv(d2ddq_dvdv.data(), d2ddq_dvdv.size());
  Map<VectorXd> mqv(d2ddq_dqdv.data(), d2ddq_dqdv.size());
  Map<VectorXd> mtq(d2ddq_dtaudq.data(), d2ddq_dtaudq.size());

  Map<VectorXd> mqq_fd(d2ddq_dqdq_fd.data(), d2ddq_dqdq_fd.size());
  Map<VectorXd> mvv_fd(d2ddq_dvdv_fd.data(), d2ddq_dvdv_fd.size());
  Map<VectorXd> mqv_fd(d2ddq_dqdv_fd.data(), d2ddq_dqdv_fd.size());
  Map<VectorXd> mtq_fd(d2ddq_dtaudq_fd.data(), d2ddq_dtaudq_fd.size());

  BOOST_CHECK(mqq.isApprox(mqq_fd, sqrt(alpha)));
  BOOST_CHECK(mvv.isApprox(mvv_fd, sqrt(alpha)));
  BOOST_CHECK(mqv.isApprox(mqv_fd, sqrt(alpha)));
  BOOST_CHECK(mtq.isApprox(mtq_fd, sqrt(alpha)));

  // 4) The Data-member overload should produce the same result as the
  //    explicit-output overload.
  Data data2(model);
  ComputeABASecondOrderDerivatives(model, data2, q, v, tau);

  Map<VectorXd> mqq2(data2.d2ddq_dqdq.data(), data2.d2ddq_dqdq.size());
  Map<VectorXd> mvv2(data2.d2ddq_dvdv.data(), data2.d2ddq_dvdv.size());
  Map<VectorXd> mqv2(data2.d2ddq_dqdv.data(), data2.d2ddq_dqdv.size());
  Map<VectorXd> mtq2(data2.d2ddq_dtaudq.data(), data2.d2ddq_dtaudq.size());

  BOOST_CHECK(mqq.isApprox(mqq2));
  BOOST_CHECK(mvv.isApprox(mvv2));
  BOOST_CHECK(mqv.isApprox(mqv2));
  BOOST_CHECK(mtq.isApprox(mtq2));
}

BOOST_AUTO_TEST_SUITE_END()
