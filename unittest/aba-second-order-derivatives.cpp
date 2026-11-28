//
// Copyright (c) 2018-2024 INRIA
//

#include "pinocchio/multibody.hpp"

#include "pinocchio/algorithm/aba-second-order-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <stdexcept>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

namespace
{
  using namespace Eigen;
  using namespace pinocchio;

  inline Map<const VectorXd> asVector(const Data::Tensor3x & tensor)
  {
    return Map<const VectorXd>(tensor.data(), tensor.size());
  }

  // Compares the analytical second-order ABA derivatives against finite differences of the
  // first-order ABA derivatives. The finite-difference references are filled with the storage
  // order documented in ComputeABASecondOrderDerivatives: for d2ddq_dxdy, the elements of ddq vary
  // along the rows, those of x (inner derivative) along the columns and those of y (outer
  // derivative) along the pages, i.e. d2ddq_dxdy(i,j,k) = d/dy_k ( d ddq_i / dx_j ).
  void checkABASecondOrderDerivativesAgainstFiniteDifferences(Model & model)
  {
    Data data(model), data_fd(model);

    model.lowerPositionLimit.head<3>().fill(-1.);
    model.upperPositionLimit.head<3>().fill(1.);

    const VectorXd q = randomConfiguration(model);
    const VectorXd v(VectorXd::Random(model.nv));
    const VectorXd tau(VectorXd::Random(model.nv));

    // 1) Analytical second-order ABA derivatives. The outputs are filled with garbage on purpose:
    //    they must be fully overwritten by the algorithm.
    Data::Tensor3x d2ddq_dqdq(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dvdv(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dqdv(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dtaudq(model.nv, model.nv, model.nv);
    d2ddq_dqdq.setConstant(1.);
    d2ddq_dvdv.setConstant(2.);
    d2ddq_dqdv.setConstant(3.);
    d2ddq_dtaudq.setConstant(4.);

    ComputeABASecondOrderDerivatives(
      model, data, q, v, tau, d2ddq_dqdq, d2ddq_dvdv, d2ddq_dqdv, d2ddq_dtaudq);

    // 2) Finite-difference references, computed by perturbing q and v and re-running the
    //    first-order ABA derivatives.
    Data::Tensor3x d2ddq_dqdq_fd(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dvdv_fd(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dqdv_fd(model.nv, model.nv, model.nv);
    Data::Tensor3x d2ddq_dtaudq_fd(model.nv, model.nv, model.nv);

    MatrixXd ddq_dq_base(MatrixXd::Zero(model.nv, model.nv));
    MatrixXd ddq_dv_base(MatrixXd::Zero(model.nv, model.nv));
    MatrixXd ddq_dtau_base(MatrixXd::Zero(model.nv, model.nv));
    computeABADerivatives(model, data_fd, q, v, tau, ddq_dq_base, ddq_dv_base, ddq_dtau_base);

    MatrixXd ddq_dq_plus(MatrixXd::Zero(model.nv, model.nv));
    MatrixXd ddq_dv_plus(MatrixXd::Zero(model.nv, model.nv));
    MatrixXd ddq_dtau_plus(MatrixXd::Zero(model.nv, model.nv));

    VectorXd v_eps(VectorXd::Zero(model.nv));
    VectorXd q_plus(model.nq);
    VectorXd v_plus(model.nv);

    const double alpha = 1e-7;

    // 2a) Perturbation of q along its k-th tangent direction: page k of d2ddq_dqdq and
    //     d2ddq_dtaudq.
    for (int k = 0; k < model.nv; ++k)
    {
      v_eps[k] += alpha;
      q_plus = integrate(model, q, v_eps);
      computeABADerivatives(
        model, data_fd, q_plus, v, tau, ddq_dq_plus, ddq_dv_plus, ddq_dtau_plus);

      const MatrixXd d_ddq_dq = (ddq_dq_plus - ddq_dq_base) / alpha;
      const MatrixXd d_ddq_dtau = (ddq_dtau_plus - ddq_dtau_base) / alpha;
      for (int j = 0; j < model.nv; ++j)
        for (int i = 0; i < model.nv; ++i)
        {
          d2ddq_dqdq_fd(i, j, k) = d_ddq_dq(i, j);
          d2ddq_dtaudq_fd(i, j, k) = d_ddq_dtau(i, j);
        }
      v_eps[k] -= alpha;
    }

    // 2b) Perturbation of v along its k-th direction: page k of d2ddq_dvdv and d2ddq_dqdv.
    for (int k = 0; k < model.nv; ++k)
    {
      v_eps[k] += alpha;
      v_plus = v + v_eps;
      computeABADerivatives(
        model, data_fd, q, v_plus, tau, ddq_dq_plus, ddq_dv_plus, ddq_dtau_plus);

      const MatrixXd d_ddq_dv = (ddq_dv_plus - ddq_dv_base) / alpha;
      const MatrixXd d_ddq_dq = (ddq_dq_plus - ddq_dq_base) / alpha;
      for (int j = 0; j < model.nv; ++j)
        for (int i = 0; i < model.nv; ++i)
        {
          d2ddq_dvdv_fd(i, j, k) = d_ddq_dv(i, j);
          d2ddq_dqdv_fd(i, j, k) = d_ddq_dq(i, j);
        }
      v_eps[k] -= alpha;
    }

    // 3) Compare analytical and finite-difference results.
    BOOST_CHECK(asVector(d2ddq_dqdq).isApprox(asVector(d2ddq_dqdq_fd), sqrt(alpha)));
    BOOST_CHECK(asVector(d2ddq_dvdv).isApprox(asVector(d2ddq_dvdv_fd), sqrt(alpha)));
    BOOST_CHECK(asVector(d2ddq_dqdv).isApprox(asVector(d2ddq_dqdv_fd), sqrt(alpha)));
    BOOST_CHECK(asVector(d2ddq_dtaudq).isApprox(asVector(d2ddq_dtaudq_fd), sqrt(alpha)));

    // 4) The Data-member overload must produce the same result as the explicit-output overload.
    Data data2(model);
    ComputeABASecondOrderDerivatives(model, data2, q, v, tau);

    BOOST_CHECK(asVector(d2ddq_dqdq).isApprox(asVector(data2.d2ddq_dqdq)));
    BOOST_CHECK(asVector(d2ddq_dvdv).isApprox(asVector(data2.d2ddq_dvdv)));
    BOOST_CHECK(asVector(d2ddq_dqdv).isApprox(asVector(data2.d2ddq_dqdv)));
    BOOST_CHECK(asVector(d2ddq_dtaudq).isApprox(asVector(data2.d2ddq_dtaudq)));
    BOOST_CHECK(model.check(data2));
  }
} // namespace

BOOST_AUTO_TEST_CASE(test_aba_derivatives_SO)
{
  Model model;
  buildModels::humanoidRandom(model);
  checkABASecondOrderDerivativesAgainstFiniteDifferences(model);
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives_SO_other_models)
{
  // Joints whose motion subspace depends on the configuration (e.g. the composite root joint of
  // humanoidRandom(model, false)) are not tested: the first-order derivatives this algorithm
  // builds on assume a constant motion subspace (see issue #2967).
  {
    // Fixed-base manipulator
    Model model;
    buildModels::manipulator(model);
    checkABASecondOrderDerivativesAgainstFiniteDifferences(model);
  }
  {
    // Humanoid with a free-flyer root
    Model model;
    const bool using_free_floating = true;
    buildModels::humanoid(model, using_free_floating);
    checkABASecondOrderDerivativesAgainstFiniteDifferences(model);
  }
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives_SO_reuse_data)
{
  // The result must not depend on what has been previously computed in data.
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const VectorXd q = randomConfiguration(model);
  const VectorXd v(VectorXd::Random(model.nv));
  const VectorXd tau(VectorXd::Random(model.nv));

  Data data_ref(model);
  ComputeABASecondOrderDerivatives(model, data_ref, q, v, tau);

  Data data(model);
  const VectorXd q_other = randomConfiguration(model);
  const VectorXd v_other(VectorXd::Random(model.nv));
  const VectorXd tau_other(VectorXd::Random(model.nv));
  ComputeABASecondOrderDerivatives(model, data, q_other, v_other, tau_other);
  ComputeABASecondOrderDerivatives(model, data, q, v, tau);

  BOOST_CHECK(asVector(data.d2ddq_dqdq) == asVector(data_ref.d2ddq_dqdq));
  BOOST_CHECK(asVector(data.d2ddq_dvdv) == asVector(data_ref.d2ddq_dvdv));
  BOOST_CHECK(asVector(data.d2ddq_dqdv) == asVector(data_ref.d2ddq_dqdv));
  BOOST_CHECK(asVector(data.d2ddq_dtaudq) == asVector(data_ref.d2ddq_dtaudq));
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives_SO_no_tensors)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  const VectorXd q = randomConfiguration(model);
  const VectorXd v(VectorXd::Random(model.nv));
  const VectorXd tau(VectorXd::Random(model.nv));

  Data data(model, DataAllocationOption::NO_TENSORS);
  BOOST_CHECK_THROW(
    ComputeABASecondOrderDerivatives(model, data, q, v, tau), std::invalid_argument);

  Data::Tensor3x d2ddq_dqdq(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dvdv(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dqdv(model.nv, model.nv, model.nv);
  Data::Tensor3x d2ddq_dtaudq(model.nv, model.nv, model.nv);
  BOOST_CHECK_THROW(
    ComputeABASecondOrderDerivatives(
      model, data, q, v, tau, d2ddq_dqdq, d2ddq_dvdv, d2ddq_dqdv, d2ddq_dtaudq),
    std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
