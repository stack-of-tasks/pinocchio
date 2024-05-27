//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_aba_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));
  VectorXd a(aba(model, data_ref, q, v, tau, Convention::LOCAL));

  VectorXd tau_from_a(rnea(model, data_ref, q, v, a));
  BOOST_CHECK(tau_from_a.isApprox(tau));

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  Data::RowMatrixXs aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  const double prec = Eigen::NumTraits<double>::dummy_precision();

  computeABADerivatives(model, data, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);
  computeRNEADerivatives(model, data_ref, q, v, a);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.ov[k]));
    BOOST_CHECK(data.oh[k].isApprox(data_ref.oh[k]));
    BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oa[k] - model.gravity));
    BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oa_gf[k]));
    BOOST_CHECK(data.of[k].isApprox(data_ref.of[k], 1e1 * prec));
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
    BOOST_CHECK(data.doYcrb[k].isApprox(data_ref.doYcrb[k]));
  }

  aba(model, data_ref, q, v, tau, Convention::LOCAL);
  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oYaba[k].isApprox(
      data_ref.oMi[k].toDualActionMatrix() * data_ref.Yaba[k]
      * data_ref.oMi[k].inverse().toActionMatrix()));
    //    BOOST_CHECK(data.of[k].isApprox(data.oMi[k].act(data_ref.f[k])));
  }
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  aba(model, data_ref, q, v, tau, Convention::WORLD);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.u.isApprox(data_ref.u));
  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.ov[k]));
    BOOST_CHECK(data.oYaba[k].isApprox(data_ref.oYaba[k]));
    //    BOOST_CHECK(data.of[k].isApprox(data_ref.of[k]));
    BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oa_gf[k]));
    BOOST_CHECK(data.joints[k].U().isApprox(data_ref.joints[k].U()));
    BOOST_CHECK(data.joints[k].StU().isApprox(data_ref.joints[k].StU()));
    BOOST_CHECK(data.joints[k].Dinv().isApprox(data_ref.joints[k].Dinv()));
    BOOST_CHECK(data.joints[k].UDinv().isApprox(data_ref.joints[k].UDinv()));
  }
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  computeJointJacobians(model, data_ref, q);
  BOOST_CHECK(data.J.isApprox(data_ref.J));

  computeMinverse(model, data_ref, q);
  data_ref.Minv.triangularView<Eigen::StrictlyLower>() =
    data_ref.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(aba_partial_dtau.isApprox(data_ref.Minv));

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dtau_dq.isApprox(data_ref.dtau_dq));
  BOOST_CHECK(data.dtau_dv.isApprox(data_ref.dtau_dv));

  MatrixXd aba_partial_dq_fd(model.nv, model.nv);
  aba_partial_dq_fd.setZero();
  MatrixXd aba_partial_dv_fd(model.nv, model.nv);
  aba_partial_dv_fd.setZero();
  MatrixXd aba_partial_dtau_fd(model.nv, model.nv);
  aba_partial_dtau_fd.setZero();

  Data data_fd(model);
  VectorXd a0 = aba(model, data_fd, q, v, tau, Convention::LOCAL);
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    a_plus = aba(model, data_fd, q_plus, v, tau, Convention::LOCAL);

    aba_partial_dq_fd.col(k) = (a_plus - a0) / alpha;
    v_eps[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_fd, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = aba(model, data_fd, q, v_plus, tau, Convention::LOCAL);

    aba_partial_dv_fd.col(k) = (a_plus - a0) / alpha;
    v_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_fd, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    a_plus = aba(model, data_fd, q, v, tau_plus, Convention::LOCAL);

    aba_partial_dtau_fd.col(k) = (a_plus - a0) / alpha;
    tau_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_fd, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_aba_minimal_argument)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));
  VectorXd a(aba(model, data_ref, q, v, tau, Convention::LOCAL));

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  Data::RowMatrixXs aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  computeABADerivatives(
    model, data_ref, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

  computeABADerivatives(model, data, q, v, tau);

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dtau_dq.isApprox(data_ref.dtau_dq));
  BOOST_CHECK(data.dtau_dv.isApprox(data_ref.dtau_dv));
  BOOST_CHECK(data.Minv.isApprox(aba_partial_dtau));
  BOOST_CHECK(data.ddq_dq.isApprox(aba_partial_dq));
  BOOST_CHECK(data.ddq_dv.isApprox(aba_partial_dv));
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives_fext)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));
  VectorXd a(aba(model, data_ref, q, v, tau, Convention::LOCAL));

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  ForceVector fext((size_t)model.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  Data::RowMatrixXs aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  computeABADerivatives(
    model, data, q, v, tau, fext, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

  aba(model, data_ref, q, v, tau, fext, Convention::LOCAL);
  //  updateGlobalPlacements(model, data_ref);
  //  for(size_t k =1; k < (size_t)model.njoints; ++k)
  //  {
  //    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  //    BOOST_CHECK(daita.of[k].isApprox(data_ref.oMi[k].act(data.f[k])));
  //
  //  }
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  computeABADerivatives(model, data_ref, q, v, tau);
  BOOST_CHECK(aba_partial_dv.isApprox(data_ref.ddq_dv));
  BOOST_CHECK(aba_partial_dtau.isApprox(data_ref.Minv));

  MatrixXd aba_partial_dq_fd(model.nv, model.nv);
  aba_partial_dq_fd.setZero();
  MatrixXd aba_partial_dv_fd(model.nv, model.nv);
  aba_partial_dv_fd.setZero();
  MatrixXd aba_partial_dtau_fd(model.nv, model.nv);
  aba_partial_dtau_fd.setZero();

  Data data_fd(model);
  const VectorXd a0 = aba(model, data_fd, q, v, tau, fext, Convention::LOCAL);
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    a_plus = aba(model, data_fd, q_plus, v, tau, fext, Convention::LOCAL);

    aba_partial_dq_fd.col(k) = (a_plus - a0) / alpha;
    v_eps[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_fd, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = aba(model, data_fd, q, v_plus, tau, fext, Convention::LOCAL);

    aba_partial_dv_fd.col(k) = (a_plus - a0) / alpha;
    v_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_fd, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    a_plus = aba(model, data_fd, q, v, tau_plus, fext, Convention::LOCAL);

    aba_partial_dtau_fd.col(k) = (a_plus - a0) / alpha;
    tau_plus[k] -= alpha;
  }
  BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_fd, sqrt(alpha)));

  // test the shortcut
  Data data_shortcut(model);
  computeABADerivatives(model, data_shortcut, q, v, tau, fext);
  BOOST_CHECK(data_shortcut.ddq_dq.isApprox(aba_partial_dq));
  BOOST_CHECK(data_shortcut.ddq_dv.isApprox(aba_partial_dv));
  BOOST_CHECK(data_shortcut.Minv.isApprox(aba_partial_dtau));
}

BOOST_AUTO_TEST_CASE(test_multiple_calls)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data1(model), data2(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));

  computeABADerivatives(model, data1, q, v, tau);
  data2 = data1;

  for (int k = 0; k < 20; ++k)
  {
    computeABADerivatives(model, data1, q, v, tau);
  }

  BOOST_CHECK(data1.J.isApprox(data2.J));
  BOOST_CHECK(data1.dJ.isApprox(data2.dJ));
  BOOST_CHECK(data1.dVdq.isApprox(data2.dVdq));
  BOOST_CHECK(data1.dAdq.isApprox(data2.dAdq));
  BOOST_CHECK(data1.dAdv.isApprox(data2.dAdv));

  BOOST_CHECK(data1.dFdq.isApprox(data2.dFdq));
  BOOST_CHECK(data1.dFdv.isApprox(data2.dFdv));

  BOOST_CHECK(data1.dtau_dq.isApprox(data2.dtau_dq));
  BOOST_CHECK(data1.dtau_dv.isApprox(data2.dtau_dv));

  BOOST_CHECK(data1.ddq_dq.isApprox(data2.ddq_dq));
  BOOST_CHECK(data1.ddq_dv.isApprox(data2.ddq_dv));
  BOOST_CHECK(data1.Minv.isApprox(data2.Minv));
}

BOOST_AUTO_TEST_CASE(test_aba_derivatives_vs_kinematics_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  VectorXd tau = rnea(model, data_ref, q, v, a);

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  MatrixXd aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  computeABADerivatives(model, data, q, v, tau, aba_partial_dq, aba_partial_dv, aba_partial_dtau);
  computeForwardKinematicsDerivatives(model, data_ref, q, v, a);

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));

  for (size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.ov[k]));
    BOOST_CHECK(data.oa[k].isApprox(data_ref.oa[k]));
  }
}

BOOST_AUTO_TEST_CASE(test_optimized_aba_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  MatrixXd aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  aba(model, data, q, v, tau, Convention::WORLD);
  computeABADerivatives(model, data, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

  MatrixXd aba_partial_dq_ref(model.nv, model.nv);
  aba_partial_dq_ref.setZero();
  MatrixXd aba_partial_dv_ref(model.nv, model.nv);
  aba_partial_dv_ref.setZero();
  MatrixXd aba_partial_dtau_ref(model.nv, model.nv);
  aba_partial_dtau_ref.setZero();

  computeABADerivatives(
    model, data_ref, q, v, tau, aba_partial_dq_ref, aba_partial_dv_ref, aba_partial_dtau_ref);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK(data.Minv.isApprox(data_ref.Minv));

  BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_ref));
  BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_ref));
  BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_ref));

  // Test multiple calls
  const int num_calls = 20;
  aba(model, data, q, v, tau, Convention::WORLD);
  for (int it = 0; it < num_calls; ++it)
  {
    computeABADerivatives(model, data, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

    BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
    BOOST_CHECK(data.Minv.isApprox(data_ref.Minv));

    for (size_t joint_id = 1; joint_id < model.joints.size(); ++joint_id)
    {
      BOOST_CHECK(data.oMi[joint_id].isApprox(data_ref.oMi[joint_id]));
      BOOST_CHECK(data.ov[joint_id].isApprox(data_ref.ov[joint_id]));
      BOOST_CHECK(data.oa[joint_id].isApprox(data_ref.oa[joint_id]));
      BOOST_CHECK(data.oa_gf[joint_id].isApprox(data_ref.oa_gf[joint_id]));
      BOOST_CHECK(data.of[joint_id].isApprox(data_ref.of[joint_id]));
      BOOST_CHECK(data.oh[joint_id].isApprox(data_ref.oh[joint_id]));
      BOOST_CHECK(data.oYaba[joint_id].isApprox(data_ref.oYaba[joint_id]));
      BOOST_CHECK(data.oYcrb[joint_id].isApprox(data_ref.oYcrb[joint_id]));
    }

    BOOST_CHECK(data.J.isApprox(data_ref.J));
    BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
    BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
    BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
    BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
    BOOST_CHECK(data.dFdq.isApprox(data_ref.dFdq));
    BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));
    BOOST_CHECK(data.dFda.isApprox(data_ref.dFda));

    BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_ref));
    BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_ref));
    BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_ref));
  }
}

BOOST_AUTO_TEST_CASE(test_optimized_aba_derivatives_fext)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd tau(VectorXd::Random(model.nv));

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  ForceVector fext((size_t)model.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  MatrixXd aba_partial_dq(model.nv, model.nv);
  aba_partial_dq.setZero();
  MatrixXd aba_partial_dv(model.nv, model.nv);
  aba_partial_dv.setZero();
  MatrixXd aba_partial_dtau(model.nv, model.nv);
  aba_partial_dtau.setZero();

  aba(model, data, q, v, tau, fext, Convention::WORLD);
  computeABADerivatives(model, data, fext, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

  MatrixXd aba_partial_dq_ref(model.nv, model.nv);
  aba_partial_dq_ref.setZero();
  MatrixXd aba_partial_dv_ref(model.nv, model.nv);
  aba_partial_dv_ref.setZero();
  MatrixXd aba_partial_dtau_ref(model.nv, model.nv);
  aba_partial_dtau_ref.setZero();

  computeABADerivatives(
    model, data_ref, q, v, tau, fext, aba_partial_dq_ref, aba_partial_dv_ref, aba_partial_dtau_ref);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK(data.Minv.isApprox(data_ref.Minv));

  BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_ref));
  BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_ref));
  BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_ref));

  // Test multiple calls
  const int num_calls = 20;
  aba(model, data, q, v, tau, fext, Convention::WORLD);
  for (int it = 0; it < num_calls; ++it)
  {
    computeABADerivatives(model, data, fext, aba_partial_dq, aba_partial_dv, aba_partial_dtau);

    BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
    BOOST_CHECK(data.Minv.isApprox(data_ref.Minv));

    for (size_t joint_id = 1; joint_id < model.joints.size(); ++joint_id)
    {
      BOOST_CHECK(data.oMi[joint_id].isApprox(data_ref.oMi[joint_id]));
      BOOST_CHECK(data.ov[joint_id].isApprox(data_ref.ov[joint_id]));
      BOOST_CHECK(data.oa[joint_id].isApprox(data_ref.oa[joint_id]));
      BOOST_CHECK(data.oa_gf[joint_id].isApprox(data_ref.oa_gf[joint_id]));
      BOOST_CHECK(data.of[joint_id].isApprox(data_ref.of[joint_id]));
      BOOST_CHECK(data.oh[joint_id].isApprox(data_ref.oh[joint_id]));
      BOOST_CHECK(data.oYaba[joint_id].isApprox(data_ref.oYaba[joint_id]));
      BOOST_CHECK(data.oYcrb[joint_id].isApprox(data_ref.oYcrb[joint_id]));
    }

    BOOST_CHECK(data.J.isApprox(data_ref.J));
    BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
    BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
    BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
    BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
    BOOST_CHECK(data.dFdq.isApprox(data_ref.dFdq));
    BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));
    BOOST_CHECK(data.dFda.isApprox(data_ref.dFda));

    BOOST_CHECK(aba_partial_dq.isApprox(aba_partial_dq_ref));
    BOOST_CHECK(aba_partial_dv.isApprox(aba_partial_dv_ref));
    BOOST_CHECK(aba_partial_dtau.isApprox(aba_partial_dtau_ref));
  }
}

BOOST_AUTO_TEST_SUITE_END()
