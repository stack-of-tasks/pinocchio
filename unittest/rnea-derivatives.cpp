//
// Copyright (c) 2017-2020 CNRS INRIA
//
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Main
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea-derivatives-SO.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/included/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_generalized_gravity_derivatives) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));

  /// Check againt non-derivative algo
  MatrixXd g_partial_dq(model.nv, model.nv);
  g_partial_dq.setZero();
  computeGeneralizedGravityDerivatives(model, data, q, g_partial_dq);

  VectorXd g0 = computeGeneralizedGravity(model, data_fd, q);
  BOOST_CHECK(data.g.isApprox(g0));

  MatrixXd g_partial_dq_fd(model.nv, model.nv);
  g_partial_dq_fd.setZero();

  VectorXd v_eps(Eigen::VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd g_plus(model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    g_plus = computeGeneralizedGravity(model, data_fd, q_plus);

    g_partial_dq_fd.col(k) = (g_plus - g0) / alpha;
    v_eps[k] -= alpha;
  }

  BOOST_CHECK(g_partial_dq.isApprox(g_partial_dq_fd, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_generalized_gravity_derivatives_fext) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  ForceVector fext((size_t)model.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  // Check againt non-derivative algo
  MatrixXd static_vec_partial_dq(model.nv, model.nv);
  static_vec_partial_dq.setZero();
  computeStaticTorqueDerivatives(model, data, q, fext, static_vec_partial_dq);

  VectorXd tau0 = computeStaticTorque(model, data_fd, q, fext);
  BOOST_CHECK(data.tau.isApprox(tau0));

  std::cout << "data.tau: " << data.tau.transpose() << std::endl;
  std::cout << "tau0: " << tau0.transpose() << std::endl;

  MatrixXd static_vec_partial_dq_fd(model.nv, model.nv);

  VectorXd v_eps(Eigen::VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    tau_plus = computeStaticTorque(model, data_fd, q_plus, fext);

    static_vec_partial_dq_fd.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(
      static_vec_partial_dq.isApprox(static_vec_partial_dq_fd, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_rnea_derivatives) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model), data_fd(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  /// Check againt computeGeneralizedGravityDerivatives
  MatrixXd rnea_partial_dq(model.nv, model.nv);
  rnea_partial_dq.setZero();
  MatrixXd rnea_partial_dv(model.nv, model.nv);
  rnea_partial_dv.setZero();
  MatrixXd rnea_partial_da(model.nv, model.nv);
  rnea_partial_da.setZero();
  computeRNEADerivatives(model, data, q, VectorXd::Zero(model.nv),
                         VectorXd::Zero(model.nv), rnea_partial_dq,
                         rnea_partial_dv, rnea_partial_da);
  rnea(model, data_ref, q, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv));
  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k) {
    BOOST_CHECK(data.of[k].isApprox(data.oMi[k].act(data_ref.f[k])));
  }

  MatrixXd g_partial_dq(model.nv, model.nv);
  g_partial_dq.setZero();
  computeGeneralizedGravityDerivatives(model, data_ref, q, g_partial_dq);

  BOOST_CHECK(data.dFdq.isApprox(data_ref.dFdq));
  BOOST_CHECK(rnea_partial_dq.isApprox(g_partial_dq));
  BOOST_CHECK(data.tau.isApprox(data_ref.g));

  VectorXd tau0 = rnea(model, data_fd, q, VectorXd::Zero(model.nv),
                       VectorXd::Zero(model.nv));
  MatrixXd rnea_partial_dq_fd(model.nv, model.nv);
  rnea_partial_dq_fd.setZero();

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    tau_plus = rnea(model, data_fd, q_plus, VectorXd::Zero(model.nv),
                    VectorXd::Zero(model.nv));

    rnea_partial_dq_fd.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd, sqrt(alpha)));

  // Check with q and a non zero
  tau0 = rnea(model, data_fd, q, 0 * v, a);
  rnea_partial_dq_fd.setZero();

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    tau_plus = rnea(model, data_fd, q_plus, VectorXd::Zero(model.nv), a);

    rnea_partial_dq_fd.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }

  rnea_partial_dq.setZero();
  computeRNEADerivatives(model, data, q, VectorXd::Zero(model.nv), a,
                         rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
  forwardKinematics(model, data_ref, q, VectorXd::Zero(model.nv), a);

  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k) {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.oh[k].isApprox(Force::Zero()));
  }

  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd, sqrt(alpha)));

  // Check with q and v non zero
  const Motion gravity(model.gravity);
  model.gravity.setZero();
  tau0 = rnea(model, data_fd, q, v, VectorXd::Zero(model.nv));
  rnea_partial_dq_fd.setZero();

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    tau_plus = rnea(model, data_fd, q_plus, v, VectorXd::Zero(model.nv));

    rnea_partial_dq_fd.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }

  VectorXd v_plus(v);
  MatrixXd rnea_partial_dv_fd(model.nv, model.nv);
  rnea_partial_dv_fd.setZero();

  for (int k = 0; k < model.nv; ++k) {
    v_plus[k] += alpha;
    tau_plus = rnea(model, data_fd, q, v_plus, VectorXd::Zero(model.nv));

    rnea_partial_dv_fd.col(k) = (tau_plus - tau0) / alpha;
    v_plus[k] -= alpha;
  }

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  computeRNEADerivatives(model, data, q, v, VectorXd::Zero(model.nv),
                         rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
  forwardKinematics(model, data_ref, q, v, VectorXd::Zero(model.nv));

  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k) {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  }

  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd, sqrt(alpha)));
  BOOST_CHECK(rnea_partial_dv.isApprox(rnea_partial_dv_fd, sqrt(alpha)));

  //    std::cout << "rnea_partial_dv:\n" << rnea_partial_dv.block<10,10>(0,0)
  //    << std::endl; std::cout << "rnea_partial_dv ref:\n" <<
  //    rnea_partial_dv_fd.block<10,10>(0,0) << std::endl; std::cout <<
  //    "rnea_partial_dv:\n" << rnea_partial_dv.topRows<10>() << std::endl;
  //    std::cout << "rnea_partial_dv ref:\n" <<
  //    rnea_partial_dv_fd.topRows<10>() << std::endl;
  // Check with q, v and a non zero
  model.gravity = gravity;
  v_plus = v;
  tau0 = rnea(model, data_fd, q, v, a);
  rnea_partial_dq_fd.setZero();

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    tau_plus = rnea(model, data_fd, q_plus, v, a);

    rnea_partial_dq_fd.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }

  rnea_partial_dv_fd.setZero();
  for (int k = 0; k < model.nv; ++k) {
    v_plus[k] += alpha;
    tau_plus = rnea(model, data_fd, q, v_plus, a);

    rnea_partial_dv_fd.col(k) = (tau_plus - tau0) / alpha;
    v_plus[k] -= alpha;
  }

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq, rnea_partial_dv,
                         rnea_partial_da);
  forwardKinematics(model, data_ref, q, v, a);

  for (Model::JointIndex k = 1; k < (Model::JointIndex)model.njoints; ++k) {
    BOOST_CHECK(data.a[k].isApprox(data_ref.a[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
  }

  computeJointJacobiansTimeVariation(model, data_ref, q, v);
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  crba(model, data_ref, q);

  rnea_partial_da.triangularView<Eigen::StrictlyLower>() =
      rnea_partial_da.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
      data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK(rnea_partial_da.isApprox(data_ref.M));

  BOOST_CHECK(data.tau.isApprox(tau0));
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd, sqrt(alpha)));
  BOOST_CHECK(rnea_partial_dv.isApprox(rnea_partial_dv_fd, sqrt(alpha)));

  Data data2(model);
  computeRNEADerivatives(model, data2, q, v, a);
  data2.M.triangularView<Eigen::StrictlyLower>() =
      data2.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(rnea_partial_dq.isApprox(data2.dtau_dq));
  BOOST_CHECK(rnea_partial_dv.isApprox(data2.dtau_dv));
  BOOST_CHECK(rnea_partial_da.isApprox(data2.M));
}

BOOST_AUTO_TEST_CASE(test_rnea_derivatives_SO) {
  using namespace Eigen;
  using namespace pinocchio;
  typedef Model::ConfigVectorType ConfigVector;
  typedef Model::TangentVectorType TangentVector;

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

  computeRNEADerivativesSO(model, data, q, VectorXd::Zero(model.nv),
                           VectorXd::Zero(model.nv), dtau2_dq, dtau2_dv,
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

  computeRNEADerivatives(model, data, q, VectorXd::Zero(model.nv),
                         VectorXd::Zero(model.nv), rnea_partial_dq,
                         rnea_partial_dv, rnea_partial_da);

  const double alpha = 1e-7;
  const double eps = 1e-6;

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(model, data_fd, q_plus, VectorXd::Zero(model.nv),
                           VectorXd::Zero(model.nv), drnea_dq_plus,
                           drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    for (int ii = 0; ii < model.nv; ii++) {
      for (int jj = 0; jj < model.nv; jj++) {
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
  computeRNEADerivativesSO(model, data, q, VectorXd::Zero(model.nv), a,
                           dtau2_dq, dtau2_dv, dtau2_dqdv, dtau2_dadq);

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  rnea_partial_da.setZero();

  dtau2_dq_fd.setZero();
  dtau2_dadq_fd.setZero();
  drnea_dq_plus.setZero();
  drnea_dv_plus.setZero();
  drnea_da_plus.setZero();

  computeRNEADerivatives(model, data, q, VectorXd::Zero(model.nv), a,
                         rnea_partial_dq, rnea_partial_dv, rnea_partial_da);

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(model, data_fd, q_plus, VectorXd::Zero(model.nv), a,
                           drnea_dq_plus, drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    temp2 = (drnea_da_plus - rnea_partial_da) / alpha;
    temp2.triangularView<Eigen::StrictlyLower>() =
        temp2.transpose().triangularView<Eigen::StrictlyLower>();
    for (int ii = 0; ii < model.nv; ii++) {
      for (int jj = 0; jj < model.nv; jj++) {
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
  computeRNEADerivativesSO(model, data, q, v, a, dtau2_dq, dtau2_dv, dtau2_dqdv,
                           dtau2_dadq);

  rnea_partial_dq.setZero();
  rnea_partial_dv.setZero();
  rnea_partial_da.setZero();
  computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq, rnea_partial_dv,
                         rnea_partial_da);

  dtau2_dq_fd.setZero();
  dtau2_dadq_fd.setZero();
  drnea_dq_plus.setZero();
  drnea_dv_plus.setZero();
  drnea_da_plus.setZero();

  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    computeRNEADerivatives(model, data_fd, q_plus, v, a, drnea_dq_plus,
                           drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    temp2 = (drnea_da_plus - rnea_partial_da) / alpha;
    temp2.triangularView<Eigen::StrictlyLower>() =
        temp2.transpose().triangularView<Eigen::StrictlyLower>();
    for (int ii = 0; ii < model.nv; ii++) {
      for (int jj = 0; jj < model.nv; jj++) {
        dtau2_dq_fd(jj, ii, k) = temp1(jj, ii);
        dtau2_dadq_fd(jj, ii, k) = temp2(jj, ii);
      }
    }
    v_eps[k] -= alpha;
  }
  dtau2_dv_fd.setZero();
  dtau2_dqdv_fd.setZero();
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    v_plus = v + v_eps;
    computeRNEADerivatives(model, data_fd, q, v_plus, a, drnea_dq_plus,
                           drnea_dv_plus, drnea_da_plus);
    temp1 = (drnea_dv_plus - rnea_partial_dv) / alpha;
    temp2 = (drnea_dq_plus - rnea_partial_dq) / alpha;
    for (int ii = 0; ii < model.nv; ii++) {
      for (int jj = 0; jj < model.nv; jj++) {
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
  computeRNEADerivativesSO(model, data2, q, v, a);

  Map<VectorXd> mq2(data2.d2tau_dq.data(), (data2.d2tau_dq).size());
  Map<VectorXd> mv2(data2.d2tau_dv.data(), (data2.d2tau_dv).size());
  Map<VectorXd> mqv2(data2.d2tau_dqdv.data(), (data2.d2tau_dqdv).size());
  Map<VectorXd> maq2(data2.d2tau_dadq.data(), (data2.d2tau_dadq).size());

  BOOST_CHECK(mq.isApprox(mq2, sqrt(alpha)));
  BOOST_CHECK(mv.isApprox(mv2, sqrt(alpha)));
  BOOST_CHECK(mqv.isApprox(mqv2, sqrt(alpha)));
  BOOST_CHECK(maq.isApprox(maq2, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_rnea_derivatives_fext) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  typedef Model::Force Force;

  Data data(model), data_fd(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  ForceVector fext((size_t)model.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  /// Check againt computeGeneralizedGravityDerivatives
  MatrixXd rnea_partial_dq(model.nv, model.nv);
  rnea_partial_dq.setZero();
  MatrixXd rnea_partial_dv(model.nv, model.nv);
  rnea_partial_dv.setZero();
  MatrixXd rnea_partial_da(model.nv, model.nv);
  rnea_partial_da.setZero();

  computeRNEADerivatives(model, data, q, v, a, fext, rnea_partial_dq,
                         rnea_partial_dv, rnea_partial_da);
  rnea(model, data_ref, q, v, a, fext);

  BOOST_CHECK(data.tau.isApprox(data_ref.tau));

  computeRNEADerivatives(model, data_ref, q, v, a);
  BOOST_CHECK(rnea_partial_dv.isApprox(data_ref.dtau_dv));
  BOOST_CHECK(rnea_partial_da.isApprox(data_ref.M));

  MatrixXd rnea_partial_dq_fd(model.nv, model.nv);
  rnea_partial_dq_fd.setZero();
  MatrixXd rnea_partial_dv_fd(model.nv, model.nv);
  rnea_partial_dv_fd.setZero();
  MatrixXd rnea_partial_da_fd(model.nv, model.nv);
  rnea_partial_da_fd.setZero();

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double eps = 1e-8;

  const VectorXd tau_ref = rnea(model, data_ref, q, v, a, fext);
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] = eps;
    q_plus = integrate(model, q, v_eps);
    tau_plus = rnea(model, data_fd, q_plus, v, a, fext);

    rnea_partial_dq_fd.col(k) = (tau_plus - tau_ref) / eps;

    v_eps[k] = 0.;
  }
  BOOST_CHECK(rnea_partial_dq.isApprox(rnea_partial_dq_fd, sqrt(eps)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k) {
    v_plus[k] += eps;

    tau_plus = rnea(model, data_fd, q, v_plus, a, fext);

    rnea_partial_dv_fd.col(k) = (tau_plus - tau_ref) / eps;

    v_plus[k] -= eps;
  }
  BOOST_CHECK(rnea_partial_dv.isApprox(rnea_partial_dv_fd, sqrt(eps)));

  VectorXd a_plus(a);
  for (int k = 0; k < model.nv; ++k) {
    a_plus[k] += eps;

    tau_plus = rnea(model, data_fd, q, v, a_plus, fext);

    rnea_partial_da_fd.col(k) = (tau_plus - tau_ref) / eps;

    a_plus[k] -= eps;
  }

  rnea_partial_da.triangularView<Eigen::Lower>() =
      rnea_partial_da.transpose().triangularView<Eigen::Lower>();
  BOOST_CHECK(rnea_partial_da.isApprox(rnea_partial_da_fd, sqrt(eps)));

  // test the shortcut
  Data data_shortcut(model);
  computeRNEADerivatives(model, data_shortcut, q, v, a, fext);
  BOOST_CHECK(data_shortcut.dtau_dq.isApprox(rnea_partial_dq));
  BOOST_CHECK(data_shortcut.dtau_dv.isApprox(rnea_partial_dv));
  data_shortcut.M.triangularView<Eigen::Lower>() =
      data_shortcut.M.transpose().triangularView<Eigen::Lower>();
  BOOST_CHECK(data_shortcut.M.isApprox(rnea_partial_da));
}

BOOST_AUTO_TEST_CASE(test_rnea_derivatives_vs_kinematics_derivatives) {
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

  /// Check againt computeGeneralizedGravityDerivatives
  MatrixXd rnea_partial_dq(model.nv, model.nv);
  rnea_partial_dq.setZero();
  MatrixXd rnea_partial_dv(model.nv, model.nv);
  rnea_partial_dv.setZero();
  MatrixXd rnea_partial_da(model.nv, model.nv);
  rnea_partial_da.setZero();

  computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq, rnea_partial_dv,
                         rnea_partial_da);
  computeForwardKinematicsDerivatives(model, data_ref, q, v, a);

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));

  for (size_t k = 1; k < (size_t)model.njoints; ++k) {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.ov[k]));
    BOOST_CHECK(data.oa[k].isApprox(data_ref.oa[k]));
  }
}

BOOST_AUTO_TEST_CASE(test_multiple_calls) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data1(model), data2(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  computeRNEADerivatives(model, data1, q, v, a);
  data2 = data1;

  for (int k = 0; k < 20; ++k) {
    computeRNEADerivatives(model, data1, q, v, a);
  }

  BOOST_CHECK(data1.J.isApprox(data2.J));
  BOOST_CHECK(data1.dJ.isApprox(data2.dJ));
  BOOST_CHECK(data1.dVdq.isApprox(data2.dVdq));
  BOOST_CHECK(data1.dAdq.isApprox(data2.dAdq));
  BOOST_CHECK(data1.dAdv.isApprox(data2.dAdv));

  BOOST_CHECK(data1.dFdq.isApprox(data2.dFdq));
  BOOST_CHECK(data1.dFdv.isApprox(data2.dFdv));
  BOOST_CHECK(data1.dFda.isApprox(data2.dFda));

  BOOST_CHECK(data1.dtau_dq.isApprox(data2.dtau_dq));
  BOOST_CHECK(data1.dtau_dv.isApprox(data2.dtau_dv));
  BOOST_CHECK(data1.M.isApprox(data2.M));
}

BOOST_AUTO_TEST_CASE(test_get_coriolis) {
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data_ref(model);
  Data data(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  computeCoriolisMatrix(model, data_ref, q, v);

  computeRNEADerivatives(model, data, q, v, tau);
  getCoriolisMatrix(model, data);

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  for (JointIndex k = 1; k < model.joints.size(); ++k) {
    BOOST_CHECK(data.B[k].isApprox(data_ref.B[k]));
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
  }

  BOOST_CHECK(data.C.isApprox(data_ref.C));
}

BOOST_AUTO_TEST_SUITE_END()
