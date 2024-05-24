//
// Copyright (c) 2019-2021 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/energy.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "casadi-utils.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_jacobian)
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

  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  Model::Index joint_id =
    model.existJointName("rarm2") ? model.getJointId("rarm2") : (Model::Index)(model.njoints - 1);
  Data::Matrix6x jacobian_local(6, model.nv), jacobian_world(6, model.nv);
  jacobian_local.setZero();
  jacobian_world.setZero();

  BOOST_CHECK(jacobian_local.isZero() && jacobian_world.isZero());

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::getJointJacobian(model, data, joint_id, pinocchio::WORLD, jacobian_world);
  pinocchio::getJointJacobian(model, data, joint_id, pinocchio::LOCAL, jacobian_local);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  for (Eigen::DenseIndex k = 0; k < model.nq; ++k)
  {
    q_ad[k] = cs_q(k);
  }
  std::cout << "q =\n " << q_ad << std::endl;

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_ad[k] = cs_v(k);
  }
  std::cout << "v =\n " << v_ad << std::endl;

  pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad);
  typedef pinocchio::MotionTpl<ADScalar> MotionAD;
  MotionAD & v_local = ad_data.v[(size_t)joint_id];
  MotionAD v_world = ad_data.oMi[(size_t)joint_id].act(v_local);

  casadi::SX cs_v_local(6, 1), cs_v_world(6, 1);
  for (Eigen::DenseIndex k = 0; k < 6; ++k)
  {
    cs_v_local(k) = v_local.toVector()[k];
    cs_v_world(k) = v_world.toVector()[k];
  }
  std::cout << "v_local = " << cs_v_local << std::endl;
  std::cout << "v_world = " << cs_v_world << std::endl;

  casadi::Function eval_velocity_local(
    "eval_velocity_local", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{cs_v_local});
  std::cout << "eval_velocity_local = " << eval_velocity_local << std::endl;

  casadi::Function eval_velocity_world(
    "eval_velocity_world", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{cs_v_world});
  std::cout << "eval_velocity_world = " << eval_velocity_world << std::endl;

  casadi::SX dv_dv_local = jacobian(cs_v_local, cs_v);
  casadi::Function eval_jacobian_local(
    "eval_jacobian_local", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{dv_dv_local});
  std::cout << "eval_jacobian_local = " << eval_jacobian_local << std::endl;

  casadi::SX dv_dv_world = jacobian(cs_v_world, cs_v);
  casadi::Function eval_jacobian_world(
    "eval_jacobian_world", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{dv_dv_world});

  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(), model.nv, 1) = v;

  casadi::DMVector v_local_res = eval_velocity_local(casadi::DMVector{q_vec, v_vec});
  casadi::DMVector J_local_res = eval_jacobian_local(casadi::DMVector{q_vec, v_vec});
  std::cout << "J_local_res:" << J_local_res << std::endl;

  std::vector<double> v_local_vec(static_cast<std::vector<double>>(v_local_res[0]));
  BOOST_CHECK(
    (jacobian_local * v).isApprox(Eigen::Map<pinocchio::Motion::Vector6>(v_local_vec.data())));

  casadi::DMVector v_world_res = eval_velocity_world(casadi::DMVector{q_vec, v_vec});
  casadi::DMVector J_world_res = eval_jacobian_world(casadi::DMVector{q_vec, v_vec});

  std::vector<double> v_world_vec(static_cast<std::vector<double>>(v_world_res[0]));
  BOOST_CHECK(
    (jacobian_world * v).isApprox(Eigen::Map<pinocchio::Motion::Vector6>(v_world_vec.data())));

  Data::Matrix6x J_local_mat(6, model.nv), J_world_mat(6, model.nv);

  std::vector<double> J_local_vec(static_cast<std::vector<double>>(J_local_res[0]));
  J_local_mat = Eigen::Map<Data::Matrix6x>(J_local_vec.data(), 6, model.nv);
  BOOST_CHECK(jacobian_local.isApprox(J_local_mat));

  std::vector<double> J_world_vec(static_cast<std::vector<double>>(J_world_res[0]));
  J_world_mat = Eigen::Map<Data::Matrix6x>(J_world_vec.data(), 6, model.nv);
  BOOST_CHECK(jacobian_world.isApprox(J_world_mat));
}

BOOST_AUTO_TEST_CASE(test_fk)
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

  pinocchio::forwardKinematics(model, data, q);

  typedef ADModel::ConfigVectorType ConfigVectorAD;
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  pinocchio::casadi::copy(cs_q, q_ad);

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  pinocchio::casadi::copy(cs_v, v_ad);

  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  pinocchio::casadi::copy(cs_a, a_ad);

  pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, a_ad);
  pinocchio::updateGlobalPlacements(ad_model, ad_data);
  pinocchio::updateFramePlacements(ad_model, ad_data);
  //    typedef pinocchio::MotionTpl<ADScalar> MotionAD;
}

BOOST_AUTO_TEST_CASE(test_rnea)
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
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  pinocchio::rnea(model, data, q, v, a);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);

  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  a_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_a).data(), model.nv, 1);

  rnea(ad_model, ad_data, q_ad, v_ad, a_ad);
  casadi::SX tau_ad(model.nv, 1);
  //    Eigen::Map<TangentVectorAD>(tau_ad->data(),model.nv,1)
  //    = ad_data.tau;
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    tau_ad(k) = ad_data.tau[k];
  casadi::Function eval_rnea(
    "eval_rnea", casadi::SXVector{cs_q, cs_v, cs_a}, casadi::SXVector{tau_ad});

  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(), model.nv, 1) = v;

  std::vector<double> a_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(a_vec.data(), model.nv, 1) = a;
  casadi::DM tau_res = eval_rnea(casadi::DMVector{q_vec, v_vec, a_vec})[0];
  std::cout << "tau_res = " << tau_res << std::endl;
  Data::TangentVectorType tau_vec = Eigen::Map<Data::TangentVectorType>(
    static_cast<std::vector<double>>(tau_res).data(), model.nv, 1);

  BOOST_CHECK(data.tau.isApprox(tau_vec));
}

BOOST_AUTO_TEST_CASE(test_crba)
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
  typedef ADModel::TangentVectorType TangentVectorAD;
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::rnea(model, data, q, v, a);

  casadi::SX cs_q = casadi::SX::sym("q", model.nq);
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);

  casadi::SX cs_a = casadi::SX::sym("a", model.nv);
  TangentVectorAD a_ad(model.nv);
  a_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_a).data(), model.nv, 1);

  // RNEA
  rnea(ad_model, ad_data, q_ad, v_ad, a_ad);
  casadi::SX cs_tau(model.nv, 1);
  //    Eigen::Map<TangentVectorAD>(tau_ad->data(),model.nv,1)
  //    = ad_data.tau;
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    cs_tau(k) = ad_data.tau[k];
  casadi::Function eval_rnea(
    "eval_rnea", casadi::SXVector{cs_q, cs_v, cs_a}, casadi::SXVector{cs_tau});
  // CRBA
  crba(ad_model, ad_data, q_ad, pinocchio::Convention::WORLD);
  ad_data.M.triangularView<Eigen::StrictlyLower>() =
    ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  casadi::SX M_ad(model.nv, model.nv);
  for (Eigen::DenseIndex j = 0; j < model.nv; ++j)
  {
    for (Eigen::DenseIndex i = 0; i < model.nv; ++i)
    {
      M_ad(i, j) = ad_data.M(i, j);
    }
  }

  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(), model.nv, 1) = v;

  std::vector<double> a_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(a_vec.data(), model.nv, 1) = a;

  casadi::Function eval_crba("eval_crba", casadi::SXVector{cs_q}, casadi::SXVector{M_ad});
  casadi::DM M_res = eval_crba(casadi::DMVector{q_vec})[0];
  Data::MatrixXs M_mat =
    Eigen::Map<Data::MatrixXs>(static_cast<std::vector<double>>(M_res).data(), model.nv, model.nv);

  BOOST_CHECK(data.M.isApprox(M_mat));

  casadi::SX dtau_da = jacobian(cs_tau, cs_a);
  casadi::Function eval_dtau_da(
    "eval_dtau_da", casadi::SXVector{cs_q, cs_v, cs_a}, casadi::SXVector{dtau_da});
  casadi::DM dtau_da_res = eval_dtau_da(casadi::DMVector{q_vec, v_vec, a_vec})[0];
  Data::MatrixXs dtau_da_mat = Eigen::Map<Data::MatrixXs>(
    static_cast<std::vector<double>>(dtau_da_res).data(), model.nv, model.nv);
  BOOST_CHECK(data.M.isApprox(dtau_da_mat));
}

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
  ConfigVectorAD q_ad(model.nq);
  q_ad = Eigen::Map<ConfigVectorAD>(static_cast<std::vector<ADScalar>>(cs_q).data(), model.nq, 1);

  casadi::SX cs_v = casadi::SX::sym("v", model.nv);
  TangentVectorAD v_ad(model.nv);
  v_ad = Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_v).data(), model.nv, 1);

  casadi::SX cs_tau = casadi::SX::sym("tau", model.nv);
  TangentVectorAD tau_ad(model.nv);
  tau_ad =
    Eigen::Map<TangentVectorAD>(static_cast<std::vector<ADScalar>>(cs_tau).data(), model.nv, 1);

  // ABA
  pinocchio::aba(ad_model, ad_data, q_ad, v_ad, tau_ad, pinocchio::Convention::WORLD);
  casadi::SX cs_ddq(model.nv, 1);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    cs_ddq(k) = ad_data.ddq[k];
  casadi::Function eval_aba(
    "eval_aba", casadi::SXVector{cs_q, cs_v, cs_tau}, casadi::SXVector{cs_ddq});

  std::vector<double> q_vec((size_t)model.nq);
  Eigen::Map<ConfigVector>(q_vec.data(), model.nq, 1) = q;

  std::vector<double> v_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(v_vec.data(), model.nv, 1) = v;

  std::vector<double> tau_vec((size_t)model.nv);
  Eigen::Map<TangentVector>(tau_vec.data(), model.nv, 1) = tau;

  casadi::DM ddq_res = eval_aba(casadi::DMVector{q_vec, v_vec, tau_vec})[0];
  Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(
    static_cast<std::vector<double>>(ddq_res).data(), model.nv, 1);

  BOOST_CHECK(ddq_mat.isApprox(data.ddq));
}

void test_interp_for_model(const pinocchio::ModelTpl<double> & model)
{
  using casadi::DMVector;
  using casadi::SX;
  using casadi::SXVector;
  typedef SX ADScalar;
  typedef pinocchio::ModelTpl<double> Model;
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  std::cout << model;

  ADModel ad_model = model.cast<ADScalar>();
  int nq = model.nq;
  int nv = model.nv;

  SX cs_q0 = SX::sym("q0", nq);
  SX cs_q1 = SX::sym("q1", nq);
  SX cs_dq0 = SX::sym("dq0", nv);
  SX cs_dq1 = SX::sym("dq0", nv);
  SX zero_vec(SX::zeros(2 * nv));

  typedef ADModel::ConfigVectorType cConfig_t;
  typedef ADModel::TangentVectorType cTangent_t;

  cConfig_t cq0(nq), cq1(nq);
  cTangent_t cdq0(nv), cdq1(nv);
  pinocchio::casadi::copy(cs_q0, cq0);
  pinocchio::casadi::copy(cs_q1, cq1);
  pinocchio::casadi::copy(cs_dq0, cdq0);
  pinocchio::casadi::copy(cs_dq1, cdq1);
  ADScalar cs_dqall = vertcat(cs_dq0, cs_dq1);

  auto cq0_i = pinocchio::integrate(ad_model, cq0, cdq0);
  auto cq1_i = pinocchio::integrate(ad_model, cq1, cdq1);

  ADScalar alpha(0.5);

  cConfig_t qinterp = pinocchio::interpolate(ad_model, cq0_i, cq1_i, alpha);
  cConfig_t qneutral = pinocchio::neutral(ad_model);
  cTangent_t log_interp = pinocchio::difference(ad_model, qinterp, qneutral);

  auto norm_interp = log_interp.dot(log_interp);
  auto Jnorm_interp = jacobian(norm_interp, cs_dqall);

  casadi::Function Jnorm_eval(
    "Jnorm", SXVector{cs_q0, cs_q1}, SXVector{substitute(Jnorm_interp, cs_dqall, zero_vec)});
  std::cout << Jnorm_eval << '\n';

  const auto q0 = pinocchio::neutral(model);
  const auto q1 = pinocchio::randomConfiguration(model);
  const auto q2 = pinocchio::randomConfiguration(model);

  typedef Eigen::Map<Model::ConfigVectorType> ConfigMap_t;
  std::vector<double> q0_vec((size_t)nq);
  std::vector<double> q1_vec((size_t)nq);
  std::vector<double> q2_vec((size_t)nq);
  ConfigMap_t(q0_vec.data(), nq, 1) = q0;
  ConfigMap_t(q1_vec.data(), nq, 1) = q1;
  ConfigMap_t(q2_vec.data(), nq, 1) = q2;

  std::cout << Jnorm_eval(DMVector{q0_vec, q0_vec})[0] << '\n';
  std::cout << Jnorm_eval(DMVector{q0_vec, q1_vec})[0] << '\n';
  std::cout << Jnorm_eval(DMVector{q1_vec, q1_vec})[0] << '\n';
  std::cout << Jnorm_eval(DMVector{q2_vec, q2_vec})[0] << '\n';
}

BOOST_AUTO_TEST_CASE(test_interp)
{
  typedef pinocchio::ModelTpl<double> Model;
  Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  test_interp_for_model(model);

  Model model2;
  typedef pinocchio::SE3Tpl<double> SE3;
  size_t baseId = model2.addJoint(0, pinocchio::JointModelSpherical(), SE3::Identity(), "base");
  model2.addJoint(baseId, pinocchio::JointModelRX(), SE3::Random(), "pole");
  model2.lowerPositionLimit.tail<1>().fill(-4.);
  model2.upperPositionLimit.tail<1>().fill(4.);

  test_interp_for_model(model2);
}

BOOST_AUTO_TEST_CASE(test_kinetic_energy)
{
  using casadi::DMVector;
  using casadi::SX;
  using casadi::SXVector;
  typedef SX ADScalar;
  typedef pinocchio::ModelTpl<double> Model;
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  Model model;
  // pinocchio::buildModels::humanoidRandom(model, true);
  model.addJoint(0, pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(), "base");
  model.appendBodyToJoint(1, pinocchio::Inertia::Identity());
  std::cout << model;
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  int nq = model.nq;
  int nv = model.nv;

  SX cs_q0 = SX::sym("q0", nq);
  SX cs_q1 = SX::sym("q1", nq);
  SX cs_dq0 = SX::sym("dq0", nv);
  SX cs_dq1 = SX::sym("dq0", nv);
  SX zero_vec(SX::zeros(2 * nv));

  typedef ADModel::ConfigVectorType cConfig_t;
  typedef ADModel::TangentVectorType cTangent_t;

  cConfig_t cq0(nq), cq1(nq);
  cTangent_t cdq0(nv), cdq1(nv);
  pinocchio::casadi::copy(cs_q0, cq0);
  pinocchio::casadi::copy(cs_q1, cq1);
  pinocchio::casadi::copy(cs_dq0, cdq0);
  pinocchio::casadi::copy(cs_dq1, cdq1);
  ADScalar cs_dqall = vertcat(cs_dq0, cs_dq1);

  cConfig_t cq0_i = pinocchio::integrate(ad_model, cq0, cdq0);
  cConfig_t cq1_i = pinocchio::integrate(ad_model, cq1, cdq1);

  const double dt(0.05);

  cTangent_t cv = pinocchio::difference(ad_model, cq0_i, cq1_i) / dt;
  pinocchio::computeKineticEnergy(ad_model, ad_data, cq0_i, cv);
  const auto KE_expr = ad_data.kinetic_energy;
  const auto gKE_expr = jacobian(KE_expr, cs_dqall);
  const auto HKE_expr = jacobian(gKE_expr, cs_dqall);

  casadi::Function KE_eval(
    "KE", SXVector{cs_q0, cs_q1}, SXVector{substitute(KE_expr, cs_dqall, zero_vec)});
  casadi::Function gKE_eval(
    "gKE", SXVector{cs_q0, cs_q1}, SXVector{substitute(gKE_expr, cs_dqall, zero_vec)});
  casadi::Function HKE_eval(
    "HKE", SXVector{cs_q0, cs_q1}, SXVector{substitute(HKE_expr, cs_dqall, zero_vec)});
  std::cout << HKE_eval << std::endl;

  auto q0 = pinocchio::neutral(model);
  auto q1 = pinocchio::randomConfiguration(model);
  auto q2 = pinocchio::randomConfiguration(model);
  casadi::DM q0d(nq);
  pinocchio::casadi::copy(q0, q0d);
  casadi::DM q1d(nq);
  pinocchio::casadi::copy(q1, q1d);
  casadi::DM q2d(nq);
  pinocchio::casadi::copy(q2, q2d);

  const static double eps = 1e-8;

  auto fd_grad_lambda =
    [&model, &KE_eval](const Model::ConfigVectorType q0_, const Model::ConfigVectorType & q1_) {
      auto nv = model.nv;
      // finite differencing
      Model::TangentVectorType dq0(nv), dq1(nv);
      dq0.setZero();
      dq1.setZero();
      Eigen::VectorXd jac_fd(2 * nv);

      const casadi::DM dm = KE_eval(DMVector{eigenToDM(q0_), eigenToDM(q1_)})[0];
      for (int i = 0; i < nv; i++)
      {
        dq0[i] = eps;
        dq1[i] = eps;

        Model::ConfigVectorType q0_i = pinocchio::integrate(model, q0_, dq0);
        //      std::cout << "\nq0_i: " << q0_i.transpose() << std::endl;
        casadi::DM dp1 = KE_eval(DMVector{eigenToDM(q0_i), eigenToDM(q1_)})[0];
        //      std::cout << "dp1: " << dp1 << std::endl;
        casadi::DM diff1 = (dp1 - dm) / eps;

        Model::ConfigVectorType q1_i = pinocchio::integrate(model, q1_, dq1);
        casadi::DM dp2 = KE_eval(DMVector{eigenToDM(q0_), eigenToDM(q1_i)})[0];
        //      std::cout << "dp2: " << dp2 << std::endl;
        casadi::DM diff2 = (dp2 - dm) / eps;

        jac_fd[i] = static_cast<double>(diff1);
        jac_fd[i + nv] = static_cast<double>(diff2);

        dq0[i] = 0.;
        dq1[i] = 0.;
      }
      return jac_fd;
    };

  std::cout << "eval: {q0d,q0d}: " << KE_eval(DMVector{q0d, q0d}) << std::endl;
  std::cout << "grad: {q0d,q0d}: " << gKE_eval(DMVector{q0d, q0d}) << std::endl;
  std::cout << "FD grad: {q0d,q0d}:" << fd_grad_lambda(q0, q0).transpose() << std::endl;
  std::cout << "---" << std::endl;

  std::cout << "eval: {q1d,q1d}: " << KE_eval(DMVector{q1d, q1d}) << std::endl;
  std::cout << "grad: {q1d,q1d}: " << gKE_eval(DMVector{q1d, q1d}) << std::endl;
  std::cout << "FD grad: {q1d,q1d}:" << fd_grad_lambda(q1, q1).transpose() << std::endl;
  std::cout << "---" << std::endl;

  std::cout << "eval: {q1d,q2d}: " << KE_eval(DMVector{q1d, q2d}) << std::endl;
  std::cout << "grad: {q1d,q2d}: " << gKE_eval(DMVector{q1d, q2d}) << std::endl;
  std::cout << "FD grad: {q1d,q2d}:" << fd_grad_lambda(q1, q2).transpose() << std::endl;
  std::cout << "---" << std::endl;

  std::cout << "eval: {q2d,q2d}: " << KE_eval(DMVector{q2d, q2d}) << std::endl;
  std::cout << "grad: {q2d,q2d}: " << gKE_eval(DMVector{q2d, q2d}) << std::endl;
  std::cout << "FD grad: {q2d,q2d}:" << fd_grad_lambda(q2, q2).transpose() << std::endl;
  std::cout << "---" << std::endl;
  std::cout << '\n';

  auto fd_hess_ambda =
    [&model, &gKE_eval](const Model::ConfigVectorType & q0_, const Model::ConfigVectorType & q1_) {
      auto nv = model.nv;
      // finite differencing
      Model::TangentVectorType dq0(nv), dq1(nv);
      dq0.setZero();
      dq1.setZero();
      Eigen::MatrixXd jac_fd(2 * nv, 2 * nv);
      for (int i = 0; i < nv; i++)
      {
        dq0(i, 0) = eps;
        dq1(i, 0) = eps;

        Model::ConfigVectorType q0_i = pinocchio::integrate(model, q0_, dq0);
        auto dp = gKE_eval(DMVector{eigenToDM(q0_i), eigenToDM(q1_)});
        auto dm = gKE_eval(DMVector{eigenToDM(q0_), eigenToDM(q1_)});
        auto diff1 = (dp[0] - dm[0]) / eps;

        Model::ConfigVectorType q1_i = pinocchio::integrate(model, q1_, dq1);
        dp = gKE_eval(DMVector{eigenToDM(q0_), eigenToDM(q1_i)});
        auto diff2 = (dp[0] - dm[0]) / eps;

        for (int j = 0; j < jac_fd.rows(); j++)
        {
          jac_fd(j, i) = static_cast<double>(diff1(j));
          jac_fd(j, i + nv) = static_cast<double>(diff2(j));
        }

        dq0(i, 0) = 0.;
        dq1(i, 0) = 0.;
      }
      return jac_fd;
    };

  std::cout << HKE_eval(DMVector{q0d, q0d})[0] << '\n';
  auto jac_fd = fd_hess_ambda(q0, q0);
  std::cout << jac_fd << '\n';

  std::cout << HKE_eval(DMVector{q1d, q2d})[0] << '\n';
  jac_fd = fd_hess_ambda(q1, q2);
  std::cout << jac_fd << '\n';

  std::cout << HKE_eval(DMVector{q0d, q2d})[0] << '\n';
  jac_fd = fd_hess_ambda(q0, q2);
  std::cout << jac_fd << '\n';

  std::cout << HKE_eval(DMVector{q2d, q2d})[0] << '\n';
  jac_fd = fd_hess_ambda(q2, q2);
  std::cout << jac_fd << '\n';
}

BOOST_AUTO_TEST_SUITE_END()
