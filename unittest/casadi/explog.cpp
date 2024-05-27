//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <casadi/casadi.hpp>

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_squaredDistance)
{
  typedef double Scalar;
  typedef casadi::SX ADScalar;
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  using casadi::SXVector;

  Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  typedef Model::ConfigVectorType ConfigVector;
  typedef Eigen::Map<ConfigVector> ConfigVectorMap;

  ConfigVector q0(pinocchio::neutral(model));
  ConfigVector q1(pinocchio::randomConfiguration(model));
  ConfigVector q2(pinocchio::randomConfiguration(model));

  ADModel ad_model = model.cast<ADScalar>();
  typedef ADModel::ConfigVectorType cConfig_t;
  typedef ADModel::TangentVectorType cTangent_t;

  casadi::SX cs_q0 = casadi::SX::sym("q0", model.nq, 1);
  casadi::SX cs_q1 = casadi::SX::sym("q1", model.nq, 1);
  casadi::SX cv0 = casadi::SX::sym("v0", model.nv, 1);
  casadi::SX cv1 = casadi::SX::sym("v1", model.nv, 1);
  casadi::SX v_all = vertcat(cv0, cv1);
  cConfig_t q0_ad(model.nq), q1_ad(model.nq);
  cTangent_t v0_ad(model.nq), v1_ad(model.nv);

  q0_ad = Eigen::Map<cConfig_t>(static_cast<SXVector>(cs_q0).data(), model.nq, 1);
  q1_ad = Eigen::Map<cConfig_t>(static_cast<SXVector>(cs_q1).data(), model.nq, 1);
  v0_ad = Eigen::Map<cTangent_t>(static_cast<SXVector>(cv0).data(), model.nv, 1);
  v1_ad = Eigen::Map<cTangent_t>(static_cast<SXVector>(cv1).data(), model.nv, 1);

  auto cq0_i = pinocchio::integrate(ad_model, q0_ad, v0_ad);
  auto cq1_i = pinocchio::integrate(ad_model, q1_ad, v1_ad);

  cTangent_t d = pinocchio::difference(ad_model, cq0_i, cq1_i);
  casadi::SX d_vec(model.nv);
  pinocchio::casadi::copy(d, d_vec);
  ADScalar dist_expr = d.dot(d);

  size_t nq = (size_t)model.nq;

  casadi::SX zero_vec = casadi::SX::zeros(2 * model.nv);
  casadi::SX Jdist_expr = substitute(gradient(dist_expr, v_all), v_all, zero_vec);
  casadi::Function eval_Jdist("Jdistance", SXVector{cs_q0, cs_q1}, SXVector{Jdist_expr});

  std::cout << "Jdistance func: " << eval_Jdist << '\n';

  std::vector<Scalar> q0_vec(nq);
  ConfigVectorMap(q0_vec.data(), model.nq, 1) = q0;
  std::vector<Scalar> q1_vec(nq);
  ConfigVectorMap(q1_vec.data(), model.nq, 1) = q1;
  std::vector<Scalar> q2_vec(nq);
  ConfigVectorMap(q2_vec.data(), model.nq, 1) = q2;

  auto J0 = eval_Jdist(casadi::DMVector{q0_vec, q0_vec})[0];
  auto J1 = eval_Jdist(casadi::DMVector{q0_vec, q1_vec})[0];
  auto J2 = eval_Jdist(casadi::DMVector{q0_vec, q2_vec})[0];

  ConfigVector q(model.nq);
  std::vector<Scalar> q_vec(nq);
  for (size_t it = 0; it < 10; it++)
  {
    q.noalias() = pinocchio::randomConfiguration(model);
    ConfigVectorMap(q_vec.data(), model.nq, 1) = q;
    auto J_vec = static_cast<std::vector<Scalar>>(eval_Jdist(casadi::DMVector{q_vec, q_vec})[0]);
    Eigen::Map<Eigen::MatrixXd> J_as_mat(J_vec.data(), 2 * model.nv, 1);
    BOOST_CHECK(J_as_mat.isApprox(Eigen::MatrixXd::Zero(2 * model.nv, 1)));
  }
}

template<typename Derived>
casadi::DM SE3toCasadiDM(const pinocchio::SE3Base<Derived> & M)
{
  typedef typename Derived::Scalar Scalar;
  auto M_mat = M.toHomogeneousMatrix();
  std::vector<Scalar> flat_M_vec(M_mat.data(), M_mat.data() + M_mat.size());
  casadi::DM out{flat_M_vec};
  return reshape(out, 4, 4);
}

BOOST_AUTO_TEST_CASE(test_Jlog6)
{
  namespace pin = pinocchio;
  using casadi::DMVector;
  using casadi::SXVector;
  using casadi::SXVectorVector;
  typedef pin::SE3Tpl<double> SE3;
  typedef casadi::SX ADScalar;
  typedef pin::SE3Tpl<ADScalar> cSE3;
  typedef pin::MotionTpl<ADScalar> cMotion;

  casadi::SX csM0 = casadi::SX::sym("cM0", 4, 4);
  casadi::SX csM1 = casadi::SX::sym("cM1", 4, 4);
  casadi::SX csm0 = casadi::SX::sym("cm0", 6);
  casadi::SX csm1 = casadi::SX::sym("cm1", 6);
  casadi::SX csm_all = vertcat(csm0, csm1);

  cMotion::Vector6 cm0_v, cm1_v;

  cSE3 cM0_i, cM1_i;
  {

    cm0_v = Eigen::Map<cMotion::Vector6>(static_cast<SXVector>(csm0).data(), 6, 1);
    cm1_v = Eigen::Map<cMotion::Vector6>(static_cast<SXVector>(csm1).data(), 6, 1);

    cSE3::Matrix4 cM0_mat, cM1_mat;
    cM0_mat = Eigen::Map<cSE3::Matrix4>(&static_cast<SXVector>(csM0)[0], 4, 4);
    cM1_mat = Eigen::Map<cSE3::Matrix4>(&static_cast<SXVector>(csM1)[0], 4, 4);

    auto rot0 = cM0_mat.template block<3, 3>(0, 0);
    auto rot1 = cM1_mat.template block<3, 3>(0, 0);
    auto trans0 = cM0_mat.template block<3, 1>(0, 3);

    cSE3 cM0(rot0, trans0);
    cSE3 cM1(rot1, cM1_mat.template block<3, 1>(0, 3));

    cM0_i = cM0 * pin::exp6(cm0_v);
    cM1_i = cM1 * pin::exp6(cm1_v);
  }

  auto cdM(pin::log6(cM0_i.actInv(cM1_i)).toVector());
  casadi::SX cdM_s(6);
  pinocchio::casadi::copy(cdM, cdM_s);
  casadi::SX zeros = casadi::SX::zeros(12);

  auto dist = cdM.squaredNorm();
  auto grad_cdM_expr = gradient(dist, csm_all);
  auto hess_cdM_expr = jacobian(grad_cdM_expr, csm_all);

  casadi::Function grad_cdM_eval(
    "gdM", SXVector{csM0, csM1}, SXVector{substitute(grad_cdM_expr, csm_all, zeros)});
  casadi::Function hess_cdM_eval(
    "gdM", SXVector{csM0, csM1}, SXVector{substitute(hess_cdM_expr, csm_all, zeros)});

  std::cout << "dM_eval: " << grad_cdM_eval << '\n';

  SE3 M_neutral(SE3::Identity());
  SE3 M0(SE3::Random()), M1(SE3::Random());

  casadi::DM M_n_dm = SE3toCasadiDM(M_neutral);
  casadi::DM M0_dm = SE3toCasadiDM(M0);
  casadi::DM M1_dm = SE3toCasadiDM(M1);
  casadi::DM M2_dm = SE3toCasadiDM(M1.actInv(M1));
  std::cout << M0_dm << "\n\n";
  std::cout << M2_dm << '\n';

  auto J0 = grad_cdM_eval(DMVector{M0_dm, M1_dm})[0];
  std::cout << J0 << '\n';

  auto J1 = grad_cdM_eval(DMVector{M0_dm, M2_dm})[0];
  std::cout << J1 << '\n';

  auto J2 = grad_cdM_eval(DMVector{M2_dm, M2_dm})[0];
  std::cout << J2 << '\n';

  std::cout << hess_cdM_eval(DMVector{M_n_dm, M_n_dm})[0];
  std::cout << hess_cdM_eval(DMVector{M0_dm, M0_dm})[0];
  std::cout << hess_cdM_eval(DMVector{M0_dm, M1_dm})[0];
  std::cout << hess_cdM_eval(DMVector{M2_dm, M2_dm})[0];
}

BOOST_AUTO_TEST_SUITE_END()
