//
// Copyright (c) 2015-2020 CNRS INRIA
//

/*
 * Unittest of the RNE algorithm. The code simply test that the algorithm does
 * not cause any serious errors. The numerical values are not cross validated
 * in any way.
 *
 */

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

// #define __SSE3__
#include <fenv.h>

#ifdef __SSE3__
  #include <pmmintrin.h>
#endif

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rnea)
{
#ifdef __SSE3__
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  pinocchio::Data data(model);
  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;

  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

#ifdef NDEBUG
  const size_t NBT = 10000;
#else
  const size_t NBT = 1;
  std::cout << "(the time score in debug mode is not relevant)  ";
#endif

  PinocchioTicToc timer(PinocchioTicToc::US);
  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model, data, q, v, a);
  }
  timer.toc(std::cout, NBT);
}

BOOST_AUTO_TEST_CASE(test_nle_vs_rnea)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  pinocchio::Data data_nle(model);
  pinocchio::Data data_rnea(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);

  VectorXd tau_nle(VectorXd::Zero(model.nv));
  VectorXd tau_rnea(VectorXd::Zero(model.nv));

  // -------
  q.tail(model.nq - 7).setZero();
  v.setZero();

  tau_nle = nonLinearEffects(model, data_nle, q, v);
  tau_rnea = rnea(model, data_rnea, q, v, VectorXd::Zero(model.nv));

  BOOST_CHECK(tau_nle.isApprox(tau_rnea, 1e-12));

  // -------
  q.tail(model.nq - 7).setZero();
  v.setOnes();

  tau_nle = nonLinearEffects(model, data_nle, q, v);
  tau_rnea = rnea(model, data_rnea, q, v, VectorXd::Zero(model.nv));

  BOOST_CHECK(tau_nle.isApprox(tau_rnea, 1e-12));

  // -------
  q.tail(model.nq - 7).setOnes();
  v.setOnes();

  tau_nle = nonLinearEffects(model, data_nle, q, v);
  tau_rnea = rnea(model, data_rnea, q, v, VectorXd::Zero(model.nv));

  BOOST_CHECK(tau_nle.isApprox(tau_rnea, 1e-12));

  // -------
  q = randomConfiguration(model);
  v.setRandom();

  tau_nle = nonLinearEffects(model, data_nle, q, v);
  tau_rnea = rnea(model, data_rnea, q, v, VectorXd::Zero(model.nv));

  BOOST_CHECK(tau_nle.isApprox(tau_rnea, 1e-12));
}

BOOST_AUTO_TEST_CASE(test_rnea_with_fext)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data_rnea_fext(model);
  Data data_rnea(model);

  VectorXd q = randomConfiguration(model);

  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  PINOCCHIO_ALIGNED_STD_VECTOR(Force) fext(model.joints.size(), Force::Zero());

  JointIndex rf = model.getJointId("rleg6_joint");
  Force Frf = Force::Random();
  fext[rf] = Frf;
  JointIndex lf = model.getJointId("lleg6_joint");
  Force Flf = Force::Random();
  fext[lf] = Flf;

  rnea(model, data_rnea, q, v, a);
  VectorXd tau_ref(data_rnea.tau);
  Data::Matrix6x Jrf(Data::Matrix6x::Zero(6, model.nv));
  computeJointJacobian(model, data_rnea, q, rf, Jrf);
  tau_ref -= Jrf.transpose() * Frf.toVector();

  Data::Matrix6x Jlf(Data::Matrix6x::Zero(6, model.nv));
  computeJointJacobian(model, data_rnea, q, lf, Jlf);
  tau_ref -= Jlf.transpose() * Flf.toVector();

  rnea(model, data_rnea_fext, q, v, a, fext);

  BOOST_CHECK(tau_ref.isApprox(data_rnea_fext.tau));
}

BOOST_AUTO_TEST_CASE(test_rnea_with_armature)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);
  model.armature = VectorXd::Random(model.nv) + VectorXd::Ones(model.nv);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data(model);
  Data data_ref(model);

  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<StrictlyLower>() =
    data_ref.M.transpose().triangularView<StrictlyLower>();
  const VectorXd nle = nonLinearEffects(model, data_ref, q, v);

  const VectorXd tau_ref = data_ref.M * a + nle;

  rnea(model, data, q, v, a);
  BOOST_CHECK(tau_ref.isApprox(data.tau));
}

BOOST_AUTO_TEST_CASE(test_compute_gravity)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data_rnea(model);
  Data data(model);

  VectorXd q = randomConfiguration(model);

  rnea(model, data_rnea, q, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv));
  computeGeneralizedGravity(model, data, q);

  BOOST_CHECK(data_rnea.tau.isApprox(data.g));

  // Compare with Jcom
  crba(model, data_rnea, q, Convention::WORLD);
  Data::Matrix3x Jcom = getJacobianComFromCrba(model, data_rnea);

  VectorXd g_ref(-data_rnea.mass[0] * Jcom.transpose() * Model::gravity981);

  BOOST_CHECK(g_ref.isApprox(data.g));
}

BOOST_AUTO_TEST_CASE(test_compute_static_torque)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data_rnea(model);
  Data data(model);

  VectorXd q = randomConfiguration(model);

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  ForceVector fext((size_t)model.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  rnea(model, data_rnea, q, VectorXd::Zero(model.nv), VectorXd::Zero(model.nv), fext);
  computeStaticTorque(model, data, q, fext);

  BOOST_CHECK(data_rnea.tau.isApprox(data.tau));

  // Compare with Jcom + Jacobian of joint
  crba(model, data_rnea, q, Convention::WORLD);
  Data::Matrix3x Jcom = getJacobianComFromCrba(model, data_rnea);

  VectorXd static_torque_ref = -data_rnea.mass[0] * Jcom.transpose() * Model::gravity981;
  computeJointJacobians(model, data_rnea, q);

  Data::Matrix6x J_local(6, model.nv);
  for (JointIndex joint_id = 1; joint_id < (JointIndex)(model.njoints); ++joint_id)
  {
    J_local.setZero();
    getJointJacobian(model, data_rnea, joint_id, LOCAL, J_local);
    static_torque_ref -= J_local.transpose() * fext[joint_id].toVector();
  }

  BOOST_CHECK(static_torque_ref.isApprox(data.tau));
}

BOOST_AUTO_TEST_CASE(test_compute_coriolis)
{
  using namespace Eigen;
  using namespace pinocchio;

  const double prec = Eigen::NumTraits<double>::dummy_precision();

  Model model;
  buildModels::humanoidRandom(model, true);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  Data data_ref(model);
  Data data(model);

  VectorXd q = randomConfiguration(model);

  VectorXd v(VectorXd::Random(model.nv));
  computeCoriolisMatrix(model, data, q, Eigen::VectorXd::Zero(model.nv));
  BOOST_CHECK(data.C.isZero(prec));

  model.gravity.setZero();
  rnea(model, data_ref, q, v, VectorXd::Zero(model.nv));
  computeJointJacobiansTimeVariation(model, data_ref, q, v);
  computeCoriolisMatrix(model, data, q, v);

  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.J.isApprox(data_ref.J));

  VectorXd tau = data.C * v;
  BOOST_CHECK(tau.isApprox(data_ref.tau, prec));

  dccrba(model, data_ref, q, v);
  crba(model, data_ref, q, Convention::WORLD);

  const Data::Vector3 & com = data_ref.com[0];
  Motion vcom(data_ref.vcom[0], Data::Vector3::Zero());
  SE3 cM1(data.oMi[1]);
  cM1.translation() -= com;

  BOOST_CHECK((cM1.toDualActionMatrix() * data_ref.M.topRows<6>()).isApprox(data_ref.Ag, prec));

  Force dh_ref = cM1.act(Force(data_ref.tau.head<6>()));
  Force dh(data_ref.dAg * v);
  BOOST_CHECK(dh.isApprox(dh_ref, prec));

  {
    Data data_ref(model), data_ref_plus(model);
    Eigen::MatrixXd dM(data.C + data.C.transpose());

    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model, q, alpha * v);

    crba(model, data_ref, q, Convention::WORLD);
    data_ref.M.triangularView<Eigen::StrictlyLower>() =
      data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
    crba(model, data_ref_plus, q_plus, Convention::WORLD);
    data_ref_plus.M.triangularView<Eigen::StrictlyLower>() =
      data_ref_plus.M.transpose().triangularView<Eigen::StrictlyLower>();

    Eigen::MatrixXd dM_ref = (data_ref_plus.M - data_ref.M) / alpha;
    BOOST_CHECK(dM.isApprox(dM_ref, sqrt(alpha)));
  }
}

void test_mimic_against_full_model(
  const pinocchio::Model & model_full,
  const pinocchio::JointIndex & primary_id,
  const pinocchio::JointIndex & secondary_id)
{
  // constants
  const int primary_idxq = model_full.joints[primary_id].idx_q();
  const int primary_idxv = model_full.joints[primary_id].idx_v();
  const int secondary_idxq = model_full.joints[secondary_id].idx_q();
  const int secondary_idxv = model_full.joints[secondary_id].idx_v();
  const double ratio = 2.5;
  const double offset = 0.75;

  // Build mimic model
  pinocchio::Model model_mimic;
  pinocchio::transformJointIntoMimic(
    model_full, primary_id, secondary_id, ratio, offset, model_mimic);
  pinocchio::Data data_nle_mimic(model_mimic);
  pinocchio::Data data_rnea_mimic(model_mimic);
  pinocchio::Data data_full(model_full);

  // Prepare test data
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(model_full.nv, model_mimic.nv);
  G.topLeftCorner(secondary_idxv, secondary_idxv).setIdentity();
  G.bottomRightCorner(model_mimic.nv - secondary_idxv, model_mimic.nv - secondary_idxv)
    .setIdentity();
  G(secondary_idxv, primary_idxv) = ratio;

  Eigen::VectorXd q = pinocchio::randomConfiguration(model_mimic);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model_mimic.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model_mimic.nv);

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_full.nq);
  Eigen::VectorXd v_full = G * v;
  Eigen::VectorXd a_full = G * v;

  for (int n = 1; n < model_full.njoints; n++)
  {
    const double joint_ratio = ((n == secondary_id) ? ratio : 1.0);
    const double joint_offset = ((n == secondary_id) ? offset : 0.0);
    model_full.joints[n].jointConfigExtendedModelSelector(q_full) =
      joint_ratio * model_mimic.joints[n].jointConfigExtendedModelSelector(q)
      + joint_offset * Eigen::VectorXd::Ones(model_full.joints[n].nq());
  }

  pinocchio::crba(model_full, data_full, q_full, pinocchio::Convention::WORLD);
  data_full.M.triangularView<Eigen::StrictlyLower>() =
    data_full.M.transpose().triangularView<Eigen::StrictlyLower>();
  const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_full, data_full, q_full, v_full);

  // // Use equation of motion to compute tau from a_reduced
  Eigen::VectorXd tau_ref = G.transpose() * data_full.M * G * a + (G.transpose() * nle);

  pinocchio::rnea(model_mimic, data_rnea_mimic, q, v, a);
  BOOST_CHECK(tau_ref.isApprox(data_rnea_mimic.tau));

  // NLE
  pinocchio::Data data_nle(model_mimic);
  pinocchio::Data data_ref_nle(model_mimic);
  Eigen::VectorXd tau_ref_nle =
    pinocchio::rnea(model_mimic, data_ref_nle, q, v, Eigen::VectorXd::Zero(model_mimic.nv));
  Eigen::VectorXd tau_nle = pinocchio::nonLinearEffects(model_mimic, data_nle, q, v);
  BOOST_CHECK(tau_nle.isApprox(tau_nle));

  // Generalized Gravity
  pinocchio::Data data_ref_gg(model_mimic);
  pinocchio::Data data_gg(model_mimic);
  Eigen::VectorXd tau_ref_gg = pinocchio::rnea(
    model_mimic, data_ref_gg, q, Eigen::VectorXd::Zero(model_mimic.nv),
    Eigen::VectorXd::Zero(model_mimic.nv));
  Eigen::VectorXd tau_gg = pinocchio::computeGeneralizedGravity(model_mimic, data_gg, q);
  BOOST_CHECK(tau_gg.isApprox(tau_ref_gg));

  // Static Torque
  typedef PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) ForceVector;
  ForceVector fext((size_t)model_mimic.njoints);
  for (ForceVector::iterator it = fext.begin(); it != fext.end(); ++it)
    (*it).setRandom();

  pinocchio::Data data_ref_st(model_mimic);
  pinocchio::Data data_st(model_mimic);
  pinocchio::rnea(
    model_mimic, data_ref_st, q, Eigen::VectorXd::Zero(model_mimic.nv),
    Eigen::VectorXd::Zero(model_mimic.nv), fext);
  Eigen::VectorXd tau_st = pinocchio::computeStaticTorque(model_mimic, data_gg, q, fext);
  BOOST_CHECK(tau_st.isApprox(data_ref_st.tau));
}

BOOST_AUTO_TEST_CASE(test_rnea_mimic)
{
  pinocchio::Model humanoid_model;
  pinocchio::buildModels::humanoidRandom(humanoid_model);

  // Test for direct parent/child joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("rleg1_joint"),
    humanoid_model.getJointId("rleg2_joint"));

  // Test for spaced parent/child joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("rleg1_joint"),
    humanoid_model.getJointId("rleg4_joint"));

  // // Test for parallel joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("lleg4_joint"),
    humanoid_model.getJointId("rleg4_joint"));
}

BOOST_AUTO_TEST_SUITE_END()
