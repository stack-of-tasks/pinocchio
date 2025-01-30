//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "utils/model-generator.hpp"

template<typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived> & x)
{
  return ((x - x).array() == (x - x).array()).all();
}

void test_jacobian_impl(const pinocchio::Model & model, pinocchio::JointIndex joint_id)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Data data(model);

  VectorXd q = pinocchio::neutral(model);
  computeJointJacobians(model, data, q);

  Data::Matrix6x Jrh(6, model.nv);
  Jrh.fill(0);
  getJointJacobian(model, data, joint_id, WORLD, Jrh);

  /* Test J*q == v */
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Zero(model.nv);
  rnea(model, data, q, qdot, qddot);
  Motion v = data.oMi[joint_id].act(data.v[joint_id]);
  BOOST_CHECK(v.toVector().isApprox(Jrh * qdot, 1e-12));

  /* Test local jacobian: rhJrh == rhXo oJrh */
  Data::Matrix6x rhJrh(6, model.nv);
  rhJrh.fill(0);
  getJointJacobian(model, data, joint_id, LOCAL, rhJrh);
  Data::Matrix6x XJrh(6, model.nv);
  motionSet::se3Action(data.oMi[joint_id].inverse(), Jrh, XJrh);
  BOOST_CHECK(XJrh.isApprox(rhJrh, 1e-12));

  XJrh.setZero();
  Data data_jointJacobian(model);
  computeJointJacobian(model, data_jointJacobian, q, joint_id, XJrh);
  BOOST_CHECK(XJrh.isApprox(rhJrh, 1e-12));

  /* Test local world-aligned jacobian */
  forwardKinematics(model, data, q, qdot);
  Data::Matrix6x WJrh = Data::Matrix6x::Zero(6, model.nv);
  getJointJacobian(model, data, joint_id, LOCAL_WORLD_ALIGNED, WJrh);
  Motion w_v = getVelocity(model, data, joint_id, LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(w_v.toVector().isApprox(WJrh * qdot));

  /* Test computeJointJacobians with pre-computation of the forward kinematics */
  Data data_fk(model);
  forwardKinematics(model, data_fk, q);
  computeJointJacobians(model, data_fk);

  BOOST_CHECK(data_fk.J.isApprox(data.J));
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_jacobian)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, false);

  for (int j = 1; j < model.njoints; j++)
  {
    test_jacobian_impl(model, static_cast<pinocchio::JointIndex>(j));
  }
}

BOOST_AUTO_TEST_CASE(test_jacobian_mimic)
{
  for (int i = 0; i < pinocchio::MimicTestCases::N_CASES; i++)
  {
    const pinocchio::MimicTestCases mimic_test_case(i);
    const pinocchio::Model & model_mimic = mimic_test_case.model_mimic;
    for (int j = 1; j < model_mimic.njoints; j++)
    {
      test_jacobian_impl(model_mimic, static_cast<pinocchio::JointIndex>(j));
    }
  }
}

BOOST_AUTO_TEST_CASE(test_jacobian_time_variation)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true, true);
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  VectorXd q = randomConfiguration(
    model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq));
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  computeJointJacobiansTimeVariation(model, data, q, v);

  BOOST_CHECK(isFinite(data.dJ));

  forwardKinematics(model, data_ref, q, v, a);
  Model::Index idx =
    model.existJointName("rarm2") ? model.getJointId("rarm2") : (Model::Index)(model.njoints - 1);

  Data::Matrix6x J(6, model.nv);
  J.fill(0.);
  Data::Matrix6x dJ(6, model.nv);
  dJ.fill(0.);

  // Regarding to the WORLD origin
  getJointJacobian(model, data, idx, WORLD, J);
  BOOST_CHECK(J.isApprox(getJointJacobian(model, data, idx, WORLD)));
  getJointJacobianTimeVariation(model, data, idx, WORLD, dJ);
  Motion v_idx(J * v);
  BOOST_CHECK(v_idx.isApprox(data_ref.oMi[idx].act(data_ref.v[idx])));

  Motion a_idx(J * a + dJ * v);
  const Motion & a_ref = data_ref.oMi[idx].act(data_ref.a[idx]);
  BOOST_CHECK(a_idx.isApprox(a_ref));

  // Regarding to the LOCAL frame
  getJointJacobian(model, data, idx, LOCAL, J);
  BOOST_CHECK(J.isApprox(getJointJacobian(model, data, idx, LOCAL)));
  getJointJacobianTimeVariation(model, data, idx, LOCAL, dJ);

  v_idx = (Motion::Vector6)(J * v);
  BOOST_CHECK(v_idx.isApprox(data_ref.v[idx]));

  a_idx = (Motion::Vector6)(J * a + dJ * v);
  BOOST_CHECK(a_idx.isApprox(data_ref.a[idx]));

  // Regarding to the LOCAL_WORLD_ALIGNED frame
  getJointJacobian(model, data, idx, LOCAL_WORLD_ALIGNED, J);
  BOOST_CHECK(J.isApprox(getJointJacobian(model, data, idx, LOCAL_WORLD_ALIGNED)));
  getJointJacobianTimeVariation(model, data, idx, LOCAL_WORLD_ALIGNED, dJ);

  Data::SE3 worldMlocal = data.oMi[idx];
  worldMlocal.translation().setZero();
  Data::Motion world_v_local = data.ov[idx];
  world_v_local.linear().setZero();

  v_idx = (Motion::Vector6)(J * v);
  BOOST_CHECK(v_idx.isApprox(worldMlocal.act(data_ref.v[idx])));

  a_idx = (Motion::Vector6)(J * a + dJ * v);
  BOOST_CHECK(a_idx.isApprox(
    world_v_local.cross(worldMlocal.act(data_ref.v[idx])) + worldMlocal.act(data_ref.a[idx])));

  // compare to finite differencies : WORLD
  {
    Data data_ref(model), data_ref_plus(model);

    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model, q, alpha * v);

    Data::Matrix6x J_ref(6, model.nv);
    J_ref.fill(0.);
    computeJointJacobians(model, data_ref, q);
    getJointJacobian(model, data_ref, idx, WORLD, J_ref);

    Data::Matrix6x J_ref_plus(6, model.nv);
    J_ref_plus.fill(0.);
    computeJointJacobians(model, data_ref_plus, q_plus);
    getJointJacobian(model, data_ref_plus, idx, WORLD, J_ref_plus);

    Data::Matrix6x dJ_ref(6, model.nv);
    dJ_ref.fill(0.);
    dJ_ref = (J_ref_plus - J_ref) / alpha;

    computeJointJacobiansTimeVariation(model, data, q, v);
    Data::Matrix6x dJ(6, model.nv);
    dJ.fill(0.);
    getJointJacobianTimeVariation(model, data, idx, WORLD, dJ);

    BOOST_CHECK(dJ.isApprox(dJ_ref, sqrt(alpha)));
  }
  // compare to finite differencies : LOCAL
  {
    Data data_ref(model), data_ref_plus(model);

    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model, q, alpha * v);

    Data::Matrix6x J_ref(6, model.nv);
    J_ref.fill(0.);
    computeJointJacobians(model, data_ref, q);
    getJointJacobian(model, data_ref, idx, LOCAL, J_ref);

    Data::Matrix6x J_ref_plus(6, model.nv);
    J_ref_plus.fill(0.);
    computeJointJacobians(model, data_ref_plus, q_plus);
    getJointJacobian(model, data_ref_plus, idx, LOCAL, J_ref_plus);

    Data::Matrix6x dJ_ref(6, model.nv);
    dJ_ref = (J_ref_plus - J_ref) / alpha;

    computeJointJacobiansTimeVariation(model, data, q, v);
    Data::Matrix6x dJ(6, model.nv);
    dJ.fill(0.);
    getJointJacobianTimeVariation(model, data, idx, LOCAL, dJ);

    BOOST_CHECK(dJ.isApprox(dJ_ref, sqrt(alpha)));
  }

  // compare to finite differencies : LOCAL_WORLD_ALIGNED
  {
    Data data_ref(model), data_ref_plus(model);

    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model, q, alpha * v);

    Data::Matrix6x J_ref(6, model.nv);
    J_ref.fill(0.);
    computeJointJacobians(model, data_ref, q);
    getJointJacobian(model, data_ref, idx, LOCAL_WORLD_ALIGNED, J_ref);

    Data::Matrix6x J_ref_plus(6, model.nv);
    J_ref_plus.fill(0.);
    computeJointJacobians(model, data_ref_plus, q_plus);
    getJointJacobian(model, data_ref_plus, idx, LOCAL_WORLD_ALIGNED, J_ref_plus);

    Data::Matrix6x dJ_ref(6, model.nv);
    dJ_ref = (J_ref_plus - J_ref) / alpha;

    computeJointJacobiansTimeVariation(model, data, q, v);
    Data::Matrix6x dJ(6, model.nv);
    dJ.fill(0.);
    getJointJacobianTimeVariation(model, data, idx, LOCAL_WORLD_ALIGNED, dJ);

    BOOST_CHECK(dJ.isApprox(dJ_ref, sqrt(alpha)));
  }
}

BOOST_AUTO_TEST_CASE(test_timings)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);

  long flag = BOOST_BINARY(1111);
  PinocchioTicToc timer(PinocchioTicToc::US);
#ifdef NDEBUG
  #ifdef _INTENSE_TESTING_
  const size_t NBT = 1000 * 1000;
  #else
  const size_t NBT = 10;
  #endif
#else
  const size_t NBT = 1;
  std::cout << "(the time score in debug mode is not relevant)  ";
#endif

  bool verbose = flag & (flag - 1); // True is two or more binaries of the flag are 1.
  if (verbose)
    std::cout << "--" << std::endl;
  Eigen::VectorXd q = pinocchio::neutral(model);

  if (flag >> 0 & 1)
  {
    timer.tic();
    SMOOTH(NBT)
    {
      computeJointJacobians(model, data, q);
    }
    if (verbose)
      std::cout << "Compute =\t";
    timer.toc(std::cout, NBT);
  }

  if (flag >> 1 & 1)
  {
    computeJointJacobians(model, data, q);
    Model::Index idx =
      model.existJointName("rarm6") ? model.getJointId("rarm6") : (Model::Index)(model.njoints - 1);
    Data::Matrix6x Jrh(6, model.nv);
    Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJointJacobian(model, data, idx, WORLD, Jrh);
    }
    if (verbose)
      std::cout << "Copy =\t";
    timer.toc(std::cout, NBT);
  }

  if (flag >> 2 & 1)
  {
    computeJointJacobians(model, data, q);
    Model::Index idx =
      model.existJointName("rarm6") ? model.getJointId("rarm6") : (Model::Index)(model.njoints - 1);
    Data::Matrix6x Jrh(6, model.nv);
    Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJointJacobian(model, data, idx, LOCAL, Jrh);
    }
    if (verbose)
      std::cout << "Change frame =\t";
    timer.toc(std::cout, NBT);
  }

  if (flag >> 3 & 1)
  {
    computeJointJacobians(model, data, q);
    Model::Index idx =
      model.existJointName("rarm6") ? model.getJointId("rarm6") : (Model::Index)(model.njoints - 1);
    Data::Matrix6x Jrh(6, model.nv);
    Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      computeJointJacobian(model, data, q, idx, Jrh);
    }
    if (verbose)
      std::cout << "Single jacobian =\t";
    timer.toc(std::cout, NBT);
  }
}

BOOST_AUTO_TEST_SUITE_END()
