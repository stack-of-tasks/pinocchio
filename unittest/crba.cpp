//
// Copyright (c) 2015-2023 CNRS INRIA
//

/*
 * Test the CRBA algorithm. The code validates both the computation times and
 * the value by comparing the results of the CRBA with the reconstruction of
 * the mass matrix using the RNEA.
 * For a strong timing benchmark, see benchmark/timings.
 *
 */

#ifndef EIGEN_RUNTIME_NO_MALLOC
  #define EIGEN_RUNTIME_NO_MALLOC
#endif

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template<typename JointModel>
static void addJointAndBody(
  pinocchio::Model & model,
  const pinocchio::JointModelBase<JointModel> & joint,
  const std::string & parent_name,
  const std::string & name,
  const pinocchio::SE3 placement = pinocchio::SE3::Random(),
  bool setRandomLimits = true)
{
  using namespace pinocchio;
  typedef typename JointModel::ConfigVector_t CV;
  typedef typename JointModel::TangentVector_t TV;

  Model::JointIndex idx;

  if (setRandomLimits)
    idx = model.addJoint(
      model.getJointId(parent_name), joint, SE3::Random(), name + "_joint",
      TV::Random() + TV::Constant(1), TV::Random() + TV::Constant(1),
      CV::Random() - CV::Constant(1), CV::Random() + CV::Constant(1));
  else
    idx = model.addJoint(model.getJointId(parent_name), joint, placement, name + "_joint");

  model.addJointFrame(idx);

  model.appendBodyToJoint(idx, Inertia::Random(), SE3::Identity());
  model.addBodyFrame(name + "_body", idx);
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_crba)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);

#ifdef NDEBUG
  #ifdef _INTENSE_TESTING_
  const size_t NBT = 1000 * 1000;
  #else
  const size_t NBT = 10;
  #endif

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  PinocchioTicToc timer(PinocchioTicToc::US);
  timer.tic();
  SMOOTH(NBT)
  {
    pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  }
  timer.toc(std::cout, NBT);

#else
  const size_t NBT = 1;
  using namespace Eigen;
  using namespace pinocchio;

  Eigen::MatrixXd M(model.nv, model.nv);
  Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);
  q.segment<4>(3).normalize();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  data.M.fill(0);
  pinocchio::crba(model, data, q, Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  /* Joint inertia from iterative crba. */
  const Eigen::VectorXd bias = rnea(model, data, q, v, a);
  for (int i = 0; i < model.nv; ++i)
  {
    M.col(i) = rnea(model, data, q, v, Eigen::VectorXd::Unit(model.nv, i)) - bias;
  }

  // std::cout << "Mcrb = [ " << data.M << "  ];" << std::endl;
  // std::cout << "Mrne = [  " << M << " ]; " << std::endl;
  BOOST_CHECK(M.isApprox(data.M, 1e-12));

  std::cout << "(the time score in debug mode is not relevant)  ";

  q = Eigen::VectorXd::Zero(model.nq);

  PinocchioTicToc timer(PinocchioTicToc::US);
  timer.tic();
  SMOOTH(NBT)
  {
    pinocchio::crba(model, data, q, Convention::WORLD);
  }
  timer.toc(std::cout, NBT);

#endif // ifndef NDEBUG
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
  pinocchio::Data data_full(model_full);
  pinocchio::Data data_mimic(model_mimic);
  pinocchio::Data data_ref_mimic(model_mimic);

  // Prepare test data
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(model_full.nv, model_mimic.nv);
  G.topLeftCorner(secondary_idxv, secondary_idxv).setIdentity();
  G.bottomRightCorner(model_mimic.nv - secondary_idxv, model_mimic.nv - secondary_idxv)
    .setIdentity();
  G(secondary_idxv, primary_idxv) = ratio;

  Eigen::VectorXd q_mimic = pinocchio::randomConfiguration(model_mimic);
  Eigen::VectorXd v_mimic = Eigen::VectorXd::Random(model_mimic.nv);
  Eigen::VectorXd a_mimic = Eigen::VectorXd::Random(model_mimic.nv);

  // World vs local
  pinocchio::crba(model_mimic, data_ref_mimic, q_mimic, pinocchio::Convention::WORLD);
  pinocchio::crba(model_mimic, data_mimic, q_mimic, pinocchio::Convention::LOCAL);

  BOOST_CHECK(data_ref_mimic.M.isApprox(data_mimic.M));

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_full.nq);
  Eigen::VectorXd v_full = G * v_mimic;
  Eigen::VectorXd a_full = G * a_mimic;

  for (int n = 1; n < model_full.njoints; n++)
  {
    const double joint_ratio = ((n == secondary_id) ? ratio : 1.0);
    const double joint_offset = ((n == secondary_id) ? offset : 0.0);
    model_full.joints[n].jointConfigExtendedModelSelector(q_full) =
      joint_ratio * model_mimic.joints[n].jointConfigExtendedModelSelector(q_mimic)
      + joint_offset * Eigen::VectorXd::Ones(model_full.joints[n].nq());
  }

  // // // Run crba
  // // pinocchio::crba(model_mimic, data_mimic, q_mimic);
  pinocchio::crba(model_full, data_full, q_full);

  // Compute other half of matrix
  data_mimic.M.triangularView<Eigen::StrictlyLower>() =
    data_mimic.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_full.M.triangularView<Eigen::StrictlyLower>() =
    data_full.M.transpose().triangularView<Eigen::StrictlyLower>();

  // Check full model against reduced
  BOOST_CHECK(data_mimic.M.isApprox(G.transpose() * data_full.M * G, 1e-12));
}

BOOST_AUTO_TEST_CASE(test_crba_mimic)
{
  pinocchio::Model humanoid_model;
  pinocchio::buildModels::humanoidRandom(humanoid_model, true);

  // Test for direct parent/child joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("rleg4_joint"),
    humanoid_model.getJointId("rleg5_joint"));

  // Test for spaced parent/child joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("rleg1_joint"),
    humanoid_model.getJointId("rleg4_joint"));

  // // Test for parallel joint mimic
  test_mimic_against_full_model(
    humanoid_model, humanoid_model.getJointId("lleg4_joint"),
    humanoid_model.getJointId("rleg4_joint"));
}

BOOST_AUTO_TEST_CASE(test_minimal_crba)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);

  Eigen::VectorXd q =
    randomConfiguration(model, model.lowerPositionLimit, model.upperPositionLimit);
  Eigen::VectorXd v(Eigen::VectorXd::Random(model.nv));

  pinocchio::crba(model, data_ref, q, pinocchio::Convention::LOCAL);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(data.M.isApprox(data_ref.M));

  ccrba(model, data_ref, q, v);
  computeJointJacobians(model, data_ref, q);
  BOOST_CHECK(data.Ag.isApprox(data_ref.Ag));
  BOOST_CHECK(data.J.isApprox(data_ref.J));

  // Check double call
  {
    pinocchio::Data data2(model);
    pinocchio::crba(model, data2, q, pinocchio::Convention::LOCAL);
    pinocchio::crba(model, data2, q, pinocchio::Convention::LOCAL);

    BOOST_CHECK(data2.Ycrb[0] == data.Ycrb[0]);
  }
}

BOOST_AUTO_TEST_CASE(test_roto_inertia_effects)
{
  pinocchio::Model model, model_ref;
  pinocchio::buildModels::humanoidRandom(model);
  model_ref = model;

  BOOST_CHECK(model == model_ref);

  pinocchio::Data data(model), data_ref(model_ref);

  model.armature = Eigen::VectorXd::Random(model.nv) + Eigen::VectorXd::Constant(model.nv, 1.);

  Eigen::VectorXd q = randomConfiguration(model);
  pinocchio::crba(model_ref, data_ref, q, pinocchio::Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.M.diagonal() += model.armature;

  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(data.M.isApprox(data_ref.M));
}

#ifndef NDEBUG

BOOST_AUTO_TEST_CASE(test_crba_malloc)
{
  using namespace pinocchio;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, false, true);

  model.addJoint(
    size_t(model.njoints - 1), pinocchio::JointModelRevoluteUnaligned(SE3::Vector3::UnitX()),
    SE3::Random(), "revolute_unaligned");
  pinocchio::Data data(model);

  const Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::internal::set_is_malloc_allowed(false);
  crba(model, data, q);
  Eigen::internal::set_is_malloc_allowed(true);
}

#endif

BOOST_AUTO_TEST_SUITE_END()
