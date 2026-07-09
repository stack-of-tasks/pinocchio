//
// Copyright (c) 2024-2025 INRIA
//

#define BOOST_TEST_MODULE loop_constrained_aba

#include <iostream>

#include "pinocchio/spatial.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/loop-constrained-aba.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

void build_trident_model(Model & model)
{

  Inertia I1 = Inertia::Random();
  Inertia I2 = Inertia::Random();
  model.gravity.linear() = model.gravity981;

  JointModel branch_connector = JointModelRX();

  JointIndex joint0 = model.addJoint(0, JointModelFreeFlyer(), SE3::Random(), "joint0");
  model.appendBodyToJoint(joint0, I1, SE3::Random());

  int num_children = 10;

  JointIndex parent_id;
  std::string joint_name;
  JointIndex mid_chain_joint;

  JointIndex joint1 = model.addJoint(joint0, branch_connector, SE3::Random(), "joint1");
  parent_id = joint1;
  model.appendBodyToJoint(joint1, I1, SE3::Random());
  for (int k = 1; k <= num_children; ++k)
  {
    joint_name = "joint1" + std::to_string(k);
    parent_id = model.addJoint(parent_id, JointModelRX(), SE3::Random(), joint_name);
    model.appendBodyToJoint(parent_id, Inertia::Random(), SE3::Random());

    if (k == 3)
      mid_chain_joint = parent_id;
  }

  JointIndex joint2 = model.addJoint(mid_chain_joint, branch_connector, SE3::Random(), "joint2");
  parent_id = joint2;
  model.appendBodyToJoint(joint2, I2, SE3::Random());
  for (int k = 1; k <= num_children; ++k)
  {
    joint_name = "joint2" + std::to_string(k);
    parent_id = model.addJoint(parent_id, JointModelRX(), SE3::Random(), joint_name);
    model.appendBodyToJoint(parent_id, Inertia::Random(), SE3::Random());

    if (k == 3)
      mid_chain_joint = parent_id;
  }

  JointIndex joint3 = model.addJoint(mid_chain_joint, branch_connector, SE3::Random(), "joint3");
  parent_id = joint3;
  model.appendBodyToJoint(joint3, Inertia::Random(), SE3::Random());
  for (int k = 1; k <= num_children; ++k)
  {
    joint_name = "joint3" + std::to_string(k);
    parent_id = model.addJoint(parent_id, JointModelRX(), SE3::Random(), joint_name);
    model.appendBodyToJoint(parent_id, Inertia::Random(), SE3::Random());
  }

  model.lowerPositionLimit.fill(-5.);
  model.upperPositionLimit.fill(5.);
}

BOOST_AUTO_TEST_CASE(test_6D_unconstrained)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-14, mu0, 1);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_6D_descendants)
{
  Model model;

  build_trident_model(model);
  // model.gravity.setZero();
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint17"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  const double mu0 = 1e-5;
  ProximalSettings prox_settings_ref(1e-14, mu0, 100);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
  std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
  std::cout << "|| data_ref.ddq - data.ddq ||: " << (data_ref.ddq - data.ddq).norm() << std::endl;
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_6D_descendants_reversed)
{
  Model model;
  typedef std::vector<RigidConstraintModel> ConstraintModelVector;
  typedef std::vector<RigidConstraintData> ConstraintDataVector;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  ConstraintModelVector contact_models, contact_models_reversed;
  ConstraintDataVector contact_datas, contact_datas_ref, contact_datas_reversed,
    contact_datas_reversed_ref;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint17"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());
  contact_datas_ref.push_back(rcm1.createData());

  const double mu0 = 1e-5;
  ProximalSettings prox_settings_ref(1e-14, mu0, 100);
  ProximalSettings prox_settings(prox_settings_ref), prox_settings_reversed(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  std::cout << "prox_settings_ref.iter: " << prox_settings_ref.iter << std::endl;
  std::cout << "prox_settings.iter: " << prox_settings.iter << std::endl;
  std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
  std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
  std::cout << "|| data_ref.ddq - data.ddq ||: " << (data_ref.ddq - data.ddq).norm() << std::endl;
  std::cout << "---" << std::endl;

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-8));

  // Check reversed

  Data data_reversed(model);
  RigidConstraintModel rcm1_reversed = RigidConstraintModel(
    CONTACT_6D, model, rcm1.joint2_id, rcm1.joint2_placement, rcm1.joint1_id, rcm1.joint1_placement,
    LOCAL);
  contact_models_reversed.push_back(rcm1_reversed);
  contact_datas_reversed.push_back(rcm1_reversed.createData());
  contact_datas_reversed_ref.push_back(rcm1_reversed.createData());

  initConstraintDynamics(model, data_ref, contact_models_reversed, contact_datas_reversed_ref);
  constraintDynamics(
    model, data_ref, q, v, tau, contact_models_reversed, contact_datas_reversed_ref,
    prox_settings_ref);

  computeJointMinimalOrdering(model, data_reversed, contact_models_reversed);
  lcaba(
    model, data_reversed, q, v, tau, contact_models_reversed, contact_datas_reversed,
    prox_settings_reversed);

  std::cout << "prox_settings_ref.iter: " << prox_settings_ref.iter << std::endl;
  std::cout << "prox_settings_reversed.iter: " << prox_settings_reversed.iter << std::endl;
  std::cout << "data_reversed.ddq: " << data_reversed.ddq.transpose() << std::endl;
  std::cout << "data_ref.ddq: " << data_ref.ddq.transpose() << std::endl;
  std::cout << "data.ddq: " << data.ddq.transpose() << std::endl;
  std::cout << "|| data_reversed.ddq - data.ddq ||: " << (data_reversed.ddq - data.ddq).norm()
            << std::endl;

  BOOST_CHECK(data_ref.ddq.isApprox(data_reversed.ddq, 1e-8));
  BOOST_CHECK(contact_datas_reversed.back().c1Mc2.isApprox(contact_datas.back().c1Mc2.inverse()));

  // Check
  for (JointIndex j = 1; j < size_t(model.njoints); ++j)
  {
    if (data.constraints_supported_dim[j] == 0)
      BOOST_CHECK(data.joint_neighbours[j].size() == 0);
  }
}

BOOST_AUTO_TEST_CASE(test_12D_descendants_redundant_reversed)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint27"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint17"), model.getJointId("joint12"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_6D_different_branches)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint210"), model.getJointId("joint38"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_12D_coupled_loop_common_link)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = pinocchio::randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint210"), model.getJointId("joint38"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint38"), model.getJointId("joint18"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  const auto & joint_cross_coupling = data.joint_cross_coupling;
  BOOST_CHECK(joint_cross_coupling.size() > 0);

  for (const auto & cm : contact_models)
  {
    BOOST_CHECK(
      joint_cross_coupling.exists({cm.joint1_id, cm.joint2_id})
      || joint_cross_coupling.exists({cm.joint2_id, cm.joint1_id}));
  }
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_24D_coupling_with_double_ground)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = pinocchio::randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint1"), model.getJointId("joint14"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint1"), model.getJointId("joint24"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  RigidConstraintModel rcm3 =
    RigidConstraintModel(CONTACT_6D, model, model.getJointId("joint24"), LOCAL);
  rcm3.joint1_placement.setRandom();
  rcm3.joint2_placement.setRandom();
  contact_models.push_back(rcm3);
  contact_datas.push_back(rcm3.createData());

  RigidConstraintModel rcm4 =
    RigidConstraintModel(CONTACT_6D, model, model.getJointId("joint14"), LOCAL);
  rcm4.joint1_placement.setRandom();
  rcm4.joint2_placement.setRandom();
  contact_models.push_back(rcm4);
  contact_datas.push_back(rcm4.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_6D_consecutive_links)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint14"), model.getJointId("joint15"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_12D_coupled_on_a_chain)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint19"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint14"), model.getJointId("joint18"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  BOOST_CHECK(data.joint_cross_coupling.exists({4, 6}));
  // BOOST_CHECK(data.joint_cross_coupling.exists({6,4}));
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_12D_cross_coupled_on_a_chain)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint19"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint14"), model.getJointId("joint18"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_24D_cross_coupling)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint11"), model.getJointId("joint39"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint21"), model.getJointId("joint38"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  RigidConstraintModel rcm3 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint19"), model.getJointId("joint29"), LOCAL);
  rcm3.joint1_placement.setRandom();
  rcm3.joint2_placement.setRandom();
  contact_models.push_back(rcm3);
  contact_datas.push_back(rcm3.createData());

  RigidConstraintModel rcm4 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint29"), model.getJointId("joint39"), LOCAL);
  rcm4.joint1_placement.setRandom();
  rcm4.joint2_placement.setRandom();
  contact_models.push_back(rcm4);
  contact_datas.push_back(rcm4.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_6D_cons_baumgarte)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 =
    RigidConstraintModel(CONTACT_6D, model, model.getJointId("joint11"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  rcm1.m_baumgarte_parameters.Kd = 1.;
  rcm1.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint21"), model.getJointId("joint38"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  rcm2.m_baumgarte_parameters.Kd = 1.;
  rcm2.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  RigidConstraintModel rcm3 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint19"), model.getJointId("joint29"), LOCAL);
  rcm3.joint1_placement.setRandom();
  rcm3.joint2_placement.setRandom();
  rcm3.m_baumgarte_parameters.Kd = 1.;
  rcm3.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm3);
  contact_datas.push_back(rcm3.createData());

  RigidConstraintModel rcm4 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint29"), model.getJointId("joint39"), LOCAL);
  rcm4.joint1_placement.setRandom();
  rcm4.joint2_placement.setRandom();
  rcm4.m_baumgarte_parameters.Kd = 1.;
  rcm4.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm4);
  contact_datas.push_back(rcm4.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_3D_cons_baumgarte)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = 0 * VectorXd::Random(model.nv);
  const VectorXd tau = 0 * VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 =
    RigidConstraintModel(CONTACT_3D, model, model.getJointId("joint11"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();
  rcm1.m_baumgarte_parameters.Kd = 1.;
  rcm1.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_3D, model, model.getJointId("joint21"), model.getJointId("joint38"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();
  rcm2.m_baumgarte_parameters.Kd = 1.;
  rcm2.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  RigidConstraintModel rcm3 = RigidConstraintModel(
    CONTACT_3D, model, model.getJointId("joint19"), model.getJointId("joint29"), LOCAL);
  rcm3.joint1_placement.setRandom();
  rcm3.joint2_placement.setRandom();
  rcm3.m_baumgarte_parameters.Kd = 1.;
  rcm3.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm3);
  contact_datas.push_back(rcm3.createData());

  RigidConstraintModel rcm4 = RigidConstraintModel(
    CONTACT_3D, model, model.getJointId("joint29"), model.getJointId("joint39"), LOCAL);
  rcm4.joint1_placement.setRandom();
  rcm4.joint2_placement.setRandom();
  rcm4.m_baumgarte_parameters.Kd = 1.;
  rcm4.m_baumgarte_parameters.Kp = 1.;
  contact_models.push_back(rcm4);
  contact_datas.push_back(rcm4.createData());

  const double mu0 = 1e-3;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  std::cout << "data_ref.ddq:\n" << data_ref.ddq.transpose() << std::endl;
  std::cout << "data.ddq:\n" << data.ddq.transpose() << std::endl;
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_loop_con_and_ground_con)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint17"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  RigidConstraintModel rcm2 =
    RigidConstraintModel(CONTACT_6D, model, model.getJointId("joint14"), LOCAL);
  rcm2.joint1_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_loop_con_and_ground_con3D)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint12"), model.getJointId("joint17"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  RigidConstraintModel rcm2 =
    RigidConstraintModel(CONTACT_3D, model, model.getJointId("joint24"), LOCAL);
  rcm2.joint1_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_loop_con3D_ground_con3D)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_3D, model, model.getJointId("joint12"), model.getJointId("joint17"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  RigidConstraintModel rcm2 =
    RigidConstraintModel(CONTACT_3D, model, model.getJointId("joint24"), LOCAL);
  rcm2.joint1_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}

BOOST_AUTO_TEST_CASE(test_coupled_3D_6D_loops)
{
  Model model;

  build_trident_model(model);
  Data data(model), data_ref(model);

  // Contact models and data
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_datas;

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  RigidConstraintModel rcm1 = RigidConstraintModel(
    CONTACT_3D, model, model.getJointId("joint12"), model.getJointId("joint27"), LOCAL);
  rcm1.joint1_placement.setRandom();
  rcm1.joint2_placement.setRandom();

  RigidConstraintModel rcm2 = RigidConstraintModel(
    CONTACT_6D, model, model.getJointId("joint24"), model.getJointId("joint11"), LOCAL);
  rcm2.joint1_placement.setRandom();
  rcm2.joint2_placement.setRandom();

  contact_models.push_back(rcm1);
  contact_datas.push_back(rcm1.createData());

  contact_models.push_back(rcm2);
  contact_datas.push_back(rcm2.createData());

  const double mu0 = 1e-1;
  ProximalSettings prox_settings_ref(1e-12, mu0, 3);
  ProximalSettings prox_settings(prox_settings_ref);

  initConstraintDynamics(model, data_ref, contact_models, contact_datas);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings_ref);

  computeJointMinimalOrdering(model, data, contact_models);
  lcaba(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq, 1e-10));
}
