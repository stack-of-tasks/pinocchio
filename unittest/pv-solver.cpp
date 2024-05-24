//
// Copyright (c) 2023-2024 INRIA
// Copyright (c) 2023 KU Leuven
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"
#include "pinocchio/algorithm/pv.hpp"

#include <boost/test/tools/old/interface.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

// TODO: add tests on J_ref*ddq - rhs and on the OSIM matrix for PV. Add tests for proxLTLs

/// \brief Computes motions in the world frame
pinocchio::Motion computeAcceleration(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::JointIndex & joint_id,
  pinocchio::ReferenceFrame reference_frame,
  const pinocchio::ContactType type,
  const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  PINOCCHIO_UNUSED_VARIABLE(model);
  using namespace pinocchio;
  Motion res(Motion::Zero());

  const Data::SE3 & oMi = data.oMi[joint_id];
  const Data::SE3 & iMc = placement;
  const Data::SE3 oMc = oMi * iMc;

  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);

  switch (reference_frame)
  {
  case WORLD:
    if (type == CONTACT_3D)
      classicAcceleration(ov, oa, res.linear());
    else
      res.linear() = oa.linear();
    res.angular() = oa.angular();
    break;
  case LOCAL_WORLD_ALIGNED:
    if (type == CONTACT_3D)
      res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id], data.a[joint_id], iMc);
    else
      res.linear() = oMc.rotation() * (iMc.actInv(data.a[joint_id])).linear();
    res.angular() = oMi.rotation() * data.a[joint_id].angular();
    break;
  case LOCAL:
    if (type == CONTACT_3D)
      classicAcceleration(data.v[joint_id], data.a[joint_id], iMc, res.linear());
    else
      res.linear() = (iMc.actInv(data.a[joint_id])).linear();
    res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
    break;
  default:
    break;
  }

  return res;
}

BOOST_AUTO_TEST_CASE(test_sparse_forward_dynamics_empty)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) empty_contact_datas;

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initPvSolver(model, data, empty_contact_models);
  pv(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings);

  // Check solutions
  aba(model, data_ref, q, v, tau, Convention::WORLD);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  // Checking if solving again the same problem gives the same solution
  pv(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  prox_settings.mu = 1e-5;
  constrainedABA(model, data, q, v, tau, empty_contact_models, empty_contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
}

BOOST_AUTO_TEST_CASE(test_forward_dynamics_in_contact_6D_LOCAL_humanoid)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  const double mu0 = 0.0;

  ProximalSettings prox_settings(1e-12, mu0, 1);
  initConstraintDynamics(model, data_ref, contact_models);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);

  initPvSolver(model, data, contact_models);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  prox_settings.mu = 0.0;

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);

  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  // Warning: the test below is not guaranteed to work for different constraints since the order of
  // constraints in PV and ProxLTL can vary.
  data_ref.osim = data_ref.contact_chol.getInverseOperationalSpaceInertiaMatrix();
  data.LA[0].template triangularView<Eigen::StrictlyUpper>() =
    data.LA[0].template triangularView<Eigen::StrictlyLower>().transpose();
  BOOST_CHECK(data_ref.osim.isApprox(data.LA[0]));

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  prox_settings.mu = 1e-4;
  prox_settings.max_iter = 6;

  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  // Send non-zero mu and max 1 iteration and the solution should not match
  prox_settings.mu = 1e-3;
  prox_settings.max_iter = 1;
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(!data.ddq.isApprox(data_ref.ddq));

  // Change max iter to 10 and now should work
  prox_settings.max_iter = 10;
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
}

BOOST_AUTO_TEST_CASE(test_forward_dynamics_3D_humanoid)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));

  const double mu0 = 0.0;

  ProximalSettings prox_settings(1e-12, mu0, 1);
  initConstraintDynamics(model, data_ref, contact_models);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);

  initPvSolver(model, data, contact_models);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);

  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  data_ref.osim = data_ref.contact_chol.getInverseOperationalSpaceInertiaMatrix();
  data.LA[0].template triangularView<Eigen::StrictlyUpper>() =
    data.LA[0].template triangularView<Eigen::StrictlyLower>().transpose();
  BOOST_CHECK(data_ref.osim.isApprox(data.LA[0]));

  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  initPvSolver(model, data, contact_models);
  prox_settings.mu = 1e-4;
  prox_settings.max_iter = 6;
  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
}

BOOST_AUTO_TEST_CASE(test_forward_dynamics_repeating_3D_humanoid)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kd.setZero();
  ci_RF.corrector.Kp.setZero();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_RF2(CONTACT_6D, model, model.getJointId("rleg5_joint"), LOCAL);
  ci_RF2.joint1_placement.setRandom();
  ci_RF2.corrector.Kd.setZero();
  ci_RF2.corrector.Kp.setZero();
  contact_models.push_back(ci_RF2);
  contact_datas.push_back(RigidConstraintData(ci_RF2));

  const double mu0 = 1e-3;

  ProximalSettings prox_settings(1e-14, mu0, 10);
  initConstraintDynamics(model, data_ref, contact_models);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);

  computeAllTerms(model, data_ref, q, v);

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  Eigen::MatrixXd J_ref(constraint_dim, model.nv);
  J_ref.setZero();
  Data::Matrix6x Jtmp = Data::Matrix6x::Zero(6, model.nv);

  getJointJacobian(model, data_ref, ci_RF.joint1_id, ci_RF.reference_frame, Jtmp);
  J_ref.middleRows<6>(0) = ci_RF.joint1_placement.inverse().toActionMatrix() * Jtmp;

  Jtmp.setZero();
  getJointJacobian(model, data_ref, ci_RF2.joint1_id, ci_RF2.reference_frame, Jtmp);
  J_ref.middleRows<6>(6) = ci_RF2.joint1_placement.inverse().toActionMatrix() * Jtmp;

  Eigen::VectorXd rhs_ref(constraint_dim);
  rhs_ref.segment<6>(0) =
    computeAcceleration(
      model, data_ref, ci_RF.joint1_id, ci_RF.reference_frame, ci_RF.type, ci_RF.joint1_placement)
      .toVector();
  rhs_ref.segment<6>(6) = computeAcceleration(
                            model, data_ref, ci_RF2.joint1_id, ci_RF2.reference_frame, ci_RF2.type,
                            ci_RF2.joint1_placement)
                            .toVector();

  BOOST_CHECK((J_ref.transpose() * (J_ref * data_ref.ddq + rhs_ref)).isZero(1e-11));

  initPvSolver(model, data, contact_models);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK((J_ref.transpose() * (J_ref * data.ddq + rhs_ref)).isZero(1e-11));

  initPvSolver(model, data, contact_models);
  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK((J_ref.transpose() * (J_ref * data.ddq + rhs_ref)).isZero(1e-11));

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);

  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  initPvSolver(model, data, contact_models);
  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
}

BOOST_AUTO_TEST_CASE(test_FD_humanoid_redundant_baumgarte)
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kd.setIdentity();
  ci_RF.corrector.Kp.setIdentity();
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_RF2(CONTACT_6D, model, model.getJointId("rleg5_joint"), LOCAL);
  ci_RF2.joint1_placement.setRandom();
  ci_RF2.corrector.Kd.setIdentity();
  ci_RF2.corrector.Kp.setZero();
  contact_models.push_back(ci_RF2);
  contact_datas.push_back(RigidConstraintData(ci_RF2));

  const double mu0 = 1e-4;

  ProximalSettings prox_settings(1e-14, mu0, 10);
  initConstraintDynamics(model, data_ref, contact_models);
  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);

  initPvSolver(model, data, contact_models);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  // Check the solver works the second time for new random inputs
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);

  constraintDynamics(model, data_ref, q, v, tau, contact_models, contact_datas, prox_settings);
  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  pv(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));

  initPvSolver(model, data, contact_models);
  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-11));

  constrainedABA(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq, 1e-11));
}

BOOST_AUTO_TEST_SUITE_END()
