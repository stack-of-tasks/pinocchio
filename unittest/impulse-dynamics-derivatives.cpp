//
// Copyright (c) 2020-2021 CNRS INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_derivatives_no_contact)
{
  // result: (dMdq)(dqafter-v) = drnea(q,0,dqafter-v)
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);

  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) empty_contact_data;

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  const double r_coeff = 0.5;

  initConstraintDynamics(model, data, empty_contact_models);
  impulseDynamics(
    model, data, q, v, empty_contact_models, empty_contact_data, r_coeff, prox_settings);

  const Eigen::VectorXd dv = data.dq_after - v;
  computeImpulseDynamicsDerivatives(
    model, data, empty_contact_models, empty_contact_data, r_coeff, prox_settings);

  Motion gravity_bk = model.gravity;
  model.gravity.setZero();
  computeRNEADerivatives(model, data_ref, q, Eigen::VectorXd::Zero(model.nv), dv);
  // Reference values
  BOOST_CHECK(data_ref.dtau_dq.isApprox(data.dtau_dq));
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL);
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);

  contact_models.push_back(ci_LF);
  contact_data.push_back(RigidConstraintData(ci_LF));
  contact_models.push_back(ci_RF);
  contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_data, r_coeff, prox_settings);
  computeImpulseDynamicsDerivatives(
    model, data, contact_models, contact_data, r_coeff, prox_settings);

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;

  ForceVector iext((size_t)model.njoints);
  for (ForceVector::iterator it = iext.begin(); it != iext.end(); ++it)
    (*it).setZero();

  iext[model.getJointId(LF)] = contact_data[0].contact_force;
  iext[model.getJointId(RF)] = contact_data[1].contact_force;

  Eigen::VectorXd effective_v = (1 + r_coeff) * v + data.ddq;

  computeForwardKinematicsDerivatives(
    model, data_ref, q, effective_v, Eigen::VectorXd::Zero(model.nv));

  for (size_t i = 0; i < data.ov.size(); i++)
  {
    BOOST_CHECK(((1 + r_coeff) * data.ov[i] + data.oa[i] - data_ref.ov[i]).isZero());
  }

  Eigen::MatrixXd Jc(9, model.nv), dv_dq(9, model.nv), Jc_tmp(6, model.nv), dv_dq_tmp(6, model.nv);
  Jc.setZero();
  dv_dq.setZero();
  dv_dq_tmp.setZero();
  Jc_tmp.setZero();

  getJointVelocityDerivatives(model, data_ref, LF_id, LOCAL, dv_dq.topRows<6>(), Jc.topRows<6>());

  getJointVelocityDerivatives(model, data_ref, RF_id, LOCAL, dv_dq_tmp, Jc_tmp);

  Jc.bottomRows<3>() = Jc_tmp.topRows<3>();
  dv_dq.bottomRows<3>() = dv_dq_tmp.topRows<3>();

  BOOST_CHECK(data_ref.J.isApprox(data.J));

  const Motion gravity_bk = model.gravity;
  model.gravity.setZero();
  computeRNEADerivatives(model, data_ref, q, Eigen::VectorXd::Zero(model.nv), data.ddq, iext);
  model.gravity = gravity_bk;

  BOOST_CHECK(data.dac_da.isApprox(Jc));
  //  BOOST_CHECK((data.dvc_dq-(dv_dq-Jc*data.Minv*data_ref.dtau_dq)).norm()<=1e-12);

  BOOST_CHECK((data.dlambda_dv + (1 + r_coeff) * data.osim * Jc).isZero());
}

BOOST_AUTO_TEST_CASE(test_impulse_dynamics_derivatives_LOCAL_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, SE3::Random(), LOCAL);
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, SE3::Random(), LOCAL);

  contact_models.push_back(ci_LF);
  contact_data.push_back(RigidConstraintData(ci_LF));
  contact_models.push_back(ci_RF);
  contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_data, r_coeff, prox_settings);
  computeImpulseDynamicsDerivatives(
    model, data, contact_models, contact_data, r_coeff, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, contact_models);

  MatrixXd dqafter_partial_dq_fd(model.nv, model.nv);
  dqafter_partial_dq_fd.setZero();
  MatrixXd dqafter_partial_dv_fd(model.nv, model.nv);
  dqafter_partial_dv_fd.setZero();

  MatrixXd impulse_partial_dq_fd(constraint_dim, model.nv);
  impulse_partial_dq_fd.setZero();
  MatrixXd impulse_partial_dv_fd(constraint_dim, model.nv);
  impulse_partial_dv_fd.setZero();

  const VectorXd dqafter0 =
    impulseDynamics(model, data_fd, q, v, contact_models, contact_data, r_coeff, prox_settings);
  const VectorXd impulse0 = data_fd.impulse_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd dqafter_plus(model.nv);

  const Eigen::MatrixXd Jc = data.contact_chol.matrix().topRightCorner(constraint_dim, model.nv);
  const Eigen::VectorXd vel_jump = Jc * (dqafter0 + r_coeff * v);

  Data data_plus(model);
  VectorXd impulse_plus(constraint_dim);

  Eigen::MatrixXd dvc_dq_fd(constraint_dim, model.nv);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    dqafter_plus = impulseDynamics(
      model, data_fd, q_plus, v, contact_models, contact_data, r_coeff, prox_settings);

    const Eigen::MatrixXd Jc_plus =
      data_fd.contact_chol.matrix().topRightCorner(constraint_dim, model.nv);
    const Eigen::VectorXd vel_jump_plus = Jc_plus * (dqafter0 + r_coeff * v);

    dqafter_partial_dq_fd.col(k) = (dqafter_plus - dqafter0) / alpha;
    impulse_partial_dq_fd.col(k) = (data_fd.impulse_c - impulse0) / alpha;
    dvc_dq_fd.col(k) = (vel_jump_plus - vel_jump) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(Jc.isApprox(data.dac_da, sqrt(alpha)));
  BOOST_CHECK(dqafter_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(impulse_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    dqafter_plus = impulseDynamics(
      model, data_fd, q, v_plus, contact_models, contact_data, r_coeff, prox_settings);

    dqafter_partial_dv_fd.col(k) = (dqafter_plus - dqafter0) / alpha;
    impulse_partial_dv_fd.col(k) = (data_fd.impulse_c - impulse0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(dqafter_partial_dv_fd.isApprox(
    Eigen::MatrixXd::Identity(model.nv, model.nv) + data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(impulse_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_impulse_dynamics_derivatives_LOCAL_WORLD_ALIGNED_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, SE3::Random(), LOCAL_WORLD_ALIGNED);
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, SE3::Random(), LOCAL_WORLD_ALIGNED);

  contact_models.push_back(ci_LF);
  contact_data.push_back(RigidConstraintData(ci_LF));
  contact_models.push_back(ci_RF);
  contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_data, r_coeff, prox_settings);
  computeImpulseDynamicsDerivatives(
    model, data, contact_models, contact_data, r_coeff, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, contact_models);

  MatrixXd dqafter_partial_dq_fd(model.nv, model.nv);
  dqafter_partial_dq_fd.setZero();
  MatrixXd dqafter_partial_dv_fd(model.nv, model.nv);
  dqafter_partial_dv_fd.setZero();

  MatrixXd impulse_partial_dq_fd(constraint_dim, model.nv);
  impulse_partial_dq_fd.setZero();
  MatrixXd impulse_partial_dv_fd(constraint_dim, model.nv);
  impulse_partial_dv_fd.setZero();

  const VectorXd dqafter0 =
    impulseDynamics(model, data_fd, q, v, contact_models, contact_data, r_coeff, prox_settings);
  const VectorXd impulse0 = data_fd.impulse_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd dqafter_plus(model.nv);

  VectorXd impulse_plus(constraint_dim);
  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    dqafter_plus = impulseDynamics(
      model, data_fd, q_plus, v, contact_models, contact_data, r_coeff, prox_settings);
    dqafter_partial_dq_fd.col(k) = (dqafter_plus - dqafter0) / alpha;
    impulse_partial_dq_fd.col(k) = (data_fd.impulse_c - impulse0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(dqafter_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(impulse_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    dqafter_plus = impulseDynamics(
      model, data_fd, q, v_plus, contact_models, contact_data, r_coeff, prox_settings);
    dqafter_partial_dv_fd.col(k) = (dqafter_plus - dqafter0) / alpha;
    impulse_partial_dv_fd.col(k) = (data_fd.impulse_c - impulse0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(dqafter_partial_dv_fd.isApprox(
    Eigen::MatrixXd::Identity(model.nv, model.nv) + data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(impulse_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));
}

BOOST_AUTO_TEST_SUITE_END()
