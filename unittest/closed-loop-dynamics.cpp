//
// Copyright (c) 2020-2022 INRIA
//

#include <iostream>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/proximal.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(closed_loop_constraint_6D_LOCAL)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;

  RigidConstraintModel ci_RF_LF(
    CONTACT_6D, model, model.getJointId(RF), model.getJointId(LF), LOCAL);
  ci_RF_LF.joint1_placement.setRandom();
  ci_RF_LF.joint2_placement.setRandom();
  contact_models.push_back(ci_RF_LF);
  contact_datas.push_back(RigidConstraintData(ci_RF_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  Eigen::MatrixXd J_ref(constraint_dim, model.nv);
  J_ref.setZero();

  computeAllTerms(model, data_ref, q, v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  Data::Matrix6x J_RF_local(6, model.nv), J_LF_local(6, model.nv);
  J_RF_local.setZero();
  J_LF_local.setZero();
  getFrameJacobian(
    model, data_ref, model.getJointId(RF), ci_RF_LF.joint1_placement, LOCAL, J_RF_local);
  getFrameJacobian(
    model, data_ref, model.getJointId(LF), ci_RF_LF.joint2_placement, LOCAL, J_LF_local);

  const SE3 oMc1_ref = data_ref.oMi[ci_RF_LF.joint1_id] * ci_RF_LF.joint1_placement;
  const SE3 oMc2_ref = data_ref.oMi[ci_RF_LF.joint2_id] * ci_RF_LF.joint2_placement;
  const SE3 c1Mc2_ref = oMc1_ref.actInv(oMc2_ref);

  J_ref = J_RF_local - c1Mc2_ref.toActionMatrix() * J_LF_local;

  Eigen::VectorXd rhs_ref(constraint_dim);

  const Motion vc1_ref = ci_RF_LF.joint1_placement.actInv(data_ref.v[ci_RF_LF.joint1_id]);
  const Motion vc2_ref = ci_RF_LF.joint2_placement.actInv(data_ref.v[ci_RF_LF.joint2_id]);
  const Motion constraint_velocity_error_ref = vc1_ref - c1Mc2_ref.act(vc2_ref);
  BOOST_CHECK(constraint_velocity_error_ref.isApprox(Motion(J_ref * v)));

  const Motion ac1_ref = ci_RF_LF.joint1_placement.actInv(data_ref.a[ci_RF_LF.joint1_id]);
  const Motion ac2_ref = ci_RF_LF.joint2_placement.actInv(data_ref.a[ci_RF_LF.joint2_id]);
  const Motion constraint_acceleration_error_ref =
    ac1_ref - c1Mc2_ref.act(ac2_ref) + constraint_velocity_error_ref.cross(c1Mc2_ref.act(vc2_ref));
  rhs_ref.segment<6>(0) = constraint_acceleration_error_ref.toVector();

  Eigen::MatrixXd KKT_matrix_ref =
    Eigen::MatrixXd::Zero(model.nv + constraint_dim, model.nv + constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim, model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv, constraint_dim) = J_ref.transpose();

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  forwardDynamics(model, data_ref, q, v, tau, J_ref, rhs_ref, mu0);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  forwardKinematics(model, data_ref, q, v, data_ref.ddq);

  BOOST_CHECK((J_ref * data_ref.ddq + rhs_ref).isZero());

  initConstraintDynamics(model, data, contact_models);
  constraintDynamics(model, data, q, v, tau, contact_models, contact_datas, prox_settings);

  BOOST_CHECK((J_ref * data.ddq + rhs_ref).isZero());

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));

  const Eigen::MatrixXd KKT_matrix = data.contact_chol.matrix();
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));

  // Check with finite differences the error computations
  Data data_plus(model);
  const double dt = 1e-8;
  const Eigen::VectorXd q_plus = integrate(model, q, (v * dt).eval());

  forwardKinematics(model, data_plus, q_plus, v, Eigen::VectorXd::Zero(model.nv));

  const SE3 oMc1_plus_ref = data_plus.oMi[ci_RF_LF.joint1_id] * ci_RF_LF.joint1_placement;
  const SE3 oMc2_plus_ref = data_plus.oMi[ci_RF_LF.joint2_id] * ci_RF_LF.joint2_placement;
  const SE3 c1Mc2_plus_ref = oMc1_plus_ref.actInv(oMc2_plus_ref);

  // Position level
  BOOST_CHECK(contact_datas[0].c1Mc2.isApprox(c1Mc2_ref));

  // Velocity level
  BOOST_CHECK(contact_datas[0].contact1_velocity.isApprox(vc1_ref));
  BOOST_CHECK(contact_datas[0].contact2_velocity.isApprox(vc2_ref));

  const Motion constraint_velocity_error_fd =
    -c1Mc2_ref.act(log6(c1Mc2_ref.actInv(c1Mc2_plus_ref))) / dt;
  BOOST_CHECK(constraint_velocity_error_ref.isApprox(constraint_velocity_error_fd, math::sqrt(dt)));
  BOOST_CHECK(contact_datas[0].contact_velocity_error.isApprox(constraint_velocity_error_ref));

  // Acceleration level
  const Motion vc1_plus_ref = ci_RF_LF.joint1_placement.actInv(data_plus.v[ci_RF_LF.joint1_id]);
  const Motion vc2_plus_ref = ci_RF_LF.joint2_placement.actInv(data_plus.v[ci_RF_LF.joint2_id]);
  const Motion constraint_velocity_error_plus_ref = vc1_plus_ref - c1Mc2_plus_ref.act(vc2_plus_ref);
  const Motion constraint_acceleration_error_fd =
    (constraint_velocity_error_plus_ref - constraint_velocity_error_ref) / dt;
  BOOST_CHECK(
    constraint_acceleration_error_ref.isApprox(constraint_acceleration_error_fd, math::sqrt(dt)));
}

BOOST_AUTO_TEST_SUITE_END()
