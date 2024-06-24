//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_empty)
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;
  computeAllTerms(model, data_ref, q, v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.dq_after.isApprox(v));
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_in_contact_6D_LOCAL)
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
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;
  Eigen::MatrixXd J_ref(constraint_dim, model.nv);
  J_ref.setZero();

  computeAllTerms(model, data_ref, q, v);
  framesForwardKinematics(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  updateFramePlacements(model, data_ref);
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_ref.middleRows<6>(0));
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, J_ref.middleRows<6>(6));

  Eigen::VectorXd rhs_ref(constraint_dim);

  rhs_ref.segment<6>(0) = getFrameVelocity(model, data_ref, model.getFrameId(RF), LOCAL).toVector();
  rhs_ref.segment<6>(6) = getFrameVelocity(model, data_ref, model.getFrameId(LF), LOCAL).toVector();

  Eigen::MatrixXd KKT_matrix_ref =
    Eigen::MatrixXd::Zero(model.nv + constraint_dim, model.nv + constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim, model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv, constraint_dim) = J_ref.transpose();

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  impulseDynamics(model, data_ref, q, v, J_ref, r_coeff, mu0);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK(
    (data_ref.M * data_ref.dq_after - data_ref.M * v - J_ref.transpose() * data_ref.impulse_c)
      .isZero());
  BOOST_CHECK((J_ref * data_ref.dq_after + r_coeff * J_ref * v).isZero());

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings);
  BOOST_CHECK((J_ref * data.dq_after + r_coeff * J_ref * v).isZero());
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK((data.M * data.dq_after - data.M * v - J_ref.transpose() * data.impulse_c).isZero());

  Data data_ag(model);
  ccrba(model, data_ag, q, v);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.Ag.isApprox(data_ag.Ag));

  for (Model::JointIndex k = 1; k < model.joints.size(); ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.liMi[k].isApprox(data_ref.liMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.oMi[k].act(data_ref.v[k])));
    // BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oMi[k].act(data_ref.a_gf[k])));
  }

  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();

  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv, model.nv)
                .isApprox(KKT_matrix_ref.bottomRightCorner(model.nv, model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));

  // Check solutions
  BOOST_CHECK(data.dq_after.isApprox(data_ref.dq_after));
  BOOST_CHECK(data.impulse_c.isApprox(data_ref.impulse_c));

  Eigen::DenseIndex constraint_id = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = contact_models[k];
    const RigidConstraintData & cdata = contact_datas[k];

    switch (cmodel.type)
    {
    case pinocchio::CONTACT_3D: {
      BOOST_CHECK(cdata.contact_force.linear().isApprox(
        data_ref.impulse_c.segment(constraint_id, cmodel.size())));
      break;
    }

    case pinocchio::CONTACT_6D: {
      ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(
        data_ref.impulse_c.segment<6>(constraint_id));
      BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
      break;
    }

    default:
      break;
    }

    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_in_contact_6D_LOCAL_WORLD_ALIGNED)
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
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_6D, model, model.getJointId(LF), LOCAL_WORLD_ALIGNED);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;
  Eigen::MatrixXd J_ref(constraint_dim, model.nv);
  J_ref.setZero();

  computeAllTerms(model, data_ref, q, v);
  framesForwardKinematics(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  updateFramePlacements(model, data_ref);
  getJointJacobian(
    model, data_ref, model.getJointId(RF), LOCAL_WORLD_ALIGNED, J_ref.middleRows<6>(0));
  getJointJacobian(
    model, data_ref, model.getJointId(LF), LOCAL_WORLD_ALIGNED, J_ref.middleRows<6>(6));

  Eigen::VectorXd rhs_ref(constraint_dim);

  rhs_ref.segment<6>(0) =
    getFrameVelocity(model, data_ref, model.getFrameId(RF), LOCAL_WORLD_ALIGNED).toVector();
  rhs_ref.segment<6>(6) =
    getFrameVelocity(model, data_ref, model.getFrameId(LF), LOCAL_WORLD_ALIGNED).toVector();

  Eigen::MatrixXd KKT_matrix_ref =
    Eigen::MatrixXd::Zero(model.nv + constraint_dim, model.nv + constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim, model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv, constraint_dim) = J_ref.transpose();

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  impulseDynamics(model, data_ref, q, v, J_ref, r_coeff, mu0);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK(
    (data_ref.M * data_ref.dq_after - data_ref.M * v - J_ref.transpose() * data_ref.impulse_c)
      .isZero());
  BOOST_CHECK((J_ref * data_ref.dq_after + r_coeff * J_ref * v).isZero());

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings);
  BOOST_CHECK((J_ref * data.dq_after + r_coeff * J_ref * v).isZero());
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK((data.M * data.dq_after - data.M * v - J_ref.transpose() * data.impulse_c).isZero());

  Data data_ag(model);
  ccrba(model, data_ag, q, v);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.Ag.isApprox(data_ag.Ag));

  for (Model::JointIndex k = 1; k < model.joints.size(); ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.liMi[k].isApprox(data_ref.liMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.oMi[k].act(data_ref.v[k])));
    // BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oMi[k].act(data_ref.a_gf[k])));
  }

  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();

  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv, model.nv)
                .isApprox(KKT_matrix_ref.bottomRightCorner(model.nv, model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));

  // Check solutions
  BOOST_CHECK(data.dq_after.isApprox(data_ref.dq_after));
  BOOST_CHECK(data.impulse_c.isApprox(data_ref.impulse_c));

  Eigen::DenseIndex constraint_id = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = contact_models[k];
    const RigidConstraintData & cdata = contact_datas[k];

    switch (cmodel.type)
    {
    case pinocchio::CONTACT_3D: {
      BOOST_CHECK(cdata.contact_force.linear().isApprox(
        data_ref.impulse_c.segment(constraint_id, cmodel.size())));
      break;
    }

    case pinocchio::CONTACT_6D: {
      ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(
        data_ref.impulse_c.segment<6>(constraint_id));
      BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
      break;
    }

    default:
      break;
    }

    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_in_contact_6D_3D)
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
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  RigidConstraintModel ci_RF(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  RigidConstraintModel ci_LF(CONTACT_3D, model, model.getJointId(LF), LOCAL);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);
  const double r_coeff = 0.5;
  Eigen::MatrixXd J_ref(constraint_dim, model.nv), Jtmp(6, model.nv);
  J_ref.setZero();
  Jtmp.setZero();

  computeAllTerms(model, data_ref, q, v);
  framesForwardKinematics(model, data_ref, q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();

  updateFramePlacements(model, data_ref);
  getJointJacobian(model, data_ref, model.getJointId(RF), LOCAL, J_ref.middleRows<6>(0));
  getJointJacobian(model, data_ref, model.getJointId(LF), LOCAL, Jtmp);
  J_ref.middleRows<3>(6) = Jtmp.middleRows<3>(Motion::LINEAR);

  Eigen::VectorXd rhs_ref(constraint_dim);

  rhs_ref.segment<6>(0) = getFrameVelocity(model, data_ref, model.getFrameId(RF), LOCAL).toVector();
  rhs_ref.segment<3>(6) = getFrameVelocity(model, data_ref, model.getFrameId(LF), LOCAL).linear();

  Eigen::MatrixXd KKT_matrix_ref =
    Eigen::MatrixXd::Zero(model.nv + constraint_dim, model.nv + constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim, model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv, constraint_dim) = J_ref.transpose();

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  impulseDynamics(model, data_ref, q, v, J_ref, r_coeff, mu0);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK(
    (data_ref.M * data_ref.dq_after - data_ref.M * v - J_ref.transpose() * data_ref.impulse_c)
      .isZero());
  BOOST_CHECK((J_ref * data_ref.dq_after + r_coeff * J_ref * v).isZero());

  initConstraintDynamics(model, data, contact_models);
  impulseDynamics(model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings);
  BOOST_CHECK((J_ref * data.dq_after + r_coeff * J_ref * v).isZero());
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  BOOST_CHECK((data.M * data.dq_after - data.M * v - J_ref.transpose() * data.impulse_c).isZero());

  Data data_ag(model);
  ccrba(model, data_ag, q, v);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data.Ag.isApprox(data_ag.Ag));

  for (Model::JointIndex k = 1; k < model.joints.size(); ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.liMi[k].isApprox(data_ref.liMi[k]));
    BOOST_CHECK(data.ov[k].isApprox(data_ref.oMi[k].act(data_ref.v[k])));
    // BOOST_CHECK(data.oa_gf[k].isApprox(data_ref.oMi[k].act(data_ref.a_gf[k])));
  }

  // Check that the decomposition is correct
  const Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
  Eigen::MatrixXd KKT_matrix = contact_chol.matrix();

  BOOST_CHECK(KKT_matrix.bottomRightCorner(model.nv, model.nv)
                .isApprox(KKT_matrix_ref.bottomRightCorner(model.nv, model.nv)));
  BOOST_CHECK(KKT_matrix.isApprox(KKT_matrix_ref));

  // Check solutions
  BOOST_CHECK(data.dq_after.isApprox(data_ref.dq_after));
  BOOST_CHECK(data.impulse_c.isApprox(data_ref.impulse_c));

  Eigen::DenseIndex constraint_id = 0;
  for (size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = contact_models[k];
    const RigidConstraintData & cdata = contact_datas[k];

    switch (cmodel.type)
    {
    case pinocchio::CONTACT_3D: {
      BOOST_CHECK(cdata.contact_force.linear().isApprox(
        data_ref.impulse_c.segment(constraint_id, cmodel.size())));
      break;
    }

    case pinocchio::CONTACT_6D: {
      ForceRef<Data::VectorXs::FixedSegmentReturnType<6>::Type> f_ref(
        data_ref.impulse_c.segment<6>(constraint_id));
      BOOST_CHECK(cdata.contact_force.isApprox(f_ref));
      break;
    }

    default:
      break;
    }

    constraint_id += cmodel.size();
  }
}

BOOST_AUTO_TEST_SUITE_END()
