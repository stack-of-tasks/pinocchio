//
// Copyright (c) 2020-2022 CNRS INRIA
//

#include <iostream>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#ifdef PINOCCHIO_WITH_SDFORMAT

  #include "pinocchio/parsers/sdf.hpp"

#endif // PINOCCHIO_WITH_SDFORMAT

#define KP 10
#define KD 10

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_sparse_constraint_dynamics_derivatives_no_contact)
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
  VectorXd tau = VectorXd::Random(model.nv);

  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) empty_constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) empty_constraint_data;

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, empty_constraint_models);
  constraintDynamics(
    model, data, q, v, tau, empty_constraint_models, empty_constraint_data, prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, empty_constraint_models, empty_constraint_data, prox_settings);

  // Reference values
  computeABADerivatives(model, data_ref, q, v, tau);
  forwardKinematics(model, data_ref, q, v, data_ref.ddq);
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));

  for (size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data_ref.oMi[k].isApprox(data.oMi[k]));
    BOOST_CHECK(data_ref.ov[k].isApprox(data.ov[k]));
    BOOST_CHECK(data_ref.v[k].isApprox(data.v[k]));
    BOOST_CHECK(data_ref.a[k].isApprox(data.a[k]));
    BOOST_CHECK(data_ref.oa[k].isApprox(data.oa[k]));
  }

  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));
  BOOST_CHECK(data_ref.dVdq.isApprox(data.dVdq));
  BOOST_CHECK(data_ref.J.isApprox(data.J));
  BOOST_CHECK(data_ref.dAdq.isApprox(data.dAdq));
  BOOST_CHECK(data_ref.dAdv.isApprox(data.dAdv));
  BOOST_CHECK(data_ref.dFdq.isApprox(data.dFdq));
  BOOST_CHECK(data_ref.dFdv.isApprox(data.dFdv));

  BOOST_CHECK(data_ref.dtau_dq.isApprox(data.dtau_dq));
  BOOST_CHECK(data_ref.dtau_dv.isApprox(data.dtau_dv));

  BOOST_CHECK(data_ref.ddq_dq.isApprox(data.ddq_dq));
  BOOST_CHECK(data_ref.ddq_dv.isApprox(data.ddq_dv));
  BOOST_CHECK(data_ref.Minv.isApprox(data.ddq_dtau));
}

BOOST_AUTO_TEST_CASE(test_sparse_constraint_dynamics_derivatives)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL);
  ci_LF.joint1_placement.setRandom();
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();

  constraint_models.push_back(ci_LF);
  constraint_data.push_back(RigidConstraintData(ci_LF));
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Reference values
  crba(model, data_ref, q, Convention::WORLD);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
    data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  container::aligned_vector<Force> fext((size_t)model.njoints, Force::Zero());
  for (size_t k = 0; k < constraint_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = constraint_models[k];
    const RigidConstraintData & cdata = constraint_data[k];
    fext[cmodel.joint1_id] = cmodel.joint1_placement.act(cdata.contact_force);

    BOOST_CHECK(cdata.oMc1.isApprox(data_ref.oMi[cmodel.joint1_id] * cmodel.joint1_placement));
  }

  computeABADerivatives(model, data_ref, q, v, tau, fext);
  forwardKinematics(model, data_ref, q, v);

  BOOST_CHECK(data.ddq.isApprox(data_ref.ddq));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dFdq.isApprox(data_ref.dFdq));
  BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));

  MatrixXd vLF_partial_dq(MatrixXd::Zero(6, model.nv)), aLF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aLF_partial_dv(MatrixXd::Zero(6, model.nv)), aLF_partial_da(MatrixXd::Zero(6, model.nv));

  MatrixXd vRF_partial_dq(MatrixXd::Zero(6, model.nv)), aRF_partial_dq(MatrixXd::Zero(6, model.nv)),
    aRF_partial_dv(MatrixXd::Zero(6, model.nv)), aRF_partial_da(MatrixXd::Zero(6, model.nv));

  getFrameAccelerationDerivatives(
    model, data_ref, LF_id, ci_LF.joint1_placement, LOCAL, vLF_partial_dq, aLF_partial_dq,
    aLF_partial_dv, aLF_partial_da);
  getFrameAccelerationDerivatives(
    model, data_ref, RF_id, ci_RF.joint1_placement, LOCAL, vRF_partial_dq, aRF_partial_dq,
    aRF_partial_dv, aRF_partial_da);

  MatrixXd Jc(constraint_dim, model.nv);
  Jc << aLF_partial_da, aRF_partial_da.topRows<3>();

  MatrixXd K(model.nv + constraint_dim, model.nv + constraint_dim);
  K << data_ref.M, Jc.transpose(), Jc, MatrixXd::Zero(constraint_dim, constraint_dim);
  const MatrixXd Kinv = K.inverse();

  MatrixXd osim((Jc * data_ref.M.inverse() * Jc.transpose()).inverse());
  BOOST_CHECK(data.osim.isApprox(osim));

  MatrixXd ac_partial_dq(constraint_dim, model.nv);

  BOOST_CHECK(data.ov[RF_id].isApprox(data_ref.oMi[RF_id].act(data_ref.v[RF_id])));
  aRF_partial_dq.topRows<3>() +=
    cross(ci_RF.joint1_placement.actInv(data_ref.v[RF_id]).angular(), vRF_partial_dq.topRows<3>())
    - cross(
      ci_RF.joint1_placement.actInv(data_ref.v[RF_id]).linear(), vRF_partial_dq.bottomRows<3>());

  BOOST_CHECK(data.ov[LF_id].isApprox(data_ref.oMi[LF_id].act(data_ref.v[LF_id])));

  ac_partial_dq << aLF_partial_dq, aRF_partial_dq.topRows<3>();

  MatrixXd dac_dq = ac_partial_dq; // - Jc * data_ref.Minv*data_ref.dtau_dq;

  BOOST_CHECK(data.dac_dq.isApprox(dac_dq, 1e-8));
  BOOST_CHECK(Kinv.bottomLeftCorner(constraint_dim, model.nv).isApprox(osim * Jc * data_ref.Minv));

  MatrixXd df_dq = Kinv.bottomLeftCorner(constraint_dim, model.nv) * data_ref.dtau_dq
                   + Kinv.bottomRightCorner(constraint_dim, constraint_dim) * ac_partial_dq;

  MatrixXd ddq_dq = data_ref.Minv * (-data_ref.dtau_dq + Jc.transpose() * df_dq);

  BOOST_CHECK(df_dq.isApprox(data.dlambda_dq));
  BOOST_CHECK(ddq_dq.isApprox(data.ddq_dq));
}

pinocchio::Motion computeAcceleration(
  const pinocchio::Model & model,
  const pinocchio::Data & data,
  const pinocchio::JointIndex & joint_id,
  const pinocchio::ReferenceFrame reference_frame,
  const pinocchio::ContactType contact_type,
  const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  PINOCCHIO_UNUSED_VARIABLE(model);
  using namespace pinocchio;
  Motion res(Motion::Zero());

  const Data::SE3 & oMi = data.oMi[joint_id];
  const Data::SE3 oMc = oMi * placement;

  const Data::SE3 & iMc = placement;
  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);

  switch (reference_frame)
  {
  case WORLD:
    if (contact_type == CONTACT_6D)
      return oa;
    classicAcceleration(ov, oa, res.linear());
    res.angular() = oa.angular();
    break;
  case LOCAL_WORLD_ALIGNED:
    if (contact_type == CONTACT_6D)
    {
      res.linear() = oMc.rotation() * iMc.actInv(data.a[joint_id]).linear();
      res.angular() = oMi.rotation() * data.a[joint_id].angular();
    }
    else
    {
      res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id], data.a[joint_id], iMc);
      res.angular() = oMi.rotation() * data.a[joint_id].angular();
    }
    break;
  case LOCAL:
    if (contact_type == CONTACT_6D)
      return oMc.actInv(oa);
    classicAcceleration(data.v[joint_id], data.a[joint_id], iMc, res.linear());
    res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
    break;
  default:
    break;
  }

  return res;
}

pinocchio::Motion getContactAcceleration(
  const Model & model,
  const Data & data,
  const RigidConstraintModel & cmodel,
  const pinocchio::SE3 & c1Mc2 = SE3::Identity())
{
  const Motion v1 = getFrameVelocity(
    model, data, cmodel.joint1_id, cmodel.joint1_placement, cmodel.reference_frame);
  const Motion v2 = getFrameVelocity(
    model, data, cmodel.joint2_id, cmodel.joint2_placement, cmodel.reference_frame);
  const Motion v = v1 - c1Mc2.act(v2);
  const Motion a1 = computeAcceleration(
    model, data, cmodel.joint1_id, cmodel.reference_frame, cmodel.type, cmodel.joint1_placement);
  const Motion a2 = computeAcceleration(
    model, data, cmodel.joint2_id, cmodel.reference_frame, cmodel.type, cmodel.joint2_placement);
  return a1 - c1Mc2.act(a2) + v.cross(c1Mc2.act(v2));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_6D_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.corrector.Kp.array() = KP;
  ci_LF.corrector.Kd.array() = KD;

  constraint_models.push_back(ci_LF);
  constraint_data.push_back(RigidConstraintData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData)
createData(
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel)
  & constraint_models)
{
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) constraint_datas;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_datas.push_back(pinocchio::RigidConstraintData(constraint_models[k]));

  return constraint_datas;
}

BOOST_AUTO_TEST_CASE(test_correction_6D)
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
  VectorXd tau = VectorXd::Random(model.nv);
  const double mu = 0.;
  ProximalSettings prox_settings(1e-12, mu, 1);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;

  RigidConstraintModel ci_RF(CONTACT_6D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.joint2_placement.setRandom();
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_RF);

  RigidConstraintModel ci_LF(CONTACT_3D, model, LF_id, LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.joint2_placement.setRandom();
  ci_LF.corrector.Kp.array() = KP;
  ci_LF.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_LF);

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
  constraint_datas = createData(constraint_models);
  initConstraintDynamics(model, data, constraint_models);
  const Eigen::VectorXd ddq0 =
    constraintDynamics(model, data, q, v, tau, constraint_models, constraint_datas, prox_settings);

  Eigen::MatrixXd ddq_dq(model.nv, model.nv), ddq_dv(model.nv, model.nv),
    ddq_dtau(model.nv, model.nv);
  Eigen::MatrixXd dlambda_dq(constraint_dim, model.nv), dlambda_dv(constraint_dim, model.nv),
    dlambda_dtau(constraint_dim, model.nv);

  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_datas, prox_settings, ddq_dq, ddq_dv, ddq_dtau,
    dlambda_dq, dlambda_dv, dlambda_dtau);
  computeForwardKinematicsDerivatives(model, data, q, v, 0 * v);

  Data::Matrix6x dv_RF_dq_L(Data::Matrix6x::Zero(6, model.nv));
  Data::Matrix6x dv_RF_dv_L(Data::Matrix6x::Zero(6, model.nv));
  getFrameVelocityDerivatives(
    model, data, ci_RF.joint1_id, ci_RF.joint1_placement, ci_RF.reference_frame, dv_RF_dq_L,
    dv_RF_dv_L);

  Data::Matrix6x dv_LF_dq_L(Data::Matrix6x::Zero(6, model.nv));
  Data::Matrix6x dv_LF_dv_L(Data::Matrix6x::Zero(6, model.nv));
  getFrameVelocityDerivatives(
    model, data, ci_LF.joint1_id, ci_LF.joint1_placement, ci_LF.reference_frame, dv_LF_dq_L,
    dv_LF_dv_L);

  const double eps = 1e-8;
  Data::Matrix6x dacc_corrector_RF_dq(6, model.nv);
  dacc_corrector_RF_dq.setZero();
  Data::Matrix6x dacc_corrector_RF_dv(6, model.nv);
  dacc_corrector_RF_dv.setZero();
  Data::Matrix3x dacc_corrector_LF_dq(3, model.nv);
  dacc_corrector_LF_dq.setZero();
  Data::Matrix3x dacc_corrector_LF_dv(3, model.nv);
  dacc_corrector_LF_dv.setZero();

  {
    const SE3::Matrix6 Jlog = Jlog6(constraint_datas[0].c1Mc2.inverse());
    dacc_corrector_RF_dq = -(ci_RF.corrector.Kp.asDiagonal() * Jlog * dv_RF_dv_L);
    dacc_corrector_RF_dq -= ci_RF.corrector.Kd.asDiagonal() * dv_RF_dq_L;

    dacc_corrector_RF_dv = -(ci_RF.corrector.Kd.asDiagonal() * dv_RF_dv_L);
    BOOST_CHECK(dv_RF_dv_L.isApprox(data.contact_chol.matrix().topRightCorner(6, model.nv)));
  }

  {
    dacc_corrector_LF_dq = -(ci_LF.corrector.Kp.asDiagonal() * dv_LF_dv_L.topRows<3>());
    dacc_corrector_LF_dq -= ci_LF.corrector.Kd.asDiagonal() * dv_LF_dq_L.topRows<3>();
    for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      dacc_corrector_LF_dq.col(k) +=
        ci_LF.corrector.Kp.asDiagonal()
        * dv_LF_dv_L.col(k).tail<3>().cross(constraint_datas[1].contact_placement_error.linear());
    }

    dacc_corrector_LF_dv = -(ci_LF.corrector.Kd.asDiagonal() * dv_LF_dv_L.topRows<3>());
    BOOST_CHECK(dv_LF_dv_L.topRows<3>().isApprox(
      data.contact_chol.matrix().topRightCorner(9, model.nv).bottomRows<3>()));
  }

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
  constraint_datas_fd = createData(constraint_models);
  initConstraintDynamics(model, data_fd, constraint_models);

  Data::Matrix6x dacc_corrector_RF_dq_fd(6, model.nv);
  Data::Matrix3x dacc_corrector_LF_dq_fd(3, model.nv);

  Eigen::MatrixXd ddq_dq_fd(model.nv, model.nv), ddq_dv_fd(model.nv, model.nv),
    ddq_dtau_fd(model.nv, model.nv);
  Eigen::MatrixXd dlambda_dq_fd(constraint_dim, model.nv), dlambda_dv_fd(constraint_dim, model.nv),
    dlambda_dtau_fd(constraint_dim, model.nv);

  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_eps = Eigen::VectorXd::Zero(model.nv);
    v_eps[k] = eps;
    const Eigen::VectorXd q_plus = integrate(model, q, v_eps);

    Eigen::VectorXd ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_datas_fd, prox_settings);
    dacc_corrector_RF_dq_fd.col(k) = (constraint_datas_fd[0].contact_acceleration_error
                                      - constraint_datas[0].contact_acceleration_error)
                                       .toVector()
                                     / eps;
    dacc_corrector_LF_dq_fd.col(k) = (constraint_datas_fd[1].contact_acceleration_error.linear()
                                      - constraint_datas[1].contact_acceleration_error.linear())
                                     / eps;

    ddq_dq_fd.col(k) = (ddq_plus - ddq0) / eps;
  }

  BOOST_CHECK(ddq_dq_fd.isApprox(ddq_dq, sqrt(eps)));
  //  std::cout << "ddq_dq_fd:\n" << ddq_dq_fd - ddq_dq << std::endl;
  //  std::cout << "ddq_dq:\n" << ddq_dq << std::endl;
  BOOST_CHECK(dacc_corrector_RF_dq.isApprox(dacc_corrector_RF_dq_fd, sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dq.isApprox(dacc_corrector_LF_dq_fd, sqrt(eps)));
  // std::cout << "dacc_corrector_RF_dq:\n" << dacc_corrector_RF_dq << std::endl;
  // std::cout << "dacc_corrector_RF_dq_fd:\n" << dacc_corrector_RF_dq_fd << std::endl;

  Data::Matrix6x dacc_corrector_RF_dv_fd(6, model.nv);
  Data::Matrix3x dacc_corrector_LF_dv_fd(3, model.nv);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_plus(v);
    v_plus[k] += eps;

    Eigen::VectorXd ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_datas_fd, prox_settings);
    dacc_corrector_RF_dv_fd.col(k) = (constraint_datas_fd[0].contact_acceleration_error
                                      - constraint_datas[0].contact_acceleration_error)
                                       .toVector()
                                     / eps;
    dacc_corrector_LF_dv_fd.col(k) = (constraint_datas_fd[1].contact_acceleration_error.linear()
                                      - constraint_datas[1].contact_acceleration_error.linear())
                                     / eps;

    ddq_dv_fd.col(k) = (ddq_plus - ddq0) / eps;
  }
  BOOST_CHECK(ddq_dv_fd.isApprox(ddq_dv, sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dv.isApprox(dacc_corrector_RF_dv_fd, sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dv.isApprox(dacc_corrector_LF_dv_fd, sqrt(eps)));

  Data::Matrix6x dacc_corrector_RF_dtau_fd(6, model.nv);
  Data::Matrix3x dacc_corrector_LF_dtau_fd(3, model.nv);
  for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd tau_plus(tau);
    tau_plus[k] += eps;

    Eigen::VectorXd ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_datas_fd, prox_settings);
    dacc_corrector_RF_dtau_fd.col(k) = (constraint_datas_fd[0].contact_acceleration_error
                                        - constraint_datas[0].contact_acceleration_error)
                                         .toVector()
                                       / eps;
    dacc_corrector_LF_dtau_fd.col(k) = (constraint_datas_fd[1].contact_acceleration_error.linear()
                                        - constraint_datas[1].contact_acceleration_error.linear())
                                       / eps;

    ddq_dtau_fd.col(k) = (ddq_plus - ddq0) / eps;
  }
  BOOST_CHECK(ddq_dtau_fd.isApprox(ddq_dtau, sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dtau_fd.isZero(sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dtau_fd.isZero(sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_fd_prox)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 1e-4;
  ProximalSettings prox_settings(1e-12, mu0, 20);

  initConstraintDynamics(model, data, constraint_models);
  VectorXd a_res =
    constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  BOOST_CHECK(prox_settings.iter > 1 && prox_settings.iter <= prox_settings.max_iter);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  ProximalSettings prox_settings_fd(1e-12, mu0, 20);
  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings_fd);
  BOOST_CHECK(a_res.isApprox(ddq0));
  const VectorXd lambda0 = data_fd.lambda_c;

  computeConstraintDynamicsDerivatives(
    model, data_fd, constraint_models, constraint_data, prox_settings_fd);
  BOOST_CHECK(data_fd.dlambda_dtau.isApprox(data.dlambda_dtau));

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_loop_closure_3D_fd_prox)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LF_id, LOCAL);
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  ci_RF.joint1_placement.setRandom();
  forwardKinematics(model, data, q);
  // data.oMi[LF_id] * ci_RF.joint2_placement = data.oMi[RF_id] * ci_RF.joint1_placement;
  ci_RF.joint2_placement = data.oMi[LF_id].inverse() * data.oMi[RF_id] * ci_RF.joint1_placement;

  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 1e-4;
  ProximalSettings prox_settings(1e-12, mu0, 20);

  initConstraintDynamics(model, data, constraint_models);
  VectorXd a_res =
    constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  BOOST_CHECK(prox_settings.iter > 1 && prox_settings.iter <= prox_settings.max_iter);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  ProximalSettings prox_settings_fd(1e-12, mu0, 20);
  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings_fd);
  BOOST_CHECK(a_res.isApprox(ddq0));
  const VectorXd lambda0 = data_fd.lambda_c;

  computeConstraintDynamicsDerivatives(
    model, data_fd, constraint_models, constraint_data, prox_settings_fd);
  BOOST_CHECK(data_fd.dlambda_dtau.isApprox(data.dlambda_dtau));

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings_fd);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_loop_closure_j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  // const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  // Add Loop Closure Constraint

  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  // const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure(
    CONTACT_3D, model, 0, SE3::Identity(), RA_id, SE3::Random(), LOCAL);
  ci_closure.corrector.Kp.array() = KP;
  ci_closure.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

void computeVelocityAndAccelerationErrors(
  const Model & model,
  const RigidConstraintModel & cmodel,
  const VectorXd & q,
  const VectorXd & v,
  const VectorXd & a,
  Motion & v_error,
  Motion & a_error,
  const VectorXd & Kp,
  const VectorXd & Kd)
{
  Data data(model);
  forwardKinematics(model, data, q, v, a);

  const SE3 oMc1 = data.oMi[cmodel.joint1_id] * cmodel.joint1_placement;
  const SE3 oMc2 = data.oMi[cmodel.joint2_id] * cmodel.joint2_placement;

  const SE3 c1Mc2 = oMc1.actInv(oMc2);

  const Motion v1 = cmodel.joint1_placement.actInv(data.v[cmodel.joint1_id]);
  const Motion v2 = cmodel.joint2_placement.actInv(data.v[cmodel.joint2_id]);

  const Motion a1 = cmodel.joint1_placement.actInv(data.a[cmodel.joint1_id]);
  const Motion a2 = cmodel.joint2_placement.actInv(data.a[cmodel.joint2_id]);

  v_error = v1 - c1Mc2.act(v2);
  a_error = a1 - c1Mc2.act(a2) + v_error.cross(c1Mc2.act(v2));
  a_error.toVector() +=
    Kd.asDiagonal() * v_error.toVector() + Kp.asDiagonal() * (-log6(c1Mc2).toVector());
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_6D_loop_closure_j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
  constraint_data, constraint_data_fd;

  // Add loop closure constraint
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);

  RigidConstraintModel ci_closure(
    CONTACT_6D, model, 0, SE3::Identity(), RA_id, SE3::Identity(), LOCAL);
  ci_closure.corrector.Kp.array() = KP;
  ci_closure.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  constraint_data_fd.push_back(RigidConstraintData(ci_closure));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 100);

  initConstraintDynamics(model, data, constraint_models);
  const VectorXd ddq0 =
    constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  BOOST_CHECK(
    prox_settings.absolute_residual <= prox_settings.absolute_accuracy
    || prox_settings.relative_residual <= prox_settings.relative_accuracy);
  //  BOOST_CHECK(prox_settings.iter == 1);

  const VectorXd a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  Motion v_error, a_error;
  computeVelocityAndAccelerationErrors(
    model, ci_closure, q, v, ddq0, v_error, a_error, ci_closure.corrector.Kp,
    ci_closure.corrector.Kd);
  BOOST_CHECK(a_error.isZero());

  const Motion constraint_velocity_error = constraint_data[0].contact_velocity_error;
  const VectorXd constraint_acceleration_error = -data.primal_rhs_contact.head(constraint_dim);
  BOOST_CHECK(constraint_velocity_error.isApprox(v_error));
  BOOST_CHECK(constraint_acceleration_error.isApprox(a_error.toVector() - data.dac_da * ddq0));

  const VectorXd lambda0 = data.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  const double alpha = 1e-8;

  // d./dq
  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd dconstraint_velocity_error_dq_fd(6, model.nv);
  dconstraint_velocity_error_dq_fd.setZero();
  MatrixXd dconstraint_velocity_error_dq2_fd(6, model.nv);
  dconstraint_velocity_error_dq2_fd.setZero();
  MatrixXd dconstraint_acceleration_error_dq_fd(6, model.nv);
  dconstraint_acceleration_error_dq_fd.setZero();

  initConstraintDynamics(model, data_fd, constraint_models);
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data_fd, prox_settings);

    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;

    Motion v_error_plus, a_error_plus;
    computeVelocityAndAccelerationErrors(
      model, ci_closure, q_plus, v, ddq0, v_error_plus, a_error_plus, ci_closure.corrector.Kp,
      ci_closure.corrector.Kd);

    const Motion & constraint_velocity_error_plus = constraint_data_fd[0].contact_velocity_error;
    const VectorXd constraint_acceleration_error_plus =
      -data_fd.primal_rhs_contact.head(constraint_dim);
    dconstraint_velocity_error_dq_fd.col(k) =
      (constraint_velocity_error_plus - constraint_velocity_error).toVector() / alpha;
    dconstraint_velocity_error_dq2_fd.col(k) = (v_error_plus - v_error).toVector() / alpha;
    dconstraint_acceleration_error_dq_fd.col(k) = (a_error_plus - a_error).toVector() / alpha;

    v_eps[k] = 0.;
  }

  BOOST_CHECK(dconstraint_velocity_error_dq_fd.isApprox(data.dvc_dq, sqrt(alpha)));
  BOOST_CHECK(dconstraint_acceleration_error_dq_fd.isApprox(data.dac_dq, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  // d./dv
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();
  MatrixXd dconstraint_velocity_error_dv_fd(6, model.nv);
  dconstraint_velocity_error_dv_fd.setZero();
  MatrixXd dconstraint_acceleration_error_dv_fd(6, model.nv);
  dconstraint_acceleration_error_dv_fd.setZero();

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data_fd, prox_settings);

    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;

    Motion v_error_plus, a_error_plus;
    computeVelocityAndAccelerationErrors(
      model, ci_closure, q, v_plus, ddq0, v_error_plus, a_error_plus, ci_closure.corrector.Kp,
      ci_closure.corrector.Kd);

    const Motion & constraint_velocity_error_plus = constraint_data_fd[0].contact_velocity_error;
    dconstraint_velocity_error_dv_fd.col(k) =
      (constraint_velocity_error_plus - constraint_velocity_error).toVector() / alpha;
    dconstraint_acceleration_error_dv_fd.col(k) = (a_error_plus - a_error).toVector() / alpha;

    v_plus[k] -= alpha;
  }

  BOOST_CHECK(dconstraint_velocity_error_dv_fd.isApprox(data.dac_da, sqrt(alpha)));
  BOOST_CHECK(dconstraint_acceleration_error_dv_fd.isApprox(data.dac_dv, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  // d./dtau
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data_fd, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_6D_loop_closure_j1j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData)
  constraint_data, constraint_data_fd;

  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  // Add loop closure constraint
  RigidConstraintModel ci_closure(
    CONTACT_6D, model, LA_id, SE3::Random(), RA_id, SE3::Random(), LOCAL);
  ci_closure.corrector.Kp.array() = KP;
  ci_closure.corrector.Kd.array() = KD;

  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  constraint_data_fd.push_back(RigidConstraintData(ci_closure));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 100);

  initConstraintDynamics(model, data, constraint_models);
  const VectorXd ddq0 =
    constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data.lambda_c;

  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  Motion v_error, a_error;
  computeVelocityAndAccelerationErrors(
    model, ci_closure, q, v, ddq0, v_error, a_error, ci_closure.corrector.Kp,
    ci_closure.corrector.Kd);
  BOOST_CHECK(a_error.isZero());

  const Motion constraint_velocity_error = constraint_data[0].contact_velocity_error;
  const VectorXd constraint_acceleration_error = -data.primal_rhs_contact.head(constraint_dim);
  BOOST_CHECK(constraint_velocity_error.isApprox(v_error));
  BOOST_CHECK(constraint_acceleration_error.isApprox(a_error.toVector() - data.dac_da * ddq0));

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;

  // d./dq
  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd dconstraint_velocity_error_dq_fd(6, model.nv);
  dconstraint_velocity_error_dq_fd.setZero();
  MatrixXd dconstraint_acceleration_error_dq_fd(6, model.nv);
  dconstraint_acceleration_error_dq_fd.setZero();

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;

    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data_fd, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;

    Motion v_error_plus, a_error_plus;
    computeVelocityAndAccelerationErrors(
      model, ci_closure, q_plus, v, ddq0, v_error_plus, a_error_plus, ci_closure.corrector.Kp,
      ci_closure.corrector.Kd);

    const Motion & constraint_velocity_error_plus = constraint_data_fd[0].contact_velocity_error;
    const VectorXd constraint_acceleration_error_plus =
      -data_fd.primal_rhs_contact.head(constraint_dim);
    dconstraint_velocity_error_dq_fd.col(k) =
      (constraint_velocity_error_plus - constraint_velocity_error).toVector() / alpha;
    dconstraint_acceleration_error_dq_fd.col(k) = (a_error_plus - a_error).toVector() / alpha;

    v_eps[k] = 0.;
  }

  BOOST_CHECK(dconstraint_velocity_error_dq_fd.isApprox(data.dvc_dq, sqrt(alpha)));
  BOOST_CHECK(dconstraint_acceleration_error_dq_fd.isApprox(data.dac_dq, sqrt(alpha)));

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  // d./dv
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();
  MatrixXd dconstraint_velocity_error_dv_fd(6, model.nv);
  dconstraint_velocity_error_dv_fd.setZero();
  MatrixXd dconstraint_acceleration_error_dv_fd(6, model.nv);
  dconstraint_acceleration_error_dv_fd.setZero();

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;

    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data_fd, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;

    Motion v_error_plus, a_error_plus;
    computeVelocityAndAccelerationErrors(
      model, ci_closure, q, v_plus, ddq0, v_error_plus, a_error_plus, ci_closure.corrector.Kp,
      ci_closure.corrector.Kd);

    const Motion & constraint_velocity_error_plus = constraint_data_fd[0].contact_velocity_error;
    dconstraint_velocity_error_dv_fd.col(k) =
      (constraint_velocity_error_plus - constraint_velocity_error).toVector() / alpha;
    dconstraint_acceleration_error_dv_fd.col(k) = (a_error_plus - a_error).toVector() / alpha;

    v_plus[k] -= alpha;
  }

  BOOST_CHECK(dconstraint_velocity_error_dv_fd.isApprox(data.dac_da, sqrt(alpha)));
  BOOST_CHECK(dconstraint_acceleration_error_dv_fd.isApprox(data.dac_dv, sqrt(alpha)));

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  // d./dtau
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data_fd, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(
  test_constraint_dynamics_derivatives_LOCAL_WORL_ALIGNED_6D_loop_closure_j1j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  // const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  // Add Loop Closure Constraint

  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure(
    CONTACT_6D, model, LA_id, SE3::Random(), RA_id, SE3::Random(), LOCAL_WORLD_ALIGNED);
  ci_closure.corrector.Kp.array() = 0.;
  ci_closure.corrector.Kd.array() = 0;

  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_loop_closure_j1j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  // const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  // Add Loop Closure Constraint

  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure(
    CONTACT_3D, model, LA_id, SE3::Random(), RA_id, SE3::Random(), LOCAL);
  ci_closure.corrector.Kp.array() = KP;
  ci_closure.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));
  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(
  test_constraint_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_loop_closure_j1j2_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  // const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  // Add Loop Closure Constraint

  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure(
    CONTACT_3D, model, LA_id, SE3::Random(), RA_id, SE3::Random(), LOCAL_WORLD_ALIGNED);

  ci_closure.corrector.Kp.array() = KP;
  ci_closure.corrector.Kd.array() = KD;

  constraint_models.push_back(ci_closure);
  constraint_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_WORLD_ALIGNED_6D_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL_WORLD_ALIGNED);
  ci_LF.corrector.Kp.array() = 0; // TODO: Add support for KP >0
  ci_LF.corrector.Kd.array() = KD;

  ci_LF.joint1_placement.setRandom();
  constraint_models.push_back(ci_LF);
  constraint_data.push_back(RigidConstraintData(ci_LF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL_WORLD_ALIGNED);
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  ci_RF.joint1_placement.setRandom();
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_mix_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);
  const std::string RH = "rarm6_joint";
  const Model::JointIndex RH_id = model.getJointId(RH);
  const std::string LH = "larm6_joint";
  const Model::JointIndex LH_id = model.getJointId(LH);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL_WORLD_ALIGNED);
  ci_LF.corrector.Kp.array() = 0; // TODO: fix local_world_aligned for 6d with kp non-zero
  ci_LF.corrector.Kd.array() = KD;
  ci_LF.joint1_placement.setRandom();
  constraint_models.push_back(ci_LF);
  constraint_data.push_back(RigidConstraintData(ci_LF));
  RigidConstraintModel ci_RF(CONTACT_6D, model, RF_id, LOCAL);
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  ci_RF.joint1_placement.setRandom();
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  RigidConstraintModel ci_LH(CONTACT_3D, model, LH_id, LOCAL_WORLD_ALIGNED);
  ci_LH.corrector.Kp.array() = KP;
  ci_LH.corrector.Kd.array() = KD;
  ci_LH.joint1_placement.setRandom();
  constraint_models.push_back(ci_LH);
  constraint_data.push_back(RigidConstraintData(ci_LH));
  RigidConstraintModel ci_RH(CONTACT_3D, model, RH_id, LOCAL);
  ci_RH.corrector.Kp.array() = KP;
  ci_RH.corrector.Kd.array() = KD;
  ci_RH.joint1_placement.setRandom();
  constraint_models.push_back(ci_RH);
  constraint_data.push_back(RigidConstraintData(ci_RH));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);

  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  const Eigen::MatrixXd Jc = data.dac_da;
  const Eigen::MatrixXd Jc_ref =
    data.contact_chol.matrix().topRightCorner(constraint_dim, model.nv);

  BOOST_CHECK(Jc.isApprox(Jc_ref));

  const Eigen::MatrixXd JMinv = Jc * data.Minv;
  const Eigen::MatrixXd dac_dq = data.dac_dq;

  Eigen::MatrixXd dac_dq_fd(constraint_dim, model.nv);

  Eigen::VectorXd contact_acc0(constraint_dim);
  Eigen::DenseIndex row_id = 0;

  forwardKinematics(model, data, q, v, data.ddq);
  for (size_t k = 0; k < constraint_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = constraint_models[k];
    const RigidConstraintData & cdata = constraint_data[k];
    const Eigen::DenseIndex size = cmodel.size();

    const Motion contact_acc = getContactAcceleration(model, data, cmodel);

    if (cmodel.type == CONTACT_3D)
      contact_acc0.segment<3>(row_id) =
        contact_acc.linear() - cdata.contact_acceleration_error.linear();
    else
      contact_acc0.segment<6>(row_id) =
        contact_acc.toVector() - cdata.contact_acceleration_error.toVector();

    row_id += size;
  }

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);

    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;

    Eigen::VectorXd contact_acc_plus(constraint_dim);
    Eigen::DenseIndex row_id = 0;
    forwardKinematics(model, data_fd, q_plus, v, data.ddq);
    for (size_t k = 0; k < constraint_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = constraint_models[k];
      const RigidConstraintData & cdata = constraint_data[k];
      const Eigen::DenseIndex size = cmodel.size();

      const Motion contact_acc = getContactAcceleration(model, data_fd, cmodel);

      if (cmodel.type == CONTACT_3D)
        contact_acc_plus.segment<3>(row_id) =
          contact_acc.linear() - cdata.contact_acceleration_error.linear();
      else
        contact_acc_plus.segment<6>(row_id) =
          contact_acc.toVector() - cdata.contact_acceleration_error.toVector();

      row_id += size;
    }

    dac_dq_fd.col(k) = (contact_acc_plus - contact_acc0) / alpha;

    v_eps[k] = 0.;
  }

  BOOST_CHECK(dac_dq_fd.isApprox(dac_dq, 1e-6));

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_data, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_data, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_loop_closure_kinematics_fd)
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
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RH = "rarm6_joint";
  const Model::JointIndex RH_id = model.getJointId(RH);
  const std::string LH = "larm6_joint";
  const Model::JointIndex LH_id = model.getJointId(LH);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_RH(CONTACT_6D, model, RH_id, SE3::Random(), LH_id, SE3::Random(), LOCAL);
  ci_RH.corrector.Kp.array() = 0;
  ci_RH.corrector.Kd.array() = 0;

  constraint_models.push_back(ci_RH);
  constraint_data.push_back(RigidConstraintData(ci_RH));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_data, prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_data, prox_settings);

  // Data_fd
  initConstraintDynamics(model, data_fd, constraint_models);
  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_data, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v, a);

  const Eigen::MatrixXd Jc = data.dac_da;
  const Eigen::MatrixXd Jc_ref =
    data.contact_chol.matrix().topRightCorner(constraint_dim, model.nv);

  BOOST_CHECK(Jc.isApprox(Jc_ref));

  const Eigen::MatrixXd JMinv = Jc * data.Minv;
  const Eigen::MatrixXd dac_dq = data.dac_dq;
  Eigen::MatrixXd dac_dq_fd(constraint_dim, model.nv);

  Eigen::VectorXd contact_acc0(constraint_dim);
  Eigen::DenseIndex row_id = 0;

  forwardKinematics(model, data, q, v, data.ddq);
  for (size_t k = 0; k < constraint_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = constraint_models[k];
    const RigidConstraintData & cdata = constraint_data[k];
    const Eigen::DenseIndex size = cmodel.size();

    const Motion contact_acc = getContactAcceleration(model, data, cmodel, cdata.c1Mc2);

    if (cmodel.type == CONTACT_3D)
      contact_acc0.segment<3>(row_id) = contact_acc.linear();
    else
      contact_acc0.segment<6>(row_id) = contact_acc.toVector();

    row_id += size;
  }

  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_data, prox_settings);

    Eigen::VectorXd contact_acc_plus(constraint_dim);
    Eigen::DenseIndex row_id = 0;
    forwardKinematics(model, data_fd, q_plus, v, data.ddq);
    for (size_t k = 0; k < constraint_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = constraint_models[k];
      const RigidConstraintData & cdata = constraint_data[k];
      const Eigen::DenseIndex size = cmodel.size();

      const Motion contact_acc = getContactAcceleration(model, data_fd, cmodel, cdata.c1Mc2);

      if (cmodel.type == CONTACT_3D)
        contact_acc_plus.segment<3>(row_id) = contact_acc.linear();
      else
        contact_acc_plus.segment<6>(row_id) = contact_acc.toVector();

      row_id += size;
    }

    dac_dq_fd.col(k) = (contact_acc_plus - contact_acc0) / alpha;

    v_eps[k] = 0.;
  }

  BOOST_CHECK(dac_dq_fd.isApprox(dac_dq, 2e-6));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_dirty_data)
{
  // Verify that a dirty data doesn't affect the results of the contact dynamics derivs
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model, true);
  Data data_dirty(model);

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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_data;

  RigidConstraintModel ci_LF(CONTACT_6D, model, LF_id, LOCAL);
  RigidConstraintModel ci_RF(CONTACT_3D, model, RF_id, LOCAL);

  ci_LF.corrector.Kp.array() = KP;
  ci_LF.corrector.Kd.array() = KD;
  ci_RF.corrector.Kp.array() = KP;
  ci_RF.corrector.Kd.array() = KD;
  constraint_models.push_back(ci_LF);
  constraint_data.push_back(RigidConstraintData(ci_LF));
  constraint_models.push_back(ci_RF);
  constraint_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12, mu0, 1);

  initConstraintDynamics(model, data_dirty, constraint_models);
  constraintDynamics(
    model, data_dirty, q, v, tau, constraint_models, constraint_data, prox_settings);
  computeConstraintDynamicsDerivatives(
    model, data_dirty, constraint_models, constraint_data, prox_settings);

  // Reuse the same data with new configurations
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  constraintDynamics(
    model, data_dirty, q, v, tau, constraint_models, constraint_data, prox_settings);
  computeConstraintDynamicsDerivatives(
    model, data_dirty, constraint_models, constraint_data, prox_settings);

  // Test with fresh data
  Data data_fresh(model);
  initConstraintDynamics(model, data_fresh, constraint_models);
  constraintDynamics(
    model, data_fresh, q, v, tau, constraint_models, constraint_data, prox_settings);
  computeConstraintDynamicsDerivatives(
    model, data_fresh, constraint_models, constraint_data, prox_settings);
  const double alpha = 1e-12;

  BOOST_CHECK(data_dirty.ddq_dq.isApprox(data_fresh.ddq_dq, sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dv.isApprox(data_fresh.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dtau.isApprox(data_fresh.ddq_dtau, sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dq.isApprox(data_fresh.dlambda_dq, sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dv.isApprox(data_fresh.dlambda_dv, sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dtau.isApprox(data_fresh.dlambda_dtau, sqrt(alpha)));
}

#ifdef PINOCCHIO_WITH_SDFORMAT

BOOST_AUTO_TEST_CASE_EXPECTED_FAILURES(test_constraint_dynamics_derivatives_cassie_proximal, 6)
BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_cassie_proximal)
{
  // TODO: 4 fd tests (ddq/dtau, ddq/dq, ddq/dv, dlambda/dq, dlambda/dv) fail for cassie
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string srdf_filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/cassie_description/srdf/cassie_v2.srdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) constraint_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) constraint_datas;

  pinocchio::sdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model, constraint_models);
  pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename, false);

  Eigen::VectorXd q = model.referenceConfigurations["standing"];
  pinocchio::normalize(model, q);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const double mu0 = 1e-5;
  ProximalSettings prox_settings(1e-12, mu0, 10);

  Data data(model), data_fd(model);

  initConstraintDynamics(model, data, constraint_models);
  for (int k = 0; k < (int)constraint_models.size(); ++k)
  {
    constraint_datas.push_back(RigidConstraintData(constraint_models[(pinocchio::JointIndex)k]));
  }

  Eigen::DenseIndex constraint_dim = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
    constraint_dim += constraint_models[k].size();

  initConstraintDynamics(model, data, constraint_models);
  constraintDynamics(model, data, q, v, tau, constraint_models, constraint_datas, prox_settings);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(
    model, data, constraint_models, constraint_datas, prox_settings);

  initConstraintDynamics(model, data_fd, constraint_models);
  MatrixXd ddq_partial_dq_fd(model.nv, model.nv);
  ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv, model.nv);
  ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv, model.nv);
  ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim, model.nv);
  lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim, model.nv);
  lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim, model.nv);
  lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(
    model, data_fd, q, v, tau, constraint_models, constraint_datas, prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);
  const double alpha = 1e-8;
  forwardKinematics(model, data, q, v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    ddq_plus = constraintDynamics(
      model, data_fd, q_plus, v, tau, constraint_models, constraint_datas, prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq, sqrt(alpha)));

  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq, sqrt(alpha)));

  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v_plus, tau, constraint_models, constraint_datas, prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv, sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv, sqrt(alpha)));

  VectorXd tau_plus(tau);
  for (int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(
      model, data_fd, q, v, tau_plus, constraint_models, constraint_datas, prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0) / alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0) / alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau, sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau, sqrt(alpha)));
}

#endif // PINOCCHIO_WITH_SDFORMAT

BOOST_AUTO_TEST_SUITE_END()
