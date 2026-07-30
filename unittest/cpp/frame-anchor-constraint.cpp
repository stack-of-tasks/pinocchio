//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE frame_anchor_constraint

#include "pinocchio/spatial.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

// Helpers
#include "constraints/jacobians-checker.hpp"

#include <iostream>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

template<typename T>
bool within(const T & elt, const std::vector<T> & vec)
{
  typename std::vector<T>::const_iterator it;

  it = std::find(vec.begin(), vec.end(), elt);
  if (it != vec.end())
    return true;
  else
    return false;
}

template<typename Matrix>
bool within(const typename Matrix::Scalar & elt, const Eigen::MatrixBase<Matrix> & mat)
{
  for (DenseIndex i = 0; i < mat.rows(); ++i)
    for (DenseIndex j = 0; j < mat.rows(); ++j)
    {
      if (elt == mat(i, j))
        return true;
    }

  return false;
}

BOOST_AUTO_TEST_CASE(basic_constructor)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  // Check complete constructor
  const SE3 M(SE3::Random());
  FrameAnchorConstraintModel cmodel2(model, 0, M);
  BOOST_CHECK(cmodel2.joint1_id == 0);
  BOOST_CHECK(cmodel2.joint1_placement == M);
  BOOST_CHECK(cmodel2.residualSize() == 6);

  // Check contructor with two arguments
  FrameAnchorConstraintModel cmodel2prime(model, 0);
  BOOST_CHECK(cmodel2prime.joint1_id == 0);
  BOOST_CHECK(cmodel2prime.joint1_placement.isIdentity(0.));
  BOOST_CHECK(cmodel2prime.residualSize() == 6);

  // Check default copy constructor
  FrameAnchorConstraintModel cmodel3(cmodel2);
  BOOST_CHECK(cmodel3 == cmodel2);
}

void check_A1_and_A2(
  const Model & model,
  const Data & data,
  const FrameAnchorConstraintModel & cmodel,
  FrameAnchorConstraintData & cdata)
{
  const FrameAnchorConstraintModel::Matrix6 A1_world = cmodel.getA1(cdata, WorldFrameTag());
  FrameAnchorConstraintModel::Matrix6 A1_world_ref = -cdata.oMc1.toActionMatrixInverse();

  BOOST_CHECK(A1_world.isApprox(A1_world_ref));

  const FrameAnchorConstraintModel::Matrix6 A2_world = cmodel.getA2(cdata, WorldFrameTag());
  const FrameAnchorConstraintModel::Matrix6 A2_world_ref = -A1_world_ref;

  BOOST_CHECK(A2_world.isApprox(A2_world_ref));

  const FrameAnchorConstraintModel::Matrix6 A1_local = cmodel.getA1(cdata, LocalFrameTag());
  FrameAnchorConstraintModel::Matrix6 A1_local_ref =
    -cmodel.joint1_placement.toActionMatrixInverse();

  BOOST_CHECK(A1_local.isApprox(A1_local_ref));

  const FrameAnchorConstraintModel::Matrix6 A2_local = cmodel.getA2(cdata, LocalFrameTag());
  const FrameAnchorConstraintModel::Matrix6 A2_local_ref =
    (cdata.c1Mc2 * cmodel.joint2_placement.inverse()).toActionMatrix();

  BOOST_CHECK(A2_local.isApprox(A2_local_ref));

  // Check Jacobians
  Data::MatrixXs J_ref(6, model.nv);
  J_ref.setZero();
  getConstraintJacobian(model, data, cmodel, cdata, J_ref);

  // World
  const Data::Matrix6x J1_world = getJointJacobian(model, data, cmodel.joint1_id, WORLD);
  const Data::Matrix6x J2_world = getJointJacobian(model, data, cmodel.joint2_id, WORLD);
  const Data::Matrix6x J_world = A1_world * J1_world + A2_world * J2_world;

  BOOST_CHECK(J_world.isApprox(J_ref));

  // Local
  const Data::Matrix6x J1_local = getJointJacobian(model, data, cmodel.joint1_id, LOCAL);
  const Data::Matrix6x J2_local = getJointJacobian(model, data, cmodel.joint2_id, LOCAL);
  const Data::Matrix6x J_local = A1_local * J1_local + A2_local * J2_local;

  BOOST_CHECK(J_local.isApprox(J_ref));
}

BOOST_AUTO_TEST_CASE(basic_operations)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);

  crba(model, data, q, Convention::WORLD);

  const std::string RF_name = "rleg6_joint";
  const std::string LF_name = "lleg6_joint";

  FrameAnchorConstraintModel cm(
    model, model.getJointId(RF_name), SE3::Random(), model.getJointId(LF_name), SE3::Random());
  FrameAnchorConstraintData cd(cm);
  cm.calc(model, data, cd);

  // Vector LOCAL
  {
    const Inertia::Vector6 diagonal_inertia(1, 2, 3, 4, 5, 6);

    const auto A1 = cm.getA1(cd, LocalFrameTag());
    const Inertia::Matrix6 I11_ref = A1.transpose() * diagonal_inertia.asDiagonal() * A1;

    const auto A2 = cm.getA2(cd, LocalFrameTag());
    const Inertia::Matrix6 I22_ref = A2.transpose() * diagonal_inertia.asDiagonal() * A2;

    const Inertia::Matrix6 I12_ref = A1.transpose() * diagonal_inertia.asDiagonal() * A2;

    Inertia::Matrix6 I11 = -Inertia::Matrix6::Ones(), I12 = -Inertia::Matrix6::Ones(),
                     I22 = -Inertia::Matrix6::Ones();

    cm.computeConstraintInertias(cd, diagonal_inertia.asDiagonal(), I11, I12, I22, LocalFrameTag());
    BOOST_CHECK(I11.isApprox(I11_ref));
    BOOST_CHECK(I12.isApprox(I12_ref));
    BOOST_CHECK(I22.isApprox(I22_ref));

    // Check against scalar signature
    const double constant_inertia_value = 10;
    const Inertia::Vector6 diagonal_inertia_scalar =
      Inertia::Vector6::Constant(constant_inertia_value);
    Inertia::Matrix6 I11_scalar = -Inertia::Matrix6::Ones(), I12_scalar = -Inertia::Matrix6::Ones(),
                     I22_scalar = -Inertia::Matrix6::Ones();

    cm.computeConstraintInertias(
      cd, diagonal_inertia_scalar.asDiagonal(), I11, I12, I22, LocalFrameTag());
    cm.computeConstraintInertias(
      cd, constant_inertia_value, I11_scalar, I12_scalar, I22_scalar, LocalFrameTag());
    BOOST_CHECK(I11 == I11_scalar);
    BOOST_CHECK(I12 == I12_scalar);
    BOOST_CHECK(I22 == I22_scalar);
  }

  // Vector WORLD
  {
    const Inertia::Vector6 diagonal_inertia(1, 2, 3, 4, 5, 6);

    const auto A1 = cm.getA1(cd, WorldFrameTag());
    const Inertia::Matrix6 I11_ref = A1.transpose() * diagonal_inertia.asDiagonal() * A1;

    const auto A2 = cm.getA2(cd, WorldFrameTag());
    const Inertia::Matrix6 I22_ref = A2.transpose() * diagonal_inertia.asDiagonal() * A2;

    const Inertia::Matrix6 I12_ref = A1.transpose() * diagonal_inertia.asDiagonal() * A2;

    Inertia::Matrix6 I11 = -Inertia::Matrix6::Ones(), I12 = -Inertia::Matrix6::Ones(),
                     I22 = -Inertia::Matrix6::Ones();

    cm.computeConstraintInertias(cd, diagonal_inertia.asDiagonal(), I11, I12, I22, WorldFrameTag());
    BOOST_CHECK(I11.isApprox(I11_ref));
    BOOST_CHECK(I12.isApprox(I12_ref));
    BOOST_CHECK(I22.isApprox(I22_ref));

    // Check against scalar signature
    const double constant_inertia_value = 10;
    const Inertia::Vector6 diagonal_inertia_scalar =
      Inertia::Vector6::Constant(constant_inertia_value);
    Inertia::Matrix6 I11_scalar = -Inertia::Matrix6::Ones(), I12_scalar = -Inertia::Matrix6::Ones(),
                     I22_scalar = -Inertia::Matrix6::Ones();

    cm.computeConstraintInertias(
      cd, diagonal_inertia_scalar.asDiagonal(), I11, I12, I22, WorldFrameTag());
    cm.computeConstraintInertias(
      cd, constant_inertia_value, I11_scalar, I12_scalar, I22_scalar, WorldFrameTag());
    BOOST_CHECK(I11 == I11_scalar);
    BOOST_CHECK(I12 == I12_scalar);
    BOOST_CHECK(I22 == I22_scalar);
  }

  // Check null values
  {
    FrameAnchorConstraintModel cm1(model, model.getJointId(RF_name), SE3::Random());
    FrameAnchorConstraintData cd1(cm1);
    cm1.calc(model, data, cd1);

    Inertia::Matrix6 I11 = -Inertia::Matrix6::Ones(), I12 = -Inertia::Matrix6::Ones(),
                     I22 = -Inertia::Matrix6::Ones();

    const double constant_inertia_value = 10;
    cm1.computeConstraintInertias(cd1, constant_inertia_value, I11, I12, I22, WorldFrameTag());
    BOOST_CHECK(!I11.isZero(0));
    BOOST_CHECK(I12.isZero(0));
    BOOST_CHECK(I22.isZero(0));

    I11.fill(-1);
    I12.fill(-1);
    I22.fill(-1);
    cm1.computeConstraintInertias(cd1, constant_inertia_value, I11, I12, I22, LocalFrameTag());
    BOOST_CHECK(!I11.isZero(0));
    BOOST_CHECK(I12.isZero(0));
    BOOST_CHECK(I22.isZero(0));

    FrameAnchorConstraintModel cm2(
      model, 0, SE3::Identity(), model.getJointId(RF_name), SE3::Random());
    FrameAnchorConstraintData cd2(cm2);
    cm2.calc(model, data, cd2);

    I11.fill(-1);
    I12.fill(-1);
    I22.fill(-1);
    cm2.computeConstraintInertias(cd2, constant_inertia_value, I11, I12, I22, WorldFrameTag());
    BOOST_CHECK(I11.isZero(0));
    BOOST_CHECK(I12.isZero(0));
    BOOST_CHECK(!I22.isZero(0));

    I11.fill(-1);
    I12.fill(-1);
    I22.fill(-1);
    cm2.computeConstraintInertias(cd2, constant_inertia_value, I11, I12, I22, LocalFrameTag());
    BOOST_CHECK(I11.isZero(0));
    BOOST_CHECK(I12.isZero(0));
    BOOST_CHECK(!I22.isZero(0));
  }
}

template<typename VectorLike>
Eigen::MatrixXd compute_jacobian_fd(
  const Model & model,
  const FrameAnchorConstraintModel & cmodel,
  const Eigen::MatrixBase<VectorLike> & q,
  const double eps)
{
  Data data_fd(model), data(model);
  FrameAnchorConstraintData cdata(cmodel), cdata_fd(cmodel);

  Eigen::MatrixXd res(cmodel.residualSize(), model.nv);
  res.setZero();

  forwardKinematics(model, data, q),

    cmodel.calc(model, data, cdata);
  Eigen::VectorXd v_plus(model.nv);
  v_plus.setZero();

  for (int i = 0; i < model.nv; ++i)
  {
    v_plus[i] = eps;
    const auto q_plus = integrate(model, q, v_plus);
    forwardKinematics(model, data_fd, q_plus),

      cmodel.calc(model, data_fd, cdata_fd);

    res.col(i) = log6(cdata_fd.c1Mc2.act(cdata.c1Mc2.inverse())).toVector() / eps;

    v_plus[i] = 0;
  }

  return res;
}

SE3 computeConstraintError(
  const Model & model, const Data & data, const FrameAnchorConstraintModel & cm)
{
  PINOCCHIO_UNUSED_VARIABLE(model);

  const SE3 oMc1 = data.oMi[cm.joint1_id] * cm.joint1_placement;
  const SE3 oMc2 = data.oMi[cm.joint2_id] * cm.joint2_placement;

  const SE3 c1Mc2 = oMc1.actInv(oMc2);

  return c1Mc2;
}

BOOST_AUTO_TEST_CASE(contact_models_sparsity_and_jacobians)
{

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  forwardKinematics(model, data, q, v);
  computeJointJacobians(model, data, q);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const double eps_fd = 1e-8;

  const FrameAnchorConstraintModel cm_RF(model, model.getJointId(RF), SE3::Random());
  const FrameAnchorConstraintModel cm_LF(model, model.getJointId(LF), SE3::Random());
  const FrameAnchorConstraintModel clm_RF_LF(
    model, cm_RF.joint1_id, cm_RF.joint1_placement, cm_LF.joint1_id, cm_LF.joint1_placement);

  // Check errors values
  {
    Data data(model);

    FrameAnchorConstraintData cd_RF(cm_RF);
    FrameAnchorConstraintData cd_LF(cm_LF);
    FrameAnchorConstraintData cld_RF_LF(clm_RF_LF);

    forwardKinematics(model, data, q);

    cm_RF.calc(model, data, cd_RF);
    const auto error_RF_ref = computeConstraintError(model, data, cm_RF);
    BOOST_CHECK(cd_RF.c1Mc2.isApprox(error_RF_ref));
    BOOST_CHECK(cd_RF.constraint_position_error.isApprox(log6(error_RF_ref).toVector()));

    cm_LF.calc(model, data, cd_LF);
    const auto error_LF_ref = computeConstraintError(model, data, cm_LF);
    BOOST_CHECK(cd_LF.c1Mc2.isApprox(error_LF_ref));

    clm_RF_LF.calc(model, data, cld_RF_LF);
    const auto error_RF_LF_ref = computeConstraintError(model, data, clm_RF_LF);
    BOOST_CHECK(cld_RF_LF.c1Mc2.isApprox(error_RF_LF_ref));
  }

  {
    forwardKinematics(model, data, q, v, a);

    FrameAnchorConstraintData cd_RF(cm_RF);
    cm_RF.calc(model, data, cd_RF);
    FrameAnchorConstraintData cd_LF(cm_LF);
    cm_LF.calc(model, data, cd_LF);
    FrameAnchorConstraintData cld_RF_LF(clm_RF_LF);
    clm_RF_LF.calc(model, data, cld_RF_LF);

    Data::Matrix6x J_RF_LOCAL(6, model.nv);
    J_RF_LOCAL.setZero();
    getFrameJacobian(model, data, cm_RF.joint1_id, cm_RF.joint1_placement, LOCAL, J_RF_LOCAL);

    const Data::Matrix6x J_RF_LOCAL_constraint = -J_RF_LOCAL;

    Data::Matrix6x J_LF_LOCAL(6, model.nv);
    J_LF_LOCAL.setZero();
    getFrameJacobian(model, data, cm_LF.joint1_id, cm_LF.joint1_placement, LOCAL, J_LF_LOCAL);

    const Data::Matrix6x J_LF_LOCAL_constraint = -J_LF_LOCAL;

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_RF.joint1_id][k]);
      BOOST_CHECK(
        J_LF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_LF.joint1_id][k]);
    }
    BOOST_CHECK(model.sparsity_pattern_vector[cm_RF.joint2_id].isZero());
    BOOST_CHECK(model.sparsity_pattern_vector[cm_LF.joint2_id].isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF.joint1_id] * clm_RF_LF.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF.joint2_id] * clm_RF_LF.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Data::Matrix6x J_clm_LOCAL = c1Mc2.toActionMatrix() * J_LF_LOCAL - J_RF_LOCAL;

    Model::EigenIndexVector colwise_span_indexes;
    clm_RF_LF.getRowIndexes(model, data, cld_RF_LF, 0, colwise_span_indexes);
    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LOCAL.col(k).isZero(0) != within(k, colwise_span_indexes));
    }

    // Check Jacobian vs sparse Jacobian computation
    Data::MatrixXs J_RF_constraint_sparse(6, model.nv);
    J_RF_constraint_sparse.setZero();
    getConstraintJacobian(model, data, cm_RF, cd_RF, J_RF_constraint_sparse);
    BOOST_CHECK(J_RF_LOCAL_constraint.isApprox(J_RF_constraint_sparse));

    const auto J_RF_constraint_fd = compute_jacobian_fd(model, cm_RF, q, eps_fd);
    BOOST_CHECK(J_RF_constraint_sparse.isApprox(J_RF_constraint_fd, sqrt(eps_fd)));

    Data::MatrixXs J_LF_constraint_sparse(6, model.nv);
    J_LF_constraint_sparse.setZero();
    getConstraintJacobian(model, data, cm_LF, cd_LF, J_LF_constraint_sparse);
    BOOST_CHECK(J_LF_LOCAL_constraint.isApprox(J_LF_constraint_sparse));

    const auto J_LF_constraint_fd = compute_jacobian_fd(model, cm_LF, q, eps_fd);
    BOOST_CHECK(J_LF_constraint_sparse.isApprox(J_LF_constraint_fd, sqrt(eps_fd)));

    Data::MatrixXs J_clm_constraint_sparse(6, model.nv);
    J_clm_constraint_sparse.setZero();
    getConstraintJacobian(model, data, clm_RF_LF, cld_RF_LF, J_clm_constraint_sparse);
    BOOST_CHECK(J_clm_LOCAL.isApprox(J_clm_constraint_sparse));

    const auto J_clm_fd = compute_jacobian_fd(model, clm_RF_LF, q, eps_fd);
    BOOST_CHECK(J_clm_constraint_sparse.isApprox(J_clm_fd, sqrt(eps_fd)));

    // Check velocity and acceleration
    {
      const double dt = eps_fd;
      FrameAnchorConstraintData cd_RF(cm_RF), cd_RF_plus(cm_RF);
      cm_RF.calc(model, data, cd_RF);

      Data data_plus(model);
      const VectorXd v_plus = v + a * dt;
      const VectorXd q_plus = integrate(model, q, v_plus * dt);
      forwardKinematics(model, data_plus, q_plus, v_plus);

      {
        FrameAnchorConstraintData cd_RF(cm_RF), cd_RF_plus(cm_RF);
        cm_RF.calc(model, data, cd_RF);
        BOOST_CHECK(cd_RF.constraint_velocity_error.isApprox(J_RF_constraint_sparse * v));

        cm_RF.calc(model, data_plus, cd_RF_plus);
        const Motion::Vector6 constraint_velocity_error_fd =
          log6(cd_RF_plus.c1Mc2.act(cd_RF.c1Mc2.inverse())).toVector() / dt;
        BOOST_CHECK(
          cd_RF.constraint_velocity_error.isApprox(constraint_velocity_error_fd, sqrt(dt)));

        const Motion::Vector6 constraint_acceleration_error_fd =
          (cd_RF_plus.constraint_velocity_error - cd_RF.constraint_velocity_error) / dt;
        BOOST_CHECK(
          cd_RF.constraint_acceleration_error.isApprox(constraint_acceleration_error_fd, sqrt(dt)));
      }

      {
        FrameAnchorConstraintData cd_LF(cm_LF), cd_LF_plus(cm_LF);
        cm_LF.calc(model, data, cd_LF);
        BOOST_CHECK(cd_LF.constraint_velocity_error.isApprox(J_LF_constraint_sparse * v));

        cm_LF.calc(model, data_plus, cd_LF_plus);
        const Motion::Vector6 constraint_velocity_error_fd =
          log6(cd_LF_plus.c1Mc2.act(cd_LF.c1Mc2.inverse())).toVector() / dt;
        BOOST_CHECK(
          cd_LF.constraint_velocity_error.isApprox(constraint_velocity_error_fd, sqrt(dt)));

        const Motion::Vector6 constraint_acceleration_error_fd =
          (cd_LF_plus.constraint_velocity_error - cd_LF.constraint_velocity_error) / dt;
        BOOST_CHECK(
          cd_LF.constraint_acceleration_error.isApprox(constraint_acceleration_error_fd, sqrt(dt)));
      }

      {
        FrameAnchorConstraintData cld_RF_LF(clm_RF_LF), cld_RF_LF_plus(clm_RF_LF);
        clm_RF_LF.calc(model, data, cld_RF_LF);
        BOOST_CHECK(cld_RF_LF.constraint_velocity_error.isApprox(J_clm_constraint_sparse * v));

        clm_RF_LF.calc(model, data_plus, cld_RF_LF_plus);
        const Motion::Vector6 constraint_velocity_error_fd =
          log6(cld_RF_LF_plus.c1Mc2.act(cld_RF_LF.c1Mc2.inverse())).toVector() / dt;
        BOOST_CHECK(
          cld_RF_LF.constraint_velocity_error.isApprox(constraint_velocity_error_fd, sqrt(dt)));

        const Motion::Vector6 constraint_acceleration_error_fd =
          (cld_RF_LF_plus.constraint_velocity_error - cld_RF_LF.constraint_velocity_error) / dt;
        BOOST_CHECK(cld_RF_LF.constraint_acceleration_error.isApprox(
          constraint_acceleration_error_fd, sqrt(dt)));
      }
    }

    check_A1_and_A2(model, data, cm_RF, cd_RF);
    check_jacobians_operations(model, data, cm_RF, cd_RF);

    check_A1_and_A2(model, data, cm_LF, cd_LF);
    check_jacobians_operations(model, data, cm_LF, cd_LF);

    check_A1_and_A2(model, data, clm_RF_LF, cld_RF_LF);
    check_jacobians_operations(model, data, clm_RF_LF, cld_RF_LF);

    // Check acceleration contributions
    {
      Data data(model), data_zero_acc(model);
      forwardKinematics(model, data, q, v, a);
      computeJointJacobians(model, data, q);
      forwardKinematics(model, data_zero_acc, q, v, VectorXd::Zero(model.nv));

      // RF
      FrameAnchorConstraintData cd_RF(cm_RF), cd_RF_zero_acc(cm_RF);
      cm_RF.calc(model, data, cd_RF);
      cm_RF.calc(model, data_zero_acc, cd_RF_zero_acc);

      Data::MatrixXs J_RF_sparse(6, model.nv);
      J_RF_sparse.setZero();
      cm_RF.jacobian(model, data, cd_RF, J_RF_sparse);

      BOOST_CHECK((J_RF_sparse * a + cd_RF_zero_acc.constraint_acceleration_error)
                    .isApprox(cd_RF.constraint_acceleration_error));

      // LF
      FrameAnchorConstraintData cd_LF(cm_LF), cd_LF_zero_acc(cm_LF);
      cm_LF.calc(model, data, cd_LF);
      cm_LF.calc(model, data_zero_acc, cd_LF_zero_acc);

      Data::MatrixXs J_LF_sparse(6, model.nv);
      J_LF_sparse.setZero();
      cm_LF.jacobian(model, data, cd_LF, J_LF_sparse);

      BOOST_CHECK((J_LF_sparse * a + cd_LF_zero_acc.constraint_acceleration_error)
                    .isApprox(cd_LF.constraint_acceleration_error));

      // Close loop
      FrameAnchorConstraintData cld_RF_LF(clm_RF_LF), cld_RF_LF_zero_acc(clm_RF_LF);
      clm_RF_LF.calc(model, data, cld_RF_LF);
      clm_RF_LF.calc(model, data_zero_acc, cld_RF_LF_zero_acc);

      Data::MatrixXs J_clm_sparse(6, model.nv);
      J_clm_sparse.setZero();
      clm_RF_LF.jacobian(model, data, cld_RF_LF, J_clm_sparse);

      BOOST_CHECK((J_clm_sparse * a + cld_RF_LF_zero_acc.constraint_acceleration_error)
                    .isApprox(cld_RF_LF.constraint_acceleration_error));
    }
  }
}

BOOST_AUTO_TEST_CASE(frame_anchor_constraint_cast)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const std::string RF = "rleg6_joint";

  const FrameAnchorConstraintModel cm_RF(model, model.getJointId(RF), SE3::Identity());
  const auto cm_RF_cast_double = cm_RF.cast<double>();
  BOOST_CHECK(cm_RF_cast_double == cm_RF);

  const auto cm_RF_cast_long_double = cm_RF.cast<long double>();
  BOOST_CHECK(cm_RF_cast_long_double.cast<double>() == cm_RF);
}

BOOST_AUTO_TEST_CASE(cholesky)
{

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  crba(model, data, q, Convention::WORLD);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  const FrameAnchorConstraintModel cm_RF(model, model.getJointId(RF), SE3::Random());
  const FrameAnchorConstraintModel cm_LF(model, model.getJointId(LF), SE3::Random());
  const FrameAnchorConstraintModel clm_RF_LF(
    model, cm_RF.joint1_id, cm_RF.joint1_placement, cm_LF.joint1_id, cm_LF.joint1_placement);

  std::vector<FrameAnchorConstraintModel> constraint_models;
  constraint_models.push_back(cm_RF);
  constraint_models.push_back(cm_LF);
  constraint_models.push_back(clm_RF_LF);

  std::vector<FrameAnchorConstraintData> constraint_datas, constraint_datas_ref;
  for (const auto & cm : constraint_models)
  {
    constraint_datas.push_back(cm.createData());
    constraint_datas_ref.push_back(cm.createData());
  }

  const double mu = 1e-10;
  calc(model, data, constraint_models, constraint_datas);
  ConstraintCholeskyDecomposition cholesky(model, data, constraint_models, constraint_datas);
  cholesky.compute(model, data, constraint_models, constraint_datas, mu);

  crba(model, data_ref, q, Convention::WORLD);
  make_symmetric(data_ref.M);
  const auto total_size = getTotalConstraintResidualSize(constraint_models);
  Eigen::MatrixXd J_constraints(total_size, model.nv);
  J_constraints.setZero();
  getConstraintsJacobian(model, data_ref, constraint_models, constraint_datas, J_constraints);

  Eigen::MatrixXd H_ref = Eigen::MatrixXd::Zero(total_size + model.nv, total_size + model.nv);
  H_ref.topLeftCorner(total_size, total_size).diagonal().fill(-mu);
  H_ref.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H_ref.topRightCorner(total_size, model.nv) = J_constraints;
  H_ref.bottomLeftCorner(model.nv, total_size) = J_constraints.transpose();

  BOOST_CHECK(cholesky.matrix().isApprox(H_ref));
}

void check_maps_impl(
  const Model & model,
  Data & data,
  const FrameAnchorConstraintModel & cm,
  FrameAnchorConstraintData & cd)
{
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  crba(model, data, q, Convention::WORLD);
  forwardKinematics(model, data, q, v);

  cm.calc(model, data, cd);
  const auto constraint_jacobian = cm.jacobian(model, data, cd);

  // Test mapConstraintForceToJointForces : WorldFrameTag
  {
    std::vector<Force> joint_forces(size_t(model.njoints), Force::Zero());
    const Motion::Vector6 constraint_force = Motion::Vector6::Ones();
    const auto joint_torque_ref = constraint_jacobian.transpose() * constraint_force;

    cm.mapConstraintForceToJointForces(
      model, data, cd, constraint_force, joint_forces, WorldFrameTag());

    for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
    {
      if (joint_id == cm.joint1_id || joint_id == cm.joint2_id)
      {
        BOOST_CHECK(!joint_forces[joint_id].isZero(0));
      }
      else
      {
        BOOST_CHECK(joint_forces[joint_id].isZero(0));
      }
    }

    // Backward pass over the joint forces
    Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(model.nv);
    for (JointIndex joint_id = JointIndex(model.njoints) - 1; joint_id > 0; --joint_id)
    {
      const JointModel & jmodel = model.joints[joint_id];
      const auto joint_nv = jmodel.nv();
      const auto joint_idx_v = jmodel.idx_v();

      joint_torque.segment(joint_idx_v, joint_nv) =
        data.J.middleCols(joint_idx_v, joint_nv).transpose() * joint_forces[joint_id].toVector();

      const JointIndex parent_id = model.parents[joint_id];
      joint_forces[parent_id] += joint_forces[joint_id];
    }

    BOOST_CHECK(joint_torque.isApprox(joint_torque_ref));
  }

  // Test mapConstraintForceToJointForces : LocalFrameTag
  {
    std::vector<Force> joint_forces(size_t(model.njoints), Force::Zero());
    const Motion::Vector6 constraint_force = Motion::Vector6::Ones();
    const auto joint_torque_ref = constraint_jacobian.transpose() * constraint_force;

    cm.mapConstraintForceToJointForces(
      model, data, cd, constraint_force, joint_forces, LocalFrameTag());

    for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
    {
      if (joint_id == cm.joint1_id || joint_id == cm.joint2_id)
      {
        BOOST_CHECK(!joint_forces[joint_id].isZero(0));
      }
      else
      {
        BOOST_CHECK(joint_forces[joint_id].isZero(0));
      }
    }

    // Backward pass over the joint forces
    Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(model.nv);
    for (JointIndex joint_id = JointIndex(model.njoints) - 1; joint_id > 0; --joint_id)
    {
      const JointModel & jmodel = model.joints[joint_id];
      const JointData & jdata = data.joints[joint_id];
      const auto joint_nv = jmodel.nv();
      const auto joint_idx_v = jmodel.idx_v();

      joint_torque.segment(joint_idx_v, joint_nv) =
        jdata.S().matrix().transpose() * joint_forces[joint_id].toVector();

      const JointIndex parent_id = model.parents[joint_id];
      joint_forces[parent_id] += data.liMi[joint_id].act(joint_forces[joint_id]);
    }

    BOOST_CHECK(joint_torque.isApprox(joint_torque_ref));
  }

  // Test mapJointMotionsToConstraintMotion : WorldFrameTag
  {
    const auto constraint_motion_ref = constraint_jacobian * v;
    for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
    {
      data.ov[joint_id] = data.oMi[joint_id].act(data.v[joint_id]);
    }

    const auto & joint_accelerations = data.ov;
    Motion::Vector6 constraint_motion = Motion::Vector6::Zero();
    cm.mapJointMotionsToConstraintMotion(
      model, data, cd, joint_accelerations, constraint_motion, WorldFrameTag());

    BOOST_CHECK(constraint_motion.isApprox(constraint_motion_ref));
  }

  // Test mapJointMotionsToConstraintMotion : LocalFrameTag
  {
    const auto constraint_motion_ref = constraint_jacobian * v;

    const auto & joint_motions = data.v;
    Motion::Vector6 constraint_motion = Motion::Vector6::Zero();
    cm.mapJointMotionsToConstraintMotion(
      model, data, cd, joint_motions, constraint_motion, LocalFrameTag());

    BOOST_CHECK(constraint_motion.isApprox(constraint_motion_ref));
  }
}

BOOST_AUTO_TEST_CASE(check_maps)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  const FrameAnchorConstraintModel cm_RF(model, model.getJointId(RF), SE3::Random());
  auto cd_RF = cm_RF.createData();

  const FrameAnchorConstraintModel cm_LF(model, model.getJointId(LF), SE3::Random());
  auto cd_LF = cm_LF.createData();
  const FrameAnchorConstraintModel clm_RF_LF(
    model, cm_RF.joint1_id, cm_RF.joint1_placement, cm_LF.joint1_id, cm_LF.joint1_placement);
  auto cld_RF_LF = clm_RF_LF.createData();

  check_maps_impl(model, data, cm_RF, cd_RF);
  check_maps_impl(model, data, cm_LF, cd_LF);
  check_maps_impl(model, data, clm_RF_LF, cld_RF_LF);
}

BOOST_AUTO_TEST_CASE(compliance)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const std::string RF = "rleg6_joint";
  FrameAnchorConstraintModel cmodel(model, model.getJointId(RF), SE3::Random());

  {
    // check retrieve compliance
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == Eigen::VectorXd::Zero(cmodel.residualSize()));
  }

  {
    // check set compliance
    Eigen::VectorXd compliance_ref = Eigen::VectorXd::Random(cmodel.residualSize()).cwiseAbs();
    cmodel.setCompliance(compliance_ref);
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == compliance_ref);
  }
}
