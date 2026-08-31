//
// Copyright (c) 2026 INRIA
//

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

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;
using namespace Eigen;

namespace
{
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  /// \brief Distance between the two points of the constraint, computed from scratch.
  double computeDistance(const Data & data, const ConstantLengthConstraintModel & cmodel)
  {
    const SE3 oMc1 = data.oMi[cmodel.joint1_id] * cmodel.joint1_placement;
    const SE3 oMc2 = data.oMi[cmodel.joint2_id] * cmodel.joint2_placement;

    return (oMc2.translation() - oMc1.translation()).norm();
  }

  /// \brief The associated point constraint, i.e. the constraint on the very same pair of points
  /// but without the projection along the direction joining them.
  PointAnchorConstraintModel
  associatedPointConstraint(const Model & model, const ConstantLengthConstraintModel & cmodel)
  {
    return PointAnchorConstraintModel(
      model, cmodel.joint1_id, cmodel.joint1_placement, cmodel.joint2_id, cmodel.joint2_placement);
  }

  template<typename VectorLike>
  Eigen::MatrixXd compute_jacobian_fd(
    const Model & model,
    const ConstantLengthConstraintModel & cmodel,
    const Eigen::MatrixBase<VectorLike> & q,
    const double eps)
  {
    Data data_fd(model), data(model);
    ConstantLengthConstraintData cdata(cmodel), cdata_fd(cmodel);

    Eigen::MatrixXd res(1, model.nv);
    res.setZero();

    forwardKinematics(model, data, q);
    cmodel.calc(model, data, cdata);

    Eigen::VectorXd v_plus(model.nv);
    v_plus.setZero();

    for (int i = 0; i < model.nv; ++i)
    {
      v_plus[i] = eps;
      const auto q_plus = integrate(model, q, v_plus);
      forwardKinematics(model, data_fd, q_plus);
      cmodel.calc(model, data_fd, cdata_fd);

      res.col(i) = (cdata_fd.constraint_position_error - cdata.constraint_position_error) / eps;

      v_plus[i] = 0;
    }

    return res;
  }
} // namespace

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(basic_constructor)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const SE3 M1(SE3::Random()), M2(SE3::Random());
  const double length = 0.125;

  // Check complete constructor
  ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), M1, model.getJointId(LF), M2, length);
  BOOST_CHECK(cmodel.joint1_id == model.getJointId(RF));
  BOOST_CHECK(cmodel.joint2_id == model.getJointId(LF));
  BOOST_CHECK(cmodel.joint1_placement == M1);
  BOOST_CHECK(cmodel.joint2_placement == M2);
  BOOST_CHECK(cmodel.getLength() == length);
  BOOST_CHECK(cmodel.residualSize() == 1);

  // Check the constructors inherited from the binary kinematics constraints
  ConstantLengthConstraintModel cmodel_from_joint1(model, model.getJointId(RF), M1);
  BOOST_CHECK(cmodel_from_joint1.joint1_id == model.getJointId(RF));
  BOOST_CHECK(cmodel_from_joint1.joint2_id == 0);
  BOOST_CHECK(cmodel_from_joint1.joint1_placement == M1);
  BOOST_CHECK(cmodel_from_joint1.getLength() == 0.);
  BOOST_CHECK(cmodel_from_joint1.residualSize() == 1);

  ConstantLengthConstraintModel cmodel_from_model(model);
  BOOST_CHECK(cmodel_from_model.joint1_id == 0);
  BOOST_CHECK(cmodel_from_model.joint2_id == 0);
  BOOST_CHECK(cmodel_from_model.joint1_placement.isIdentity(0.));

  // Check the length setter
  cmodel_from_joint1.setLength(length);
  BOOST_CHECK(cmodel_from_joint1.getLength() == length);
  BOOST_CHECK_THROW(cmodel_from_joint1.setLength(-1.), std::invalid_argument);

  // Check default copy constructor
  ConstantLengthConstraintModel cmodel_copy(cmodel);
  BOOST_CHECK(cmodel_copy == cmodel);

  // The length takes part in the comparison
  cmodel_copy.setLength(2. * length);
  BOOST_CHECK(cmodel_copy != cmodel);
}

BOOST_AUTO_TEST_CASE(constraint_position_error)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);

  forwardKinematics(model, data, q);

  ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.);
  ConstantLengthConstraintData cdata(cmodel);

  const double distance_ref = computeDistance(data, cmodel);
  BOOST_CHECK(distance_ref > 1e-3); // sanity check: we are away from the singularity

  // Zero length: the residual is the distance itself
  cmodel.calc(model, data, cdata);
  BOOST_CHECK_CLOSE(cdata.distance, distance_ref, 1e-10);
  BOOST_CHECK_CLOSE(cdata.constraint_position_error[0], distance_ref, 1e-10);
  BOOST_CHECK_CLOSE(cdata.direction.norm(), 1., 1e-10);
  BOOST_CHECK(cdata.direction.isApprox(cdata.relative_position / distance_ref));
  BOOST_CHECK(cdata.relative_position.isApprox(cdata.c1Mc2.translation()));

  // The residual vanishes exactly on the constraint manifold
  cmodel.setLength(distance_ref);
  cmodel.calc(model, data, cdata);
  BOOST_CHECK_SMALL(cdata.constraint_position_error[0], 1e-12);

  // Sign convention: too far apart -> positive, too close -> negative
  cmodel.setLength(distance_ref / 2.);
  cmodel.calc(model, data, cdata);
  BOOST_CHECK(cdata.constraint_position_error[0] > 0.);

  cmodel.setLength(2. * distance_ref);
  cmodel.calc(model, data, cdata);
  BOOST_CHECK(cdata.constraint_position_error[0] < 0.);

  // The generic offset shifts the residual, exactly as the length does
  cmodel.setLength(distance_ref);
  cmodel.desired_constraint_offset[0] = 0.01;
  cmodel.calc(model, data, cdata);
  BOOST_CHECK_CLOSE(cdata.constraint_position_error[0], -0.01, 1e-8);
}

BOOST_AUTO_TEST_CASE(rigid_displacement_invariance)
{
  // Both points carried by moving bodies: a rigid displacement of the whole system
  // leaves the distance, hence the residual, unchanged.
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_moved(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);

  const ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.3);
  ConstantLengthConstraintData cdata(cmodel), cdata_moved(cmodel);

  forwardKinematics(model, data, q);
  cmodel.calc(model, data, cdata);

  // The root joint of humanoidRandom is a free flyer: move it around.
  VectorXd q_moved = q;
  const SE3 root_displacement(SE3::Random());
  const SE3 root_placement(
    Eigen::Quaterniond(q[6], q[3], q[4], q[5]).toRotationMatrix(), q.head<3>());
  const SE3 root_placement_moved = root_displacement * root_placement;
  q_moved.head<3>() = root_placement_moved.translation();
  const Eigen::Quaterniond quat_moved(root_placement_moved.rotation());
  q_moved[3] = quat_moved.x();
  q_moved[4] = quat_moved.y();
  q_moved[5] = quat_moved.z();
  q_moved[6] = quat_moved.w();

  forwardKinematics(model, data_moved, q_moved);
  cmodel.calc(model, data_moved, cdata_moved);

  BOOST_CHECK_CLOSE(cdata_moved.distance, cdata.distance, 1e-10);
  BOOST_CHECK_SMALL(
    cdata_moved.constraint_position_error[0] - cdata.constraint_position_error[0], 1e-12);
}

BOOST_AUTO_TEST_CASE(constraint_projectors_and_jacobians)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd a = VectorXd::Random(model.nv);

  forwardKinematics(model, data, q, v, a);
  computeJointJacobians(model, data, q);

  const double eps_fd = 1e-8;

  std::vector<ConstantLengthConstraintModel> cmodels;
  // Anchored on the universe
  cmodels.push_back(ConstantLengthConstraintModel(
    model, model.getJointId(RF), SE3::Random(), 0, SE3::Random(), 1.));
  // Closing a kinematic loop between two moving bodies
  cmodels.push_back(ConstantLengthConstraintModel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.5));

  for (const ConstantLengthConstraintModel & cmodel : cmodels)
  {
    ConstantLengthConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);

    // The point constraint the constant length constraint is built upon
    const PointAnchorConstraintModel point_cmodel = associatedPointConstraint(model, cmodel);
    PointAnchorConstraintData point_cdata(point_cmodel);
    point_cmodel.calc(model, data, point_cdata);

    BOOST_CHECK(point_cdata.constraint_position_error.isApprox(cdata.relative_position));

    // The projectors are the projectors of the point constraint, projected on the direction
    // joining the two points.
    const ConstantLengthConstraintModel::MatrixSize6 A1_world =
      cmodel.getA1(cdata, WorldFrameTag());
    const ConstantLengthConstraintModel::MatrixSize6 A2_world =
      cmodel.getA2(cdata, WorldFrameTag());
    BOOST_CHECK(A1_world.isApprox(
      cdata.direction.transpose() * point_cmodel.getA1(point_cdata, WorldFrameTag())));
    BOOST_CHECK(A2_world.isApprox(
      cdata.direction.transpose() * point_cmodel.getA2(point_cdata, WorldFrameTag())));
    BOOST_CHECK(A1_world.isApprox(cdata.A1_world));
    BOOST_CHECK(A2_world.isApprox(cdata.A2_world));

    const ConstantLengthConstraintModel::MatrixSize6 A1_local =
      cmodel.getA1(cdata, LocalFrameTag());
    const ConstantLengthConstraintModel::MatrixSize6 A2_local =
      cmodel.getA2(cdata, LocalFrameTag());
    BOOST_CHECK(A1_local.isApprox(
      cdata.direction.transpose() * point_cmodel.getA1(point_cdata, LocalFrameTag())));
    BOOST_CHECK(A2_local.isApprox(
      cdata.direction.transpose() * point_cmodel.getA2(point_cdata, LocalFrameTag())));
    BOOST_CHECK(A1_local.isApprox(cdata.A1_local));
    BOOST_CHECK(A2_local.isApprox(cdata.A2_local));

    // A rigid displacement of the whole system does not change the distance, hence in the world
    // frame the two projectors cancel each other out.
    BOOST_CHECK(cdata.A_world.isZero());

    // Jacobian
    Data::MatrixXs J(1, model.nv);
    J.setZero();
    getConstraintJacobian(model, data, cmodel, cdata, J);

    // ... against the projection of the point constraint Jacobian
    Data::MatrixXs point_J(3, model.nv);
    point_J.setZero();
    getConstraintJacobian(model, data, point_cmodel, point_cdata, point_J);
    BOOST_CHECK(J.isApprox(cdata.direction.transpose() * point_J));

    // ... against the projectors
    const Data::Matrix6x J1_world = getJointJacobian(model, data, cmodel.joint1_id, WORLD);
    const Data::Matrix6x J2_world = getJointJacobian(model, data, cmodel.joint2_id, WORLD);
    BOOST_CHECK(J.isApprox(A1_world * J1_world + A2_world * J2_world));

    const Data::Matrix6x J1_local = getJointJacobian(model, data, cmodel.joint1_id, LOCAL);
    const Data::Matrix6x J2_local = getJointJacobian(model, data, cmodel.joint2_id, LOCAL);
    BOOST_CHECK(J.isApprox(A1_local * J1_local + A2_local * J2_local));

    // ... against finite differences
    const auto J_fd = compute_jacobian_fd(model, cmodel, q, eps_fd);
    BOOST_CHECK(J.isApprox(J_fd, sqrt(eps_fd)));

    // The Jacobian sparsity matches the one advertised by the constraint
    Model::EigenIndexVector colwise_span_indexes;
    cmodel.getRowIndexes(model, data, cdata, 0, colwise_span_indexes);
    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      const bool within_span =
        std::find(colwise_span_indexes.begin(), colwise_span_indexes.end(), k)
        != colwise_span_indexes.end();
      BOOST_CHECK(J.col(k).isZero(0) != within_span);
    }

    check_jacobians_operations(model, data, cmodel, cdata);
  }
}

BOOST_AUTO_TEST_CASE(constraint_velocity_and_acceleration_errors)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd a = VectorXd::Random(model.nv);

  forwardKinematics(model, data, q, v, a);
  computeJointJacobians(model, data, q);

  const ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.5);
  ConstantLengthConstraintData cdata(cmodel);
  cmodel.calc(model, data, cdata);

  Data::MatrixXs J(1, model.nv);
  J.setZero();
  cmodel.jacobian(model, data, cdata, J);

  // Velocity error is the Jacobian applied to the joint velocity
  BOOST_CHECK((cdata.constraint_velocity_error).isApprox(J * v));

  // Velocity and acceleration errors against finite differences
  {
    const double dt = 1e-8;
    Data data_plus(model);
    const VectorXd v_plus = v + a * dt;
    const VectorXd q_plus = integrate(model, q, v_plus * dt);
    forwardKinematics(model, data_plus, q_plus, v_plus);

    ConstantLengthConstraintData cdata_plus(cmodel);
    cmodel.calc(model, data_plus, cdata_plus);

    const auto constraint_velocity_error_fd =
      (cdata_plus.constraint_position_error - cdata.constraint_position_error) / dt;
    BOOST_CHECK(cdata.constraint_velocity_error.isApprox(constraint_velocity_error_fd, sqrt(dt)));

    const auto constraint_acceleration_error_fd =
      (cdata_plus.constraint_velocity_error - cdata.constraint_velocity_error) / dt;
    BOOST_CHECK(
      cdata.constraint_acceleration_error.isApprox(constraint_acceleration_error_fd, sqrt(dt)));
  }

  // The acceleration error is affine in the joint acceleration, with the Jacobian as linear part
  {
    Data data_zero_acc(model);
    forwardKinematics(model, data_zero_acc, q, v, VectorXd::Zero(model.nv));

    ConstantLengthConstraintData cdata_zero_acc(cmodel);
    cmodel.calc(model, data_zero_acc, cdata_zero_acc);

    BOOST_CHECK((J * a + cdata_zero_acc.constraint_acceleration_error)
                  .isApprox(cdata.constraint_acceleration_error));
  }
}

BOOST_AUTO_TEST_CASE(map_constraint_force_and_joint_motions)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);

  forwardKinematics(model, data, q, v);
  computeJointJacobians(model, data, q);

  const ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.5);
  ConstantLengthConstraintData cdata(cmodel);
  cmodel.calc(model, data, cdata);

  Data::MatrixXs J(1, model.nv);
  J.setZero();
  cmodel.jacobian(model, data, cdata, J);

  // Mapping a constraint force to the joints and back to the generalized torques must give
  // J^T * lambda.
  const ConstantLengthConstraintModel::ResidualVectorType lambda =
    ConstantLengthConstraintModel::ResidualVectorType::Random();

  typedef Data::Force Force;
  std::vector<Force> joint_forces(size_t(model.njoints), Force::Zero());
  cmodel.mapConstraintForceToJointForces(model, data, cdata, lambda, joint_forces, WorldFrameTag());

  VectorXd tau_ref = J.transpose() * lambda;
  VectorXd tau = VectorXd::Zero(model.nv);
  for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
  {
    const Data::Matrix6x J_joint = getJointJacobian(model, data, joint_id, WORLD);
    tau += J_joint.transpose() * joint_forces[joint_id].toVector();
  }
  BOOST_CHECK(tau.isApprox(tau_ref));

  // Mapping joint motions to the constraint motion must give J * v.
  std::vector<Motion> joint_motions(size_t(model.njoints), Motion::Zero());
  for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
  {
    const Data::Matrix6x J_joint = getJointJacobian(model, data, joint_id, WORLD);
    joint_motions[joint_id] = Motion(Data::Vector6(J_joint * v));
  }

  ConstantLengthConstraintModel::ResidualVectorType constraint_motion;
  cmodel.mapJointMotionsToConstraintMotion(
    model, data, cdata, joint_motions, constraint_motion, WorldFrameTag());
  BOOST_CHECK(constraint_motion.isApprox(J * v));
}

BOOST_AUTO_TEST_CASE(cast)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const ConstantLengthConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Identity(), model.getJointId(LF), SE3::Identity(), 0.75);

  const auto cmodel_cast_double = cmodel.cast<double>();
  BOOST_CHECK(cmodel_cast_double == cmodel);

  const auto cmodel_cast_long_double = cmodel.cast<long double>();
  BOOST_CHECK(cmodel_cast_long_double.getLength() == 0.75L);
  BOOST_CHECK(cmodel_cast_long_double.cast<double>() == cmodel);
}

BOOST_AUTO_TEST_CASE(compliance)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  ConstantLengthConstraintModel cmodel(model, model.getJointId(RF), SE3::Random());

  {
    // check retrieve compliance
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == Eigen::VectorXd::Zero(cmodel.residualSize()));
  }

  {
    // check set compliance
    const Eigen::VectorXd compliance_ref =
      Eigen::VectorXd::Random(cmodel.residualSize()).cwiseAbs();
    cmodel.setCompliance(compliance_ref);
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == compliance_ref);
  }
}

BOOST_AUTO_TEST_CASE(variant)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);

  forwardKinematics(model, data, q);
  computeJointJacobians(model, data, q);

  const ConstantLengthConstraintModel cmodel_(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.5);
  ConstantLengthConstraintData cdata_(cmodel_);
  cmodel_.calc(model, data, cdata_);

  // The constraint is part of the default constraint collection
  const ConstraintModel cmodel(cmodel_);
  ConstraintData cdata(cmodel.createData());
  BOOST_CHECK(cmodel.residualSize() == 1);
  cmodel.calc(model, data, cdata);

  Data::MatrixXs J(1, model.nv), J_(1, model.nv);
  J.setZero();
  J_.setZero();
  cmodel.jacobian(model, data, cdata, J);
  cmodel_.jacobian(model, data, cdata_, J_);
  BOOST_CHECK(J.isApprox(J_));
}

BOOST_AUTO_TEST_CASE(cholesky)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);

  crba(model, data, q, Convention::WORLD);

  std::vector<ConstantLengthConstraintModel> constraint_models;
  constraint_models.push_back(ConstantLengthConstraintModel(
    model, model.getJointId(RF), SE3::Random(), 0, SE3::Random(), 1.));
  constraint_models.push_back(ConstantLengthConstraintModel(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random(), 0.5));

  std::vector<ConstantLengthConstraintData> constraint_datas;
  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const double mu = 1e-10;
  calc(model, data, constraint_models, constraint_datas);
  ConstraintCholeskyDecomposition cholesky(model, data, constraint_models, constraint_datas);
  cholesky.compute(model, data, constraint_models, constraint_datas, mu);

  crba(model, data_ref, q, Convention::WORLD);
  make_symmetric(data_ref.M);
  const auto total_size = getTotalConstraintResidualSize(constraint_models);
  BOOST_CHECK(total_size == 2);

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

BOOST_AUTO_TEST_SUITE_END()
