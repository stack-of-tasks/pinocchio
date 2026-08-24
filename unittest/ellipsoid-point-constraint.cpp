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

  // A thoracic-sized ellipsoid, as in the scapulothoracic models.
  const Vector3d RADII(0.082998, 0.199991, 0.083001);

  /// \brief The point constraint the ellipsoid constraint is built upon.
  PointAnchorConstraintModel
  associatedPointConstraint(const Model & model, const EllipsoidPointConstraintModel & cmodel)
  {
    return PointAnchorConstraintModel(
      model, cmodel.joint1_id, cmodel.joint1_placement, cmodel.joint2_id, cmodel.joint2_placement);
  }

  /// \brief Residual recomputed from scratch, straight from the definition.
  double computeResidual(
    const Data & data, const EllipsoidPointConstraintModel & cmodel, const Vector3d & radii)
  {
    const SE3 oMc1 = data.oMi[cmodel.joint1_id] * cmodel.joint1_placement;
    const SE3 oMc2 = data.oMi[cmodel.joint2_id] * cmodel.joint2_placement;
    const Vector3d x = oMc1.actInv(oMc2.translation());

    const Vector3d Ax = x.cwiseQuotient(radii.cwiseProduct(radii));
    return (x.dot(Ax) - 1.0) / (2.0 * Ax.norm());
  }

  /// \brief Places the material point at `scale` times the radius along the direction `u`, so
  /// that scale == 1 puts it exactly on the surface.
  SE3 placementOnEllipsoid(const Vector3d & u, const Vector3d & radii, const double scale)
  {
    return SE3(Matrix3d::Identity(), scale * radii.cwiseProduct(u.normalized()));
  }

  template<typename VectorLike>
  Eigen::MatrixXd compute_jacobian_fd(
    const Model & model,
    const EllipsoidPointConstraintModel & cmodel,
    const Eigen::MatrixBase<VectorLike> & q,
    const double eps)
  {
    Data data_fd(model), data(model);
    EllipsoidPointConstraintData cdata(cmodel), cdata_fd(cmodel);

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

  EllipsoidPointConstraintModel cmodel(
    model, model.getJointId(RF), M1, model.getJointId(LF), M2, RADII);
  BOOST_CHECK(cmodel.joint1_id == model.getJointId(RF));
  BOOST_CHECK(cmodel.joint2_id == model.getJointId(LF));
  BOOST_CHECK(cmodel.joint1_placement == M1);
  BOOST_CHECK(cmodel.joint2_placement == M2);
  BOOST_CHECK(cmodel.getRadii() == RADII);
  BOOST_CHECK(cmodel.residualSize() == 1);

  // The constructors inherited from the binary kinematics constraints default to a unit sphere.
  EllipsoidPointConstraintModel cmodel_from_joint1(model, model.getJointId(RF), M1);
  BOOST_CHECK(cmodel_from_joint1.joint2_id == 0);
  BOOST_CHECK(cmodel_from_joint1.getRadii() == Vector3d::Ones());
  BOOST_CHECK(cmodel_from_joint1.residualSize() == 1);

  // Radii must be strictly positive
  BOOST_CHECK_THROW(cmodel_from_joint1.setRadii(Vector3d(1., 0., 1.)), std::invalid_argument);
  BOOST_CHECK_THROW(cmodel_from_joint1.setRadii(Vector3d(1., -1., 1.)), std::invalid_argument);
  cmodel_from_joint1.setRadii(RADII);
  BOOST_CHECK(cmodel_from_joint1.getRadii() == RADII);

  // Check default copy constructor, and that the radii take part in the comparison
  EllipsoidPointConstraintModel cmodel_copy(cmodel);
  BOOST_CHECK(cmodel_copy == cmodel);
  cmodel_copy.setRadii(2.0 * RADII);
  BOOST_CHECK(cmodel_copy != cmodel);
}

BOOST_AUTO_TEST_CASE(residual_on_the_surface)
{
  // The residual vanishes exactly on the surface, whatever the pose of the ellipsoid.
  pinocchio::Model model;
  Data data(model);
  const VectorXd q = VectorXd::Zero(0);
  forwardKinematics(model, data, q);

  const SE3 M_ellipsoid(SE3::Random()); // ellipsoid carried by the universe, arbitrary pose

  srand(42);
  for (int k = 0; k < 50; ++k)
  {
    const Vector3d u = Vector3d::Random().normalized();
    const Vector3d x_on_surface = RADII.cwiseProduct(u);
    const SE3 point_placement(Matrix3d::Identity(), M_ellipsoid.act(x_on_surface));

    const EllipsoidPointConstraintModel cmodel(model, 0, M_ellipsoid, 0, point_placement, RADII);
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);

    BOOST_CHECK_SMALL(cdata.constraint_position_error[0], 1e-14);
    BOOST_CHECK_SMALL(cdata.algebraic_error, 1e-14);
  }
}

BOOST_AUTO_TEST_CASE(residual_sign_and_reference_value)
{
  pinocchio::Model model;
  Data data(model);
  forwardKinematics(model, data, VectorXd::Zero(0));

  const SE3 M_ellipsoid(SE3::Random());
  const Vector3d u = Vector3d(0.3, -0.7, 0.5).normalized();

  // Outside the ellipsoid -> positive, inside -> negative.
  {
    const Vector3d x_outside = 1.3 * RADII.cwiseProduct(u);
    const SE3 point_placement(Matrix3d::Identity(), M_ellipsoid.act(x_outside));
    const EllipsoidPointConstraintModel cmodel(model, 0, M_ellipsoid, 0, point_placement, RADII);
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);
    BOOST_CHECK(cdata.constraint_position_error[0] > 0.);
    BOOST_CHECK(cdata.algebraic_error > 0.);
    BOOST_CHECK_CLOSE(
      cdata.constraint_position_error[0], computeResidual(data, cmodel, RADII), 1e-8);
  }
  {
    const Vector3d x_inside = 0.7 * RADII.cwiseProduct(u);
    const SE3 point_placement(Matrix3d::Identity(), M_ellipsoid.act(x_inside));
    const EllipsoidPointConstraintModel cmodel(model, 0, M_ellipsoid, 0, point_placement, RADII);
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);
    BOOST_CHECK(cdata.constraint_position_error[0] < 0.);
    BOOST_CHECK(cdata.algebraic_error < 0.);
    BOOST_CHECK_CLOSE(
      cdata.constraint_position_error[0], computeResidual(data, cmodel, RADII), 1e-8);
  }
}

BOOST_AUTO_TEST_CASE(sphere_case)
{
  // For a sphere of radius R, the residual is exactly (||x|| - R) (||x|| + R) / (2 ||x||):
  // the signed distance to the surface times the known factor (||x|| + R) / (2 ||x||), which
  // tends to 1 on the surface. This is what makes the residual a first order distance.
  pinocchio::Model model;
  Data data(model);
  forwardKinematics(model, data, VectorXd::Zero(0));

  const double R = 0.35;
  const Vector3d radii = Vector3d::Constant(R);
  const Vector3d u = Vector3d(1.0, 2.0, -0.5).normalized();

  for (const double distance : {1e-3, 5e-3, 5e-2, 3e-1})
  {
    const Vector3d x = (R + distance) * u;
    const EllipsoidPointConstraintModel cmodel(
      model, 0, SE3::Identity(), 0, SE3(Matrix3d::Identity(), x), radii);
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);

    const double norm = x.norm();
    const double expected = (norm - R) * (norm + R) / (2.0 * norm);
    BOOST_CHECK_CLOSE(cdata.constraint_position_error[0], expected, 1e-8);

    // ... and it is the exact distance to first order.
    BOOST_CHECK_CLOSE(cdata.constraint_position_error[0], distance, 100.0 * distance / R);
  }
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

  std::vector<EllipsoidPointConstraintModel> cmodels;
  // Ellipsoid carried by the universe, point on a moving body
  cmodels.push_back(EllipsoidPointConstraintModel(
    model, 0, SE3::Random(), model.getJointId(RF), SE3::Random(), RADII));
  // Both the ellipsoid and the point carried by moving bodies
  cmodels.push_back(EllipsoidPointConstraintModel(
    model, model.getJointId(LF), SE3::Random(), model.getJointId(RF), SE3::Random(), RADII));

  for (const EllipsoidPointConstraintModel & cmodel : cmodels)
  {
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);

    const PointAnchorConstraintModel point_cmodel = associatedPointConstraint(model, cmodel);
    PointAnchorConstraintData point_cdata(point_cmodel);
    point_cmodel.calc(model, data, point_cdata);

    BOOST_CHECK(point_cdata.constraint_position_error.isApprox(cdata.relative_position));

    // The projectors are those of the point constraint, projected on the residual gradient.
    const EllipsoidPointConstraintModel::MatrixSize6 A1_world =
      cmodel.getA1(cdata, WorldFrameTag());
    const EllipsoidPointConstraintModel::MatrixSize6 A2_world =
      cmodel.getA2(cdata, WorldFrameTag());
    BOOST_CHECK(A1_world.isApprox(
      cdata.gradient.transpose() * point_cmodel.getA1(point_cdata, WorldFrameTag())));
    BOOST_CHECK(A2_world.isApprox(
      cdata.gradient.transpose() * point_cmodel.getA2(point_cdata, WorldFrameTag())));
    BOOST_CHECK(A1_world.isApprox(cdata.A1_world));
    BOOST_CHECK(A2_world.isApprox(cdata.A2_world));

    const EllipsoidPointConstraintModel::MatrixSize6 A1_local =
      cmodel.getA1(cdata, LocalFrameTag());
    const EllipsoidPointConstraintModel::MatrixSize6 A2_local =
      cmodel.getA2(cdata, LocalFrameTag());
    BOOST_CHECK(A1_local.isApprox(
      cdata.gradient.transpose() * point_cmodel.getA1(point_cdata, LocalFrameTag())));
    BOOST_CHECK(A2_local.isApprox(
      cdata.gradient.transpose() * point_cmodel.getA2(point_cdata, LocalFrameTag())));
    BOOST_CHECK(A1_local.isApprox(cdata.A1_local));
    BOOST_CHECK(A2_local.isApprox(cdata.A2_local));

    // Moving the ellipsoid and the point together does not change the residual.
    BOOST_CHECK(cdata.A_world.isZero());

    Data::MatrixXs J(1, model.nv);
    J.setZero();
    getConstraintJacobian(model, data, cmodel, cdata, J);

    Data::MatrixXs point_J(3, model.nv);
    point_J.setZero();
    getConstraintJacobian(model, data, point_cmodel, point_cdata, point_J);
    BOOST_CHECK(J.isApprox(cdata.gradient.transpose() * point_J));

    const Data::Matrix6x J1_world = getJointJacobian(model, data, cmodel.joint1_id, WORLD);
    const Data::Matrix6x J2_world = getJointJacobian(model, data, cmodel.joint2_id, WORLD);
    BOOST_CHECK(J.isApprox(A1_world * J1_world + A2_world * J2_world));

    const Data::Matrix6x J1_local = getJointJacobian(model, data, cmodel.joint1_id, LOCAL);
    const Data::Matrix6x J2_local = getJointJacobian(model, data, cmodel.joint2_id, LOCAL);
    BOOST_CHECK(J.isApprox(A1_local * J1_local + A2_local * J2_local));

    const auto J_fd = compute_jacobian_fd(model, cmodel, q, eps_fd);
    BOOST_CHECK(J.isApprox(J_fd, sqrt(eps_fd)));

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

BOOST_AUTO_TEST_CASE(jacobian_away_from_the_surface)
{
  // The gradient includes the derivative of the 1 / (2 ||A x||) scaling, so it is exact
  // everywhere and not only on the manifold. Dropping that term would give a few percent of
  // error a handful of millimetres away from the surface.
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  const VectorXd q = randomConfiguration(model);
  forwardKinematics(model, data, q);
  computeJointJacobians(model, data, q);

  const double eps_fd = 1e-8;
  const Vector3d u = Vector3d(0.4, 0.6, -0.7).normalized();
  const JointIndex joint_id = model.getJointId(RF);

  // The point is placed at 1 + scale times the radius along u: 0 is on the surface.
  for (const double scale : {0.0, 0.006, 0.06, 0.6})
  {
    const SE3 point_placement = placementOnEllipsoid(u, RADII, 1.0 + scale);
    const EllipsoidPointConstraintModel cmodel(
      model, 0, SE3::Random(), joint_id, point_placement, RADII);
    EllipsoidPointConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);

    Data::MatrixXs J(1, model.nv);
    J.setZero();
    cmodel.jacobian(model, data, cdata, J);

    const auto J_fd = compute_jacobian_fd(model, cmodel, q, eps_fd);
    BOOST_CHECK(J.isApprox(J_fd, sqrt(eps_fd)));
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

  const EllipsoidPointConstraintModel cmodel(
    model, model.getJointId(LF), SE3::Random(), model.getJointId(RF), SE3::Random(), RADII);
  EllipsoidPointConstraintData cdata(cmodel);
  cmodel.calc(model, data, cdata);

  Data::MatrixXs J(1, model.nv);
  J.setZero();
  cmodel.jacobian(model, data, cdata, J);

  BOOST_CHECK(cdata.constraint_velocity_error.isApprox(J * v));

  {
    const double dt = 1e-8;
    Data data_plus(model);
    const VectorXd v_plus = v + a * dt;
    const VectorXd q_plus = integrate(model, q, v_plus * dt);
    forwardKinematics(model, data_plus, q_plus, v_plus);

    EllipsoidPointConstraintData cdata_plus(cmodel);
    cmodel.calc(model, data_plus, cdata_plus);

    const auto constraint_velocity_error_fd =
      (cdata_plus.constraint_position_error - cdata.constraint_position_error) / dt;
    BOOST_CHECK(cdata.constraint_velocity_error.isApprox(constraint_velocity_error_fd, sqrt(dt)));

    const auto constraint_acceleration_error_fd =
      (cdata_plus.constraint_velocity_error - cdata.constraint_velocity_error) / dt;
    BOOST_CHECK(
      cdata.constraint_acceleration_error.isApprox(constraint_acceleration_error_fd, sqrt(dt)));
  }

  {
    Data data_zero_acc(model);
    forwardKinematics(model, data_zero_acc, q, v, VectorXd::Zero(model.nv));

    EllipsoidPointConstraintData cdata_zero_acc(cmodel);
    cmodel.calc(model, data_zero_acc, cdata_zero_acc);

    BOOST_CHECK((J * a + cdata_zero_acc.constraint_acceleration_error)
                  .isApprox(cdata.constraint_acceleration_error));
  }
}

BOOST_AUTO_TEST_CASE(cast)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  const EllipsoidPointConstraintModel cmodel(
    model, model.getJointId(RF), SE3::Identity(), model.getJointId(LF), SE3::Identity(), RADII);

  const auto cmodel_cast_double = cmodel.cast<double>();
  BOOST_CHECK(cmodel_cast_double == cmodel);

  const auto cmodel_cast_long_double = cmodel.cast<long double>();
  BOOST_CHECK(cmodel_cast_long_double.cast<double>() == cmodel);
}

BOOST_AUTO_TEST_CASE(compliance)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  EllipsoidPointConstraintModel cmodel(model, model.getJointId(RF), SE3::Random());

  {
    Eigen::VectorXd compliance(cmodel.residualSize());
    cmodel.retrieveCompliance(compliance);
    BOOST_CHECK(compliance == Eigen::VectorXd::Zero(cmodel.residualSize()));
  }

  {
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

  const EllipsoidPointConstraintModel cmodel_(
    model, 0, SE3::Random(), model.getJointId(RF), SE3::Random(), RADII);
  EllipsoidPointConstraintData cdata_(cmodel_);
  cmodel_.calc(model, data, cdata_);

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

  std::vector<EllipsoidPointConstraintModel> constraint_models;
  constraint_models.push_back(EllipsoidPointConstraintModel(
    model, 0, SE3::Random(), model.getJointId(RF), SE3::Random(), RADII));
  constraint_models.push_back(EllipsoidPointConstraintModel(
    model, model.getJointId(LF), SE3::Random(), model.getJointId(RF), SE3::Random(), RADII));

  std::vector<EllipsoidPointConstraintData> constraint_datas;
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
