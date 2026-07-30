//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE delassus_operations

#include <pinocchio/fwd.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/utility/binary.hpp>

#include <pinocchio/algorithm/delassus-operator.hpp>
#include <pinocchio/constraints.hpp>

#include "utils.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace pinocchio::unittest;

BOOST_AUTO_TEST_CASE(delassus_dense_rebuild)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  const double damping_val = 1e-4;

  {
    // Test rebuild from cholesky delassus
    // we need to compute the mass matrix for the cholesky
    crba(scene.model, scene.data, scene.q, Convention::WORLD);
    ConstraintCholeskyDecomposition chol(
      scene.model, scene.data, scene.constraint_models, scene.constraint_datas);
    BOOST_CHECK(chol.getCompliance().isApprox(compliance));
    chol.updateDamping(damping_val);
    chol.compute(scene.model, scene.data, scene.constraint_models, scene.constraint_datas);

    DelassusOperatorDense delassus_from_chol;
    delassus_from_chol.rebuild(chol.getDelassusOperatorCholeskyExpression());

    // -- test compliance and damping
    BOOST_CHECK(delassus_from_chol.getCompliance().isApprox(compliance));
    Eigen::MatrixXd damping_mat = damping_val * Eigen::MatrixXd::Identity(size, size);
    BOOST_CHECK(delassus_from_chol.getDamping().matrix().isApprox(damping_mat));

    // -- test undamped matrix
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    const bool enforce_symmetry = true;
    BOOST_CHECK(delassus_from_chol.undampedMatrix(enforce_symmetry).isApprox(mat));

    // -- test matrix
    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();
    BOOST_CHECK(delassus_from_chol.matrix(enforce_symmetry).isApprox(damped_mat));
  }

  {
    // Test rebuild from rigid body delassus
    typedef ConstrainedHumanoidScene<double>::ConstraintModel ConstraintModel;
    typedef DelassusOperatorRigidBodySystemsTpl<
      double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
      DelassusOperatorRigidBody;

    DelassusOperatorRigidBody delassus_rigid_body(
      internal::helper::make_ref(scene.model), internal::helper::make_ref(scene.data),
      internal::helper::make_ref(scene.constraint_models),
      internal::helper::make_ref(scene.constraint_datas), damping_val);
    BOOST_CHECK(delassus_rigid_body.getCompliance().isApprox(compliance));
    delassus_rigid_body.updateDamping(damping_val);
    delassus_rigid_body.compute();

    DelassusOperatorDense delassus_dense_from_rigid_body;
    delassus_dense_from_rigid_body.rebuild(delassus_rigid_body);

    // -- test compliance and damping
    BOOST_CHECK(delassus_dense_from_rigid_body.getCompliance().isApprox(compliance));
    BOOST_CHECK(
      delassus_dense_from_rigid_body.getCompliance().isApprox(delassus_rigid_body.getCompliance()));
    BOOST_CHECK(delassus_dense_from_rigid_body.getDamping() == delassus_rigid_body.getDamping());

    // -- test undamped matrix
    const bool enforce_symmetry = true;
    BOOST_CHECK(delassus_dense_from_rigid_body.undampedMatrix(enforce_symmetry)
                  .isApprox(delassus_rigid_body.undampedMatrix(enforce_symmetry)));

    // -- test matrix
    const bool with_damping = true;
    BOOST_CHECK(delassus_dense_from_rigid_body.matrix(enforce_symmetry, with_damping)
                  .isApprox(delassus_rigid_body.matrix(enforce_symmetry, with_damping)));
  }
}

BOOST_AUTO_TEST_CASE(delassus_dense_diag_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  const double damping_val = 1e-4;

  // Use the dense matrix from scene
  // Note: Scene computes J * Minv * J^T, which is the undamped/uncompliant part.
  DelassusOperatorDense delassus(scene.delassus_matrix_gt);
  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight
    delassus.applyOnTheRight(rhs, res);
    BOOST_CHECK(res.isApprox(scene.delassus_matrix_gt * rhs));
  }

  {
    // Test applyOnTheRight with damping
    delassus.updateDamping(damping_val);
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus =
      scene.delassus_matrix_gt + damping_val * Eigen::MatrixXd::Identity(size, size);
    BOOST_CHECK(res.isApprox(damped_delassus * rhs));
  }

  {
    // Test applyOnTheRight without damping
    delassus.updateDamping(damping_val);
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    BOOST_CHECK(res.isApprox(scene.delassus_matrix_gt * rhs));
  }

  {
    // Test solveInPlace
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test solveInPlace with compliance
    delassus.updateDamping(damping_val);
    delassus.updateCompliance(compliance);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test matrix()
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat));
  }

  {
    // Test undampedMatrix()
    // updating damping should not affect the undamped matrix.
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat));
  }
}

BOOST_AUTO_TEST_CASE(delassus_dense_block_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  internal::BlockDiagonalMatrix block_damping;
  constructPositiveDefiniteBlockDiagonalMatrix(scene.constraint_models, block_damping);

  // Use the dense matrix from scene
  // Note: Scene computes J * Minv * J^T, which is the undamped/uncompliant part.
  DelassusOperatorDense delassus(scene.delassus_matrix_gt);
  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight
    delassus.applyOnTheRight(rhs, res);
    BOOST_CHECK(res.isApprox(scene.delassus_matrix_gt * rhs));
  }

  {
    // Test applyOnTheRight with damping
    delassus.updateDamping(block_damping);
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus = scene.delassus_matrix_gt + block_damping.matrix();
    BOOST_CHECK(res.isApprox(damped_delassus * rhs));
  }

  {
    // Test applyOnTheRight without damping
    delassus.updateDamping(block_damping);
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    BOOST_CHECK(res.isApprox(scene.delassus_matrix_gt * rhs));
  }

  {
    // Test solveInPlace
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test solveInPlace with compliance
    delassus.updateDamping(block_damping);
    delassus.updateCompliance(compliance);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test matrix()
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat));
  }

  {
    // Test undampedMatrix()
    // updating damping should not affect the undamped matrix.
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat));
  }
}

BOOST_AUTO_TEST_CASE(delassus_rigid_body_rebuild)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  const double damping_val = 1e-4;

  typedef ConstrainedHumanoidScene<double>::ConstraintModel ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBody;

  DelassusOperatorRigidBody delassus_rigid_body(
    internal::helper::make_ref(scene.model), internal::helper::make_ref(scene.data),
    internal::helper::make_ref(scene.constraint_models),
    internal::helper::make_ref(scene.constraint_datas), damping_val);
  BOOST_CHECK(delassus_rigid_body.getCompliance().isApprox(compliance));
  delassus_rigid_body.updateDamping(damping_val);
  delassus_rigid_body.compute();

  // Test rebuild from rigid body delassus to another rigid body delassus
  DelassusOperatorRigidBody delassus_rigid_body_rebuilt(
    internal::helper::make_ref(scene.model), internal::helper::make_ref(scene.data),
    internal::helper::make_ref(scene.constraint_models),
    internal::helper::make_ref(scene.constraint_datas), damping_val);
  delassus_rigid_body_rebuilt.rebuild(
    internal::helper::make_ref(scene.model), internal::helper::make_ref(scene.data),
    internal::helper::make_ref(scene.constraint_models),
    internal::helper::make_ref(scene.constraint_datas));

  // -- test compliance and damping
  BOOST_CHECK(delassus_rigid_body_rebuilt.getCompliance().isApprox(compliance));
  BOOST_CHECK(
    delassus_rigid_body_rebuilt.getCompliance().isApprox(delassus_rigid_body.getCompliance()));

  // -- test undamped matrix
  const bool enforce_symmetry = true;
  delassus_rigid_body_rebuilt.compute();
  BOOST_CHECK(delassus_rigid_body_rebuilt.undampedMatrix(enforce_symmetry)
                .isApprox(delassus_rigid_body.undampedMatrix(enforce_symmetry)));

  // -- test matrix
  const bool with_damping = true;
  BOOST_CHECK(delassus_rigid_body_rebuilt.matrix(enforce_symmetry, with_damping)
                .isApprox(delassus_rigid_body.matrix(enforce_symmetry, with_damping)));
}

BOOST_AUTO_TEST_CASE(delassus_rigid_body_diag_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  const double damping_val = 1e-4;

  typedef ConstrainedHumanoidScene<double>::ConstraintModel ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBody;

  DelassusOperatorRigidBody delassus(
    internal::helper::make_ref(scene.model),             //
    internal::helper::make_ref(scene.data),              //
    internal::helper::make_ref(scene.constraint_models), //
    internal::helper::make_ref(scene.constraint_datas),  //
    damping_val);
  delassus.compute();

  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight without damping
    delassus.updateDamping(0.0);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::VectorXd expected = scene.delassus_matrix_gt * rhs;
    expected += compliance.cwiseProduct(rhs);
    BOOST_CHECK(res.isApprox(expected));
  }

  {
    // Test applyOnTheRight with damping
    delassus.updateDamping(damping_val);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus =
      scene.delassus_matrix_gt + damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(damped_delassus * rhs));
  }

  {
    // Test applyOnTheRight without damping (explicit flag)
    delassus.updateDamping(damping_val);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::MatrixXd undamped_delassus = scene.delassus_matrix_gt;
    undamped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(undamped_delassus * rhs));
  }

  {
    // Test solveInPlace
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test solveInPlace with updated compliance
    Eigen::VectorXd new_compliance = 2.0 * compliance;
    delassus.updateDamping(damping_val);
    delassus.updateCompliance(new_compliance);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += new_compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test matrix()
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat));
  }

  {
    // Test undampedMatrix()
    // updating damping should not affect the undamped matrix.
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat));
  }
}

BOOST_AUTO_TEST_CASE(delassus_rigid_body_block_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  internal::BlockDiagonalMatrix block_damping;
  constructPositiveDefiniteBlockDiagonalMatrix(scene.constraint_models, block_damping);

  typedef ConstrainedHumanoidScene<double>::ConstraintModel ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBody;

  DelassusOperatorRigidBody delassus(
    internal::helper::make_ref(scene.model), internal::helper::make_ref(scene.data),
    internal::helper::make_ref(scene.constraint_models),
    internal::helper::make_ref(scene.constraint_datas), 1e-8);
  delassus.compute();

  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight without damping
    delassus.updateDamping(0.0);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::VectorXd expected = scene.delassus_matrix_gt * rhs;
    expected += compliance.cwiseProduct(rhs);
    BOOST_CHECK(res.isApprox(expected));
  }

  {
    // Test applyOnTheRight with block damping
    delassus.updateDamping(block_damping);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus = scene.delassus_matrix_gt + block_damping.matrix();
    damped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(damped_delassus * rhs));
  }

  {
    // Test applyOnTheRight without damping (explicit flag)
    delassus.updateDamping(block_damping);
    delassus.compute();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::MatrixXd undamped_delassus = scene.delassus_matrix_gt;
    undamped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(undamped_delassus * rhs));
  }

  {
    // Test solveInPlace
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test solveInPlace with updated compliance
    Eigen::VectorXd new_compliance = 2.0 * compliance;
    delassus.updateDamping(block_damping);
    delassus.updateCompliance(new_compliance);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += new_compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-8));
  }

  {
    // Test matrix()
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat));
  }

  {
    // Test undampedMatrix()
    // updating damping should not affect the undamped matrix.
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat));
  }
}

BOOST_AUTO_TEST_CASE(delassus_cholesky_expression_diag_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  const double damping_val = 1e-4;

  // Build a ConstraintCholeskyDecomposition and compute
  crba(scene.model, scene.data, scene.q, Convention::WORLD);
  ConstraintCholeskyDecomposition chol(
    scene.model, scene.data, scene.constraint_models, scene.constraint_datas);
  chol.updateDamping(damping_val);
  chol.compute(scene.model, scene.data, scene.constraint_models, scene.constraint_datas);

  auto delassus = chol.getDelassusOperatorCholeskyExpression();

  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight without damping
    delassus.updateDamping(0.0);
    delassus.updateDecomposition();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::VectorXd expected = scene.delassus_matrix_gt * rhs;
    expected += compliance.cwiseProduct(rhs);
    BOOST_CHECK(res.isApprox(expected, 1e-6));
  }

  {
    // Test applyOnTheRight with scalar damping
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus =
      scene.delassus_matrix_gt + damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(damped_delassus * rhs, 1e-6));
  }

  {
    // Test applyOnTheRight without damping (explicit flag)
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::MatrixXd undamped_delassus = scene.delassus_matrix_gt;
    undamped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(undamped_delassus * rhs, 1e-6));
  }

  {
    // Test solveInPlace with scalar damping
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-6));
  }

  {
    // Test matrix() with damping
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += damping_val * Eigen::MatrixXd::Identity(size, size);
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat, 1e-6));
  }

  {
    // Test undampedMatrix()
    delassus.updateCompliance(compliance);
    delassus.updateDamping(damping_val);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat, 1e-6));
  }

  {
    // Test getDamping() after scalar update
    delassus.updateDamping(damping_val);
    BOOST_CHECK(
      delassus.getDamping().matrix().isApprox(damping_val * Eigen::MatrixXd::Identity(size, size)));
  }
}

BOOST_AUTO_TEST_CASE(delassus_cholesky_expression_block_operations)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  BOOST_CHECK(compliance.minCoeff() >= 0);
  internal::BlockDiagonalMatrix block_damping;
  constructPositiveDefiniteBlockDiagonalMatrix(scene.constraint_models, block_damping);

  // Build a ConstraintCholeskyDecomposition and compute
  crba(scene.model, scene.data, scene.q, Convention::WORLD);
  ConstraintCholeskyDecomposition chol(
    scene.model, scene.data, scene.constraint_models, scene.constraint_datas);
  chol.compute(scene.model, scene.data, scene.constraint_models, scene.constraint_datas);

  auto delassus = chol.getDelassusOperatorCholeskyExpression();

  Eigen::VectorXd res(size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);

  {
    // Test applyOnTheRight with block damping
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();
    delassus.applyOnTheRight(rhs, res);
    Eigen::MatrixXd damped_delassus = scene.delassus_matrix_gt + block_damping.matrix();
    damped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(damped_delassus * rhs, 1e-6));
  }

  {
    // Test applyOnTheRight without damping (explicit flag)
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();
    delassus.applyOnTheRight(rhs, res, false /*no damping*/);
    Eigen::MatrixXd undamped_delassus = scene.delassus_matrix_gt;
    undamped_delassus += compliance.asDiagonal();
    BOOST_CHECK(res.isApprox(undamped_delassus * rhs, 1e-6));
  }

  {
    // Test solveInPlace with block damping
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-6));
  }

  {
    // Test solveInPlace with updated compliance
    Eigen::VectorXd new_compliance = 2.0 * compliance;
    delassus.updateDamping(block_damping);
    delassus.updateCompliance(new_compliance);
    delassus.updateDecomposition();
    Eigen::VectorXd sol = rhs;
    delassus.solveInPlace(sol);

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += new_compliance.asDiagonal();
    Eigen::VectorXd sol_ref = damped_mat.ldlt().solve(rhs);

    BOOST_CHECK(sol.isApprox(sol_ref, 1e-6));
  }

  {
    // Test matrix() with block damping
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    Eigen::MatrixXd damped_mat = scene.delassus_matrix_gt;
    damped_mat += block_damping.matrix();
    damped_mat += compliance.asDiagonal();

    const bool enforce_symmetry = true;
    const bool with_damping = true;
    BOOST_CHECK(delassus.matrix(enforce_symmetry, with_damping).isApprox(damped_mat, 1e-6));
  }

  {
    // Test undampedMatrix() with block damping
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    delassus.updateDecomposition();

    const bool enforce_symmetry = true;
    Eigen::MatrixXd mat = scene.delassus_matrix_gt;
    mat += compliance.asDiagonal();
    BOOST_CHECK(delassus.undampedMatrix(enforce_symmetry).isApprox(mat, 1e-6));
  }

  {
    // Test getDamping() after block update
    delassus.updateDamping(block_damping);
    BOOST_CHECK(delassus.getDamping() == block_damping);
  }

  {
    // Test consistency of cholesky expression and dense operator with block damping
    delassus.updateCompliance(compliance);
    delassus.updateDamping(block_damping);
    DelassusOperatorDense delassus_dense(delassus);

    BOOST_CHECK(delassus_dense.getDamping() == block_damping);
    BOOST_CHECK(delassus_dense.getCompliance().isApprox(compliance));

    delassus.updateDecomposition();
    delassus_dense.updateDecomposition();

    const Eigen::MatrixXd rhs_mat = Eigen::MatrixXd::Random(size, 3);
    Eigen::MatrixXd res_chol(size, 3), res_dense(size, 3);
    delassus.applyOnTheRight(rhs_mat, res_chol);
    delassus_dense.applyOnTheRight(rhs_mat, res_dense);
    BOOST_CHECK(res_chol.isApprox(res_dense, 1e-6));
  }
}

BOOST_AUTO_TEST_CASE(delassus_cholesky_expression_unsafe)
{
  // scene
  ConstrainedHumanoidScene<double> scene(true, true, true, true, true);
  const Eigen::Index size = scene.delassus_matrix_gt.rows();
  Eigen::VectorXd compliance = scene.compliance;
  const double damping_val = 1e-4;

  crba(scene.model, scene.data, scene.q, Convention::WORLD);
  ConstraintCholeskyDecomposition chol(
    scene.model, scene.data, scene.constraint_models, scene.constraint_datas);
  chol.compute(scene.model, scene.data, scene.constraint_models, scene.constraint_datas);

  auto delassus = chol.getDelassusOperatorCholeskyExpression();
  delassus.updateDamping(damping_val);

  // Test unsafe().damping() gives direct write access to the block diagonal damping
  internal::BlockDiagonalMatrix block_damping;
  constructPositiveDefiniteBlockDiagonalMatrix(scene.constraint_models, block_damping);

  delassus.unsafe().damping() = block_damping;
  delassus.unsafe().makeDirty();

  BOOST_CHECK(delassus.getDamping() == block_damping);
  BOOST_CHECK(delassus.isDirty());

  delassus.updateDecomposition();
  BOOST_CHECK(!delassus.isDirty());

  // Verify correctness: applyOnTheRight should use block_damping
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(size);
  Eigen::VectorXd res(size);
  delassus.applyOnTheRight(rhs, res);

  Eigen::MatrixXd expected_mat = scene.delassus_matrix_gt + block_damping.matrix();
  expected_mat += compliance.asDiagonal();
  BOOST_CHECK(res.isApprox(expected_mat * rhs, 1e-6));
}
