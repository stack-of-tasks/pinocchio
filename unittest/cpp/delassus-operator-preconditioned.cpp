#define BOOST_TEST_MODULE delassus_operator_preconditioned

#include <pinocchio/algorithm/delassus-operator.hpp>
#include <pinocchio/math.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(delassus_dense_preconditioned)
{
  const Eigen::Index mat_size = 3;
  const Eigen::MatrixXd mat = Eigen::MatrixXd::Random(mat_size, mat_size);
  const Eigen::MatrixXd symmetric_mat = mat.transpose() * mat;
  const Eigen::VectorXd diag_vec = 1e-1 * Eigen::VectorXd::Ones(mat_size);
  const Eigen::MatrixXd preconditioner_matrix = diag_vec.asDiagonal();
  const Eigen::MatrixXd preconditioned_matrix =
    preconditioner_matrix * symmetric_mat * preconditioner_matrix;

  BOOST_CHECK(isSymmetric(symmetric_mat));
  BOOST_CHECK(isSymmetric(preconditioned_matrix));

  DelassusOperatorDense delassus(symmetric_mat);
  DiagonalPreconditionerTpl<Eigen::VectorXd> diag_preconditioner(diag_vec);
  DelassusOperatorPreconditionedTpl<
    DelassusOperatorDense, DiagonalPreconditionerTpl<Eigen::VectorXd>>
    delassus_preconditioned(delassus, diag_preconditioner);

  Eigen::VectorXd res(mat_size);
  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(mat_size);

  // Checking matrix() method
  BOOST_CHECK(preconditioned_matrix.isApprox(delassus_preconditioned.matrix()));

  // Checking apply on the right
  delassus_preconditioned.applyOnTheRight(rhs, res);
  BOOST_CHECK(res.isApprox((preconditioned_matrix * rhs).eval()));

  // Checking damping and compliance values
  BOOST_CHECK(delassus_preconditioned.getDamping().isZero(0));
  BOOST_CHECK(delassus_preconditioned.getCompliance().isZero(0));

  // Checking solved
  Eigen::VectorXd damping_vec = 5e-3 * Eigen::VectorXd::Ones(mat_size);
  Eigen::MatrixXd damping = damping_vec.asDiagonal();
  delassus_preconditioned.updateDamping(damping_vec);
  delassus_preconditioned.updateDecomposition();
  BOOST_CHECK(delassus_preconditioned.getDamping().isApprox(damping_vec));

  delassus_preconditioned.solve(rhs, res);
  const Eigen::MatrixXd preconditioned_matrix_inv = (preconditioned_matrix + damping).inverse();
  Eigen::VectorXd res_solve = preconditioned_matrix_inv * rhs;
  BOOST_CHECK(res.isApprox(res_solve));

  // Checking solveInPlace
  delassus_preconditioned.solveInPlace(rhs);
  BOOST_CHECK(rhs.isApprox(res_solve));

  // Checking updateDamping
  double new_damping = 1e-3;
  const Eigen::MatrixXd damped_preconditioned_matrix =
    preconditioned_matrix + new_damping * Eigen::MatrixXd::Identity(mat_size, mat_size);
  delassus_preconditioned.updateDamping(new_damping);
  delassus_preconditioned.applyOnTheRight(rhs, res);
  Eigen::VectorXd res_apply = damped_preconditioned_matrix * rhs;
  BOOST_CHECK(res.isApprox(res_apply));

  // Checking updateCompliance
  const double new_compliance = 3e-3;
  delassus.updateDamping(0.);
  delassus.updateCompliance(new_compliance);
  DelassusOperatorPreconditionedTpl<
    DelassusOperatorDense, DiagonalPreconditionerTpl<Eigen::VectorXd>>
    delassus_preconditioned2(delassus, diag_preconditioner);
  const Eigen::MatrixXd preconditioned_compliant_matrix =
    preconditioner_matrix
    * (symmetric_mat + new_compliance * Eigen::MatrixXd::Identity(mat_size, mat_size))
    * preconditioner_matrix;
  new_damping = 8e-3;
  const Eigen::MatrixXd damped_preconditioned_compliant_matrix =
    preconditioned_compliant_matrix + new_damping * Eigen::MatrixXd::Identity(mat_size, mat_size);
  delassus_preconditioned2.updateDamping(new_damping);
  BOOST_CHECK(damped_preconditioned_compliant_matrix.isApprox(delassus_preconditioned2.matrix()));
  delassus_preconditioned2.applyOnTheRight(rhs, res);
  Eigen::VectorXd res_apply2 = damped_preconditioned_compliant_matrix * rhs;
  BOOST_CHECK(res.isApprox(res_apply2));
}
