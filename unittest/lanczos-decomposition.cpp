//
// Copyright (c) 2024 INRIA
//

#include <iostream>

#include <pinocchio/math/lanczos-decomposition.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::DenseIndex mat_size = 20;
  const Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(mat_size, mat_size);

  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(identity_matrix, 10);

  const auto residual = lanczos_decomposition.computeDecompositionResidual(identity_matrix);
  BOOST_CHECK(residual.isZero());

  BOOST_CHECK(lanczos_decomposition.rank() == 1);
  BOOST_CHECK((lanczos_decomposition.Qs().transpose() * lanczos_decomposition.Qs())
                .topLeftCorner(lanczos_decomposition.rank(), lanczos_decomposition.rank())
                .isIdentity());
}

BOOST_AUTO_TEST_CASE(test_diagonal_matrix)
{
  const Eigen::DenseIndex mat_size = 20;
  const Eigen::VectorXd diagonal_terms = Eigen::VectorXd::LinSpaced(mat_size, 0.0, mat_size - 1);
  const Eigen::MatrixXd diagonal_matrix = diagonal_terms.asDiagonal();

  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  LanczosDecomposition lanczos_decomposition(diagonal_matrix, 10);

  const auto residual = lanczos_decomposition.computeDecompositionResidual(diagonal_matrix);
  BOOST_CHECK(residual.isZero());

  for (Eigen::DenseIndex col_id = 0; col_id < lanczos_decomposition.Ts().cols(); ++col_id)
  {
    BOOST_CHECK(math::fabs(lanczos_decomposition.Qs().col(col_id).norm() - 1.) <= 1e-12);
  }

  BOOST_CHECK(lanczos_decomposition.rank() == lanczos_decomposition.Ts().cols());
  BOOST_CHECK((lanczos_decomposition.Qs().transpose() * lanczos_decomposition.Qs()).isIdentity());
}

BOOST_AUTO_TEST_CASE(test_random)
{
  typedef LanczosDecompositionTpl<Eigen::MatrixXd> LanczosDecomposition;
  const Eigen::DenseIndex mat_size = 20;

  for (int it = 0; it < 1000; ++it)
  {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd matrix = A.transpose() * A;

    LanczosDecomposition lanczos_decomposition(matrix, 10);

    const auto residual = lanczos_decomposition.computeDecompositionResidual(matrix);

    const auto & Ts = lanczos_decomposition.Ts();
    const auto & Qs = lanczos_decomposition.Qs();
    const Eigen::MatrixXd residual_bis = matrix * Qs - Qs * Ts.matrix();
    const Eigen::MatrixXd residual_tierce = Qs.transpose() * matrix * Qs - Ts.matrix();
    const Eigen::MatrixXd residual_fourth = matrix - Qs * Ts.matrix() * Qs.transpose();
    BOOST_CHECK(residual.isZero());

    for (Eigen::DenseIndex col_id = 0; col_id < lanczos_decomposition.Ts().cols(); ++col_id)
    {
      BOOST_CHECK(math::fabs(lanczos_decomposition.Qs().col(col_id).norm() - 1.) <= 1e-12);
    }
    // Check orthonormality
    BOOST_CHECK(lanczos_decomposition.rank() == lanczos_decomposition.Ts().cols());
    BOOST_CHECK((lanczos_decomposition.Qs().transpose() * lanczos_decomposition.Qs()).isIdentity());
  }
}

BOOST_AUTO_TEST_SUITE_END()
