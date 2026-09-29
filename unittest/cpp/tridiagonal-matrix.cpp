//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE tridiagonal_matrix

#include <pinocchio/math.hpp>

#include <Eigen/Eigenvalues>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_zero)
{
  const Eigen::Index mat_size = 20;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);

  tridiagonal_matrix.setZero();
  BOOST_CHECK(tridiagonal_matrix.isZero(0));
  BOOST_CHECK(tridiagonal_matrix.isDiagonal(0));
  BOOST_CHECK(tridiagonal_matrix.matrix().isZero(0));
}

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::Index mat_size = 20;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);
  typedef TridiagonalSymmetricMatrixTpl<double>::PlainMatrixType PlainMatrixType;

  tridiagonal_matrix.setIdentity();
  BOOST_CHECK(tridiagonal_matrix.isIdentity(0));
  BOOST_CHECK(tridiagonal_matrix.isDiagonal(0));

  BOOST_CHECK(tridiagonal_matrix.rows() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.cols() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.diagonal().size() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.subDiagonal().size() == mat_size - 1);

  BOOST_CHECK(tridiagonal_matrix.diagonal().isOnes(0));
  BOOST_CHECK(tridiagonal_matrix.subDiagonal().isZero(0));
  BOOST_CHECK(tridiagonal_matrix.matrix().isIdentity(0));

  // Fill matrix
  {
    PlainMatrixType mat(mat_size, mat_size);
    mat = tridiagonal_matrix.eigen();

    BOOST_CHECK(mat.isIdentity(0));
  }

  // Matrix multiplication left and right
  for (int k = 0; k < 1000; ++k)
  {
    PlainMatrixType mat = PlainMatrixType::Random(mat_size, mat_size);

    PlainMatrixType plain(mat_size, mat_size);
    plain = tridiagonal_matrix.eigen();

    PlainMatrixType res_apply_on_the_right = tridiagonal_matrix * mat;
    PlainMatrixType res_apply_on_the_right_ref = plain * mat;
    BOOST_CHECK(res_apply_on_the_right.isApprox(res_apply_on_the_right_ref));

    PlainMatrixType res_apply_on_the_left = mat * tridiagonal_matrix;
    PlainMatrixType res_apply_on_the_left_ref = mat * plain;
    BOOST_CHECK(res_apply_on_the_left.isApprox(res_apply_on_the_left_ref));
  }
}

BOOST_AUTO_TEST_CASE(test_random)
{
  const Eigen::Index mat_size = 20;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);
  typedef TridiagonalSymmetricMatrixTpl<double>::PlainMatrixType PlainMatrixType;

  tridiagonal_matrix.setRandom();

  BOOST_CHECK(tridiagonal_matrix.rows() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.cols() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.diagonal().size() == mat_size);
  BOOST_CHECK(tridiagonal_matrix.subDiagonal().size() == mat_size - 1);

  // Fill matrix
  {
    PlainMatrixType mat(mat_size, mat_size);
    mat = tridiagonal_matrix.eigen();

    BOOST_CHECK(mat.diagonal() == tridiagonal_matrix.diagonal());
    BOOST_CHECK(mat.diagonal<-1>() == tridiagonal_matrix.subDiagonal());
    BOOST_CHECK(mat.diagonal<+1>().conjugate() == tridiagonal_matrix.subDiagonal().conjugate());
  }

  // Matrix multiplication
  for (int k = 0; k < 1000; ++k)
  {
    PlainMatrixType rhs_mat = PlainMatrixType::Random(mat_size, mat_size);

    PlainMatrixType plain(mat_size, mat_size);
    plain = tridiagonal_matrix.eigen();

    PlainMatrixType res = tridiagonal_matrix * rhs_mat;

    PlainMatrixType res_ref = plain * rhs_mat;
    BOOST_CHECK(res.isApprox(res_ref));
    BOOST_CHECK((tridiagonal_matrix * PlainMatrixType::Identity(mat_size, mat_size))
                  .isApprox(tridiagonal_matrix.matrix()));
  }
}

BOOST_AUTO_TEST_CASE(test_inverse)
{
  typedef TridiagonalSymmetricMatrixTpl<double> TridiagonalSymmetricMatrixd;
  const Eigen::Index mat_size = 10;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);
  typedef TridiagonalSymmetricMatrixTpl<double>::PlainMatrixType PlainMatrixType;

  tridiagonal_matrix.setRandom();

  PlainMatrixType plain_mat(mat_size, mat_size);
  plain_mat = tridiagonal_matrix.eigen();
  const PlainMatrixType plain_mat_inverse = plain_mat.inverse();

  const TridiagonalSymmetricMatrixInverse<TridiagonalSymmetricMatrixd> &
    tridiagonal_matrix_inverse = tridiagonal_matrix.inverse();

  BOOST_CHECK(tridiagonal_matrix_inverse.rows() == tridiagonal_matrix.rows());
  BOOST_CHECK(tridiagonal_matrix_inverse.cols() == tridiagonal_matrix.cols());

  const auto & tridiagonal_matrix_ref = tridiagonal_matrix_inverse.inverse();
  BOOST_CHECK(&tridiagonal_matrix_ref == &tridiagonal_matrix);

  const PlainMatrixType tridiagonal_matrix_inverse_plain = tridiagonal_matrix_inverse;
  BOOST_CHECK(tridiagonal_matrix_inverse_plain.isApprox(plain_mat_inverse));

  // Matrix multiplication
  for (int k = 0; k < 1; ++k)
  {
    PlainMatrixType rhs_mat = PlainMatrixType::Random(mat_size, mat_size);

    PlainMatrixType res(mat_size, mat_size);
    res = tridiagonal_matrix_ref * rhs_mat;

    BOOST_CHECK(res.isApprox((plain_mat * rhs_mat).eval()));

    res = tridiagonal_matrix_inverse * rhs_mat;
    const PlainMatrixType res_ref = plain_mat_inverse * rhs_mat;

    BOOST_CHECK(res.isApprox(res_ref));
  }

  // Test diagonal
  {
    Eigen::MatrixXd sub_diagonal_matrix = Eigen::MatrixXd::Zero(mat_size, mat_size);
    sub_diagonal_matrix.diagonal<1>().setRandom();
    sub_diagonal_matrix.diagonal().setRandom();
    sub_diagonal_matrix.diagonal<-1>().setRandom();

    const Eigen::MatrixXd test_mat = Eigen::MatrixXd::Random(mat_size, mat_size);
    const Eigen::MatrixXd res_ref = sub_diagonal_matrix * test_mat;

    Eigen::MatrixXd res(mat_size, mat_size); // res.setZero();
    res.noalias() = sub_diagonal_matrix.diagonal().asDiagonal() * test_mat;
    res.topRows(mat_size - 1) +=
      sub_diagonal_matrix.diagonal<1>().asDiagonal() * test_mat.bottomRows(mat_size - 1);
    res.bottomRows(mat_size - 1) +=
      sub_diagonal_matrix.diagonal<-1>().asDiagonal() * test_mat.topRows(mat_size - 1);
    BOOST_CHECK(res.isApprox(res_ref));
  }
}

BOOST_AUTO_TEST_CASE(test_eigenvalues)
{
  const Eigen::Index mat_size = 20;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);

  const double eps = 1e-8;

  // Case: Identity matrix
  {
    tridiagonal_matrix.setIdentity();
    const auto spectrum = computeSpectrum(tridiagonal_matrix, eps);

    BOOST_CHECK(spectrum.isOnes(eps));
  }

  // Case: Zero matrix
  {
    tridiagonal_matrix.setZero();
    const auto spectrum = computeSpectrum(tridiagonal_matrix, eps);

    BOOST_CHECK(spectrum.isZero(eps));
  }

  // Case: Random matrix
#ifndef NDEBUG
  for (int k = 0; k < 100; ++k)
#else
  for (int k = 0; k < 10000; ++k)
#endif
  {
    tridiagonal_matrix.setRandom();
    const auto spectrum = computeSpectrum(tridiagonal_matrix, eps);

    const Eigen::MatrixXd dense_matrix = tridiagonal_matrix.matrix();

    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(
      dense_matrix, Eigen::EigenvaluesOnly);
    const auto spectrum_ref = eigen_solver.eigenvalues();
    BOOST_CHECK(spectrum.isApprox(spectrum_ref, eps));

    // Compute largest and lowest eigenvalues
    const Eigen::Index last_index = tridiagonal_matrix.rows() - 1;
    const Eigen::Index first_index = 0;
    const double largest_eigenvalue = computeEigenvalue(tridiagonal_matrix, last_index, eps);
    BOOST_CHECK(math::fabs(largest_eigenvalue - spectrum_ref[last_index]) <= eps);

    const double lowest_eigenvalue = computeEigenvalue(tridiagonal_matrix, first_index, eps);
    BOOST_CHECK(math::fabs(lowest_eigenvalue - spectrum_ref[first_index]) <= eps);
  }
}
