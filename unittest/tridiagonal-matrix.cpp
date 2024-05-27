//
// Copyright (c) 2024 INRIA
//

#include <iostream>

#include <pinocchio/math/tridiagonal-matrix.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_zero)
{
  const Eigen::DenseIndex mat_size = 20;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);

  tridiagonal_matrix.setZero();
  BOOST_CHECK(tridiagonal_matrix.isZero(0));
  BOOST_CHECK(tridiagonal_matrix.isDiagonal(0));
  BOOST_CHECK(tridiagonal_matrix.matrix().isZero(0));
}

BOOST_AUTO_TEST_CASE(test_identity)
{
  const Eigen::DenseIndex mat_size = 20;
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
    mat = tridiagonal_matrix;

    BOOST_CHECK(mat.isIdentity(0));
  }

  // Matrix multiplication left and right
  for (int k = 0; k < 1000; ++k)
  {
    PlainMatrixType mat = PlainMatrixType::Random(mat_size, mat_size);

    PlainMatrixType plain(mat_size, mat_size);
    plain = tridiagonal_matrix;

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
  const Eigen::DenseIndex mat_size = 20;
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
    mat = tridiagonal_matrix;

    BOOST_CHECK(mat.diagonal() == tridiagonal_matrix.diagonal());
    BOOST_CHECK(mat.diagonal<-1>() == tridiagonal_matrix.subDiagonal());
    BOOST_CHECK(mat.diagonal<+1>().conjugate() == tridiagonal_matrix.subDiagonal().conjugate());
  }

  // Matrix multiplication
  for (int k = 0; k < 1000; ++k)
  {
    PlainMatrixType rhs_mat = PlainMatrixType::Random(mat_size, mat_size);

    PlainMatrixType plain(mat_size, mat_size);
    plain = tridiagonal_matrix;

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
  const Eigen::DenseIndex mat_size = 10;
  TridiagonalSymmetricMatrixTpl<double> tridiagonal_matrix(mat_size);
  typedef TridiagonalSymmetricMatrixTpl<double>::PlainMatrixType PlainMatrixType;

  tridiagonal_matrix.setRandom();

  PlainMatrixType plain_mat(mat_size, mat_size);
  plain_mat = tridiagonal_matrix;
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

BOOST_AUTO_TEST_SUITE_END()
