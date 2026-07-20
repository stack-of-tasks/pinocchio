//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE matrix_product

#include <pinocchio/math.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

#ifndef NDEBUG
const int N = int(1e3);
#else
const int N = int(1e4);
#endif

template<typename Matrix, template<typename, typename> class EigenOp>
void test(
  const Eigen::Index rows = 10, const Eigen::Index cols = 20, const Eigen::Index inner_dim = 30)
{
  using Scalar = typename Matrix::Scalar;

  for (int i = 0; i < N; ++i)
  {
    const Matrix lhs = Matrix::Random(rows, inner_dim), rhs = Matrix::Random(inner_dim, cols);
    Matrix res = Matrix::Random(rows, cols);

    Matrix res_gt = Matrix::Zero(rows, cols);
    typedef EigenOp<Scalar, Scalar> Op;

    if constexpr (internal::is_specialization_of_v<Op, Eigen::internal::assign_op>)
      res_gt = (lhs * rhs).eval();
    else if constexpr (internal::is_specialization_of_v<Op, Eigen::internal::add_assign_op>)
      res_gt = res + (lhs * rhs).eval();
    else if constexpr (internal::is_specialization_of_v<Op, Eigen::internal::sub_assign_op>)
      res_gt = res - (lhs * rhs).eval();

    internal::matrix_product<EigenOp>(lhs, rhs, res);

    BOOST_CHECK(res.isApprox(res_gt, 1e-14));
  }
}

BOOST_AUTO_TEST_CASE(test_col_major)
{
  typedef Eigen::MatrixXd Matrix;

  const Eigen::Index rows = 10, cols = 20, inner_dim = 30;
  test<Matrix, Eigen::internal::assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::add_assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::sub_assign_op>(rows, cols, inner_dim);
}

BOOST_AUTO_TEST_CASE(test_row_major)
{
  typedef PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Eigen::MatrixXd) Matrix;

  const Eigen::Index rows = 10, cols = 20, inner_dim = 30;
  test<Matrix, Eigen::internal::assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::add_assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::sub_assign_op>(rows, cols, inner_dim);
}

BOOST_AUTO_TEST_CASE(test_col_major_small)
{
  typedef Eigen::MatrixXd Matrix;

  const Eigen::Index rows = 4, cols = 4, inner_dim = 4;
  test<Matrix, Eigen::internal::assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::add_assign_op>(rows, cols, inner_dim);
  test<Matrix, Eigen::internal::sub_assign_op>(rows, cols, inner_dim);
}
