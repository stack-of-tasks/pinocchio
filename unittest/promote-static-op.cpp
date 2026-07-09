//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE promote_static_op

#include "pinocchio/utils/promote-static-eval.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_helpers)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(2, 2);

  BOOST_CHECK(!internal::helper::is_eigen_noalias_v<decltype(A)>);
  BOOST_CHECK(internal::helper::is_eigen_noalias_v<decltype(A.noalias())>);

  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3d;
  BOOST_CHECK(internal::helper::has_fixed_rows_v<Matrix3d>);
  BOOST_CHECK(!internal::helper::has_fixed_cols_v<Matrix3d>);
  BOOST_CHECK(!internal::helper::has_fixed_size_v<Matrix3d>);

  typedef Eigen::Matrix<double, Eigen::Dynamic, 3> Matrixd3;
  BOOST_CHECK(!internal::helper::has_fixed_rows_v<Matrixd3>);
  BOOST_CHECK(internal::helper::has_fixed_cols_v<Matrixd3>);
  BOOST_CHECK(!internal::helper::has_fixed_size_v<Matrixd3>);

  typedef Eigen::Matrix<double, 3, 3> Matrix33;
  BOOST_CHECK(internal::helper::has_fixed_rows_v<Matrix33>);
  BOOST_CHECK(internal::helper::has_fixed_cols_v<Matrix33>);
  BOOST_CHECK(internal::helper::has_fixed_size_v<Matrix33>);
}

BOOST_AUTO_TEST_CASE(test_size_product)
{
  {
    typedef Eigen::MatrixXd ResType;
    typedef Eigen::Matrix3d LhsType;
    typedef Eigen::Matrix3d RhsType;
    BOOST_CHECK(
      (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::is_static_size_product()));
    BOOST_CHECK((internal::MatrixProductDimensions<
                 ResType, LhsType, RhsType>::is_partial_static_size_product()));
  }

  {
    typedef Eigen::MatrixXd ResType;
    typedef Eigen::MatrixXd LhsType;
    typedef Eigen::MatrixXd RhsType;
    BOOST_CHECK(
      (!internal::MatrixProductDimensions<ResType, LhsType, RhsType>::is_static_size_product()));
    BOOST_CHECK((!internal::MatrixProductDimensions<
                 ResType, LhsType, RhsType>::is_partial_static_size_product()));
  }

  {
    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> ResType;
    typedef Eigen::Matrix3d LhsType;
    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> RhsType;
    BOOST_CHECK(
      (!internal::MatrixProductDimensions<ResType, LhsType, RhsType>::is_static_size_product()));
    BOOST_CHECK((internal::MatrixProductDimensions<
                 ResType, LhsType, RhsType>::is_partial_static_size_product()));
  }
}

BOOST_AUTO_TEST_CASE(test_dynamic_matrix)
{
  const Eigen::Index n = 10, m = 5;
  typedef Eigen::MatrixXd Matrix;
  Eigen::MatrixXd A = Eigen::MatrixXd::Constant(n, n, 1);
  Eigen::MatrixXd B = Eigen::MatrixXd::Constant(n, m, 2);
  Eigen::MatrixXd C = Eigen::MatrixXd::Random(n, m);

  const auto C_expression = A * B;

  BOOST_CHECK(C_expression.rows() == A.rows());
  BOOST_CHECK(C_expression.cols() == B.cols());

  auto C_op = internal::promote_static_eval<10>(C);
  BOOST_CHECK(&C_op.expression() == &C);

  BOOST_CHECK(
    (!internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::is_static_size_product()));
  BOOST_CHECK(
    (!internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::dynamic_size(A, B)) == B.cols());

  C_op = A * B;
  const auto res_aliasing = C_expression.eval();
  BOOST_CHECK(C == res_aliasing);

  // Test with noalias
  A.setConstant(3);
  B.setConstant(4);

  auto C_noalias_op = internal::promote_static_eval<10>(C.noalias());
  BOOST_CHECK(&C_noalias_op.expression().expression() == &C);

  BOOST_CHECK(
    (!internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::is_static_size_product()));
  BOOST_CHECK(
    (!internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<Matrix, Matrix, Matrix>::dynamic_size(A, B)) == B.cols());

  C_noalias_op = A * B;
  const auto res_noaliasing = C_expression.eval();
  BOOST_CHECK(res_noaliasing != res_aliasing);
  BOOST_CHECK(C == res_noaliasing);
}

BOOST_AUTO_TEST_CASE(test_static_matrix)
{
  constexpr Eigen::Index Rows = 10, Cols = 5, InnerDim = 8;
  typedef Eigen::Matrix<double, Rows, InnerDim> LhsType;
  typedef Eigen::Matrix<double, InnerDim, Cols> RhsType;
  typedef Eigen::Matrix<double, Rows, Cols> ResType;

  LhsType A = LhsType::Constant(1);
  RhsType B = RhsType::Constant(2);
  ResType C = ResType::Random();

  const auto C_expression = A * B;

  BOOST_CHECK(C_expression.rows() == A.rows());
  BOOST_CHECK(C_expression.cols() == B.cols());

  auto C_op = internal::promote_static_eval<10>(C);
  BOOST_CHECK(&C_op.expression() == &C);

  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::is_static_size_product()));
  BOOST_CHECK((internal::MatrixProductDimensions<
               ResType, LhsType, RhsType>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::dynamic_size(A, B)) == -1);

  C_op = A * B;
  const auto res_aliasing = C_expression.eval();
  BOOST_CHECK(C == res_aliasing);

  // Test with noalias
  A.setConstant(3);
  B.setConstant(4);

  auto C_noalias_op = internal::promote_static_eval<10>(C.noalias());
  BOOST_CHECK(&C_noalias_op.expression().expression() == &C);

  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::is_static_size_product()));
  BOOST_CHECK((internal::MatrixProductDimensions<
               ResType, LhsType, RhsType>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::dynamic_size(A, B)) == -1);

  C_noalias_op = A * B;
  const auto res_noaliasing = C_expression.eval();
  BOOST_CHECK(res_noaliasing != res_aliasing);
  BOOST_CHECK(C == res_noaliasing);
}

BOOST_AUTO_TEST_CASE(test_partial_static_matrix)
{
  constexpr Eigen::Index Rows = 3, Cols = Eigen::Dynamic, InnerDim = Rows;
  const Eigen::Index n = Rows, m = 10;
  typedef Eigen::Matrix<double, Rows, InnerDim> LhsType;
  typedef Eigen::Matrix<double, InnerDim, Cols> RhsType;
  typedef Eigen::Matrix<double, Rows, Cols> ResType;

  LhsType A = LhsType::Constant(n, n, 1);
  RhsType B = RhsType::Constant(n, m, 2);
  ResType C = ResType::Random(n, m);

  const auto C_expression = A * B;

  BOOST_CHECK(C_expression.rows() == A.rows());
  BOOST_CHECK(C_expression.cols() == B.cols());

  auto C_op = internal::promote_static_eval<10>(C);
  BOOST_CHECK(&C_op.expression() == &C);

  BOOST_CHECK(
    (!internal::MatrixProductDimensions<ResType, LhsType, ResType>::is_static_size_product()));
  BOOST_CHECK((internal::MatrixProductDimensions<
               ResType, LhsType, ResType>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::dynamic_size(A, B)) == m);

  C_op = A * B;
  const auto res_aliasing = C_expression.eval();
  BOOST_CHECK(C == res_aliasing);

  // Test with noalias
  A.setConstant(3);
  B.setConstant(4);

  auto C_noalias_op = internal::promote_static_eval<10>(C.noalias());
  BOOST_CHECK(&C_noalias_op.expression().expression() == &C);

  BOOST_CHECK(
    (!internal::MatrixProductDimensions<ResType, LhsType, ResType>::is_static_size_product()));
  BOOST_CHECK((internal::MatrixProductDimensions<
               ResType, LhsType, ResType>::is_partial_static_size_product()));
  BOOST_CHECK(
    (internal::MatrixProductDimensions<ResType, LhsType, RhsType>::dynamic_size(A, B)) == m);

  C_noalias_op = A * B;
  const auto res_noaliasing = C_expression.eval();
  BOOST_CHECK(res_noaliasing != res_aliasing);
  BOOST_CHECK(C == res_noaliasing);
}
