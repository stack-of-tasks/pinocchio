//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE orthant_cone_jordan_operation

#include "pinocchio/constraints.hpp"

#ifndef NDEBUG
  #include <iostream>
  #define PRINT_VECTOR(x)                                                                          \
    do                                                                                             \
    {                                                                                              \
      std::cout << #x << " = " << (x).transpose() << std::endl;                                    \
    } while (0);
  #define PRINT_MATRIX(M)                                                                          \
    do                                                                                             \
    {                                                                                              \
      std::cout << #M << " = \n" << (M) << std::endl;                                              \
    } while (0);
#else
  #define PRINT_VECTOR(x)                                                                          \
    do                                                                                             \
    {                                                                                              \
    } while (0)
  #define PRINT_MATRIX(M)                                                                          \
    do                                                                                             \
    {                                                                                              \
    } while (0)
#endif

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

typedef context::Scalar Scalar;
typedef NonNegativeOrthantJordanOperationTpl<Scalar, context::Options> JordanOperation;
typedef JordanOperation::VectorXs VectorXs;
typedef JordanOperation::DiagonalMatrixXs DiagonalMatrixXs;

#define CONE_DIM 5

BOOST_AUTO_TEST_CASE(is_in_cone)
{
  {
    VectorXs e = JordanOperation::GetConeIdentityElement(CONE_DIM);
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(e));
  }

  {
    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(x));
  }

  {
    VectorXs x = VectorXs::Random(CONE_DIM).cwiseAbs();
    x[0] = -Scalar(1); // x not in cone
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(x) == false);
  }
}

BOOST_AUTO_TEST_CASE(jordan_product)
{
  VectorXs e = JordanOperation::GetConeIdentityElement(CONE_DIM);

  // Test fundamental jordan properties
  {
    {
      // -- commutativity: x o y = y o x
      VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
      VectorXs y = JordanOperation::GetConeRandomElement(CONE_DIM);
      VectorXs z1(CONE_DIM);
      JordanOperation::JordanProduct(x, y, z1);
      VectorXs z2(CONE_DIM);
      JordanOperation::JordanProduct(y, x, z2);
      BOOST_CHECK(z1 == z2);
    }
    {
      // -- jordan identity: x^2 o (x o y) = x o (x^2 o y)
      VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
      VectorXs y = JordanOperation::GetConeRandomElement(CONE_DIM);
      VectorXs x2(CONE_DIM);
      JordanOperation::JordanProduct(x, x, x2);

      VectorXs z1(CONE_DIM);
      JordanOperation::JordanProduct(x, y, z1);
      JordanOperation::JordanProduct(x2, z1, z1);

      VectorXs z2(CONE_DIM);
      JordanOperation::JordanProduct(x2, y, z2);
      JordanOperation::JordanProduct(x, z2, z2);

      PRINT_VECTOR(z1);
      PRINT_VECTOR(z2);
      BOOST_CHECK(z1.isApprox(z2));
    }
  }

  // Test inverse product: x^1 o x = x o x^-1 = e
  {
    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs e_expected(CONE_DIM);
    JordanOperation::JordanInverseProduct(x, x, e_expected);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));

    VectorXs xinv(CONE_DIM);
    JordanOperation::JordanInverseProduct(x, e, xinv);
    JordanOperation::JordanProduct(xinv, x, e_expected);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));

    JordanOperation::JordanProduct(x, xinv, e_expected);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));
  }

  // Test property: x^-1 o (x o y ) = y
  {
    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs y = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs xinv(CONE_DIM);
    JordanOperation::JordanInverseProduct(x, e, xinv);
    VectorXs xy(CONE_DIM);
    JordanOperation::JordanProduct(x, y, xy);

    VectorXs y_expected(CONE_DIM);
    JordanOperation::JordanProduct(xinv, xy, y_expected);
    PRINT_VECTOR(y_expected);
    PRINT_VECTOR(y);
    BOOST_CHECK(y_expected.isApprox(y));
  }
}

BOOST_AUTO_TEST_CASE(jordan_quadratic_form)
{
  VectorXs e = JordanOperation::GetConeIdentityElement(CONE_DIM);

  {
    // We test the property: P(x^1/2)e = x
    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs x_expected(CONE_DIM);
    JordanOperation::ApplyQuadraticForm(x, e, x_expected);
    PRINT_VECTOR(x);
    PRINT_VECTOR(x_expected);
    BOOST_CHECK(x_expected.isApprox(x));
  }

  {
    // We test the property: P(x^-1/2)x = e
    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs e_expected(CONE_DIM);
    JordanOperation::ApplyInverseQuadraticForm(x, x, e_expected);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));
  }
}

BOOST_AUTO_TEST_CASE(jordan_scaling)
{
  VectorXs s = JordanOperation::GetConeRandomElement(CONE_DIM);
  VectorXs z = JordanOperation::GetConeRandomElement(CONE_DIM);

  VectorXs lambda(CONE_DIM);
  VectorXs w(CONE_DIM);

  JordanOperation::ComputeScaling(s, z, lambda, w);

  VectorXs lambda_expected1(CONE_DIM);
  JordanOperation::ApplyInverseScaling(w, s, lambda_expected1); // W^-1 s
  VectorXs lambda_expected2(CONE_DIM);
  JordanOperation::ApplyScaling(w, z, lambda_expected2); // W z
  PRINT_VECTOR(lambda);
  PRINT_VECTOR(lambda_expected1);
  PRINT_VECTOR(lambda_expected2);
  BOOST_CHECK(lambda_expected1.isApprox(lambda));
  BOOST_CHECK(lambda_expected2.isApprox(lambda));
}

BOOST_AUTO_TEST_CASE(jordan_scaling_lambda)
{
  VectorXs s = JordanOperation::GetConeRandomElement(CONE_DIM);
  VectorXs z = JordanOperation::GetConeRandomElement(CONE_DIM);

  VectorXs lambda1(CONE_DIM);
  VectorXs lambda2(CONE_DIM);
  VectorXs w(CONE_DIM);
  VectorXs winv(CONE_DIM);

  // Test property:
  // lambda is the same, wether scaling is U or W (U = W^-1)
  JordanOperation::ComputeScaling(s, z, lambda1, w);
  JordanOperation::ComputeScaling(z, s, lambda2, winv);

  PRINT_VECTOR(lambda1);
  PRINT_VECTOR(lambda2);
  BOOST_CHECK(lambda1.isApprox(lambda2));
}

BOOST_AUTO_TEST_CASE(jordan_scaling_matrix)
{
  VectorXs s = JordanOperation::GetConeRandomElement(CONE_DIM);
  VectorXs z = JordanOperation::GetConeRandomElement(CONE_DIM);

  VectorXs lambda(CONE_DIM);
  VectorXs w(CONE_DIM);
  DiagonalMatrixXs W, Winv;

  JordanOperation::ComputeScaling(s, z, lambda, w);
  JordanOperation::RetrieveScalingMatrix(w, W);
  JordanOperation::RetrieveInverseScalingMatrix(w, Winv);

  {
    // Test property:
    // W x = ApplyScaling(w, x)
    // W^-1 x = ApplyInverseScaling(w, x)

    VectorXs x = JordanOperation::GetConeRandomElement(CONE_DIM);
    VectorXs wx(CONE_DIM);
    JordanOperation::ApplyScaling(w, x, wx);
    BOOST_CHECK(wx.isApprox(W * x));

    VectorXs winvx(CONE_DIM);
    JordanOperation::ApplyInverseScaling(w, x, winvx);
    BOOST_CHECK(winvx.isApprox(Winv * x));
  }

  {
    // Test property:
    // W^-1 = Winv
    DiagonalMatrixXs W, Winv;
    JordanOperation::RetrieveScalingMatrix(w, W);
    JordanOperation::RetrieveInverseScalingMatrix(w, Winv);
    PRINT_MATRIX(W.toDenseMatrix());
    PRINT_MATRIX(Winv.toDenseMatrix());
    PRINT_MATRIX(W.inverse().toDenseMatrix());
    BOOST_CHECK((W.inverse().diagonal()).isApprox(Winv.diagonal()));
  }

  {
    // Test property: U = W^-1
    VectorXs lambda1(CONE_DIM);
    VectorXs lambda2(CONE_DIM);
    VectorXs w(CONE_DIM);
    VectorXs winv(CONE_DIM);
    JordanOperation::ComputeScaling(s, z, lambda1, w);
    JordanOperation::ComputeScaling(z, s, lambda2, winv);

    DiagonalMatrixXs U, Winv;
    JordanOperation::RetrieveScalingMatrix(winv, U);
    JordanOperation::RetrieveInverseScalingMatrix(w, Winv);
    PRINT_MATRIX(U.toDenseMatrix());
    PRINT_MATRIX(Winv.toDenseMatrix());
    BOOST_CHECK((U.diagonal()).isApprox(Winv.diagonal()));
  }

  {
    // Test property: W^2 = getScalingMatrixSquared(w, W2)
    JordanOperation::ComputeScaling(s, z, lambda, w);
    JordanOperation::RetrieveScalingMatrix(w, W);
    DiagonalMatrixXs W2;
    JordanOperation::RetrieveSquaredScalingMatrix(w, W2);
    DiagonalMatrixXs W2_expected;
    W2_expected.diagonal() = W.diagonal().array().square();
    PRINT_MATRIX(W2_expected.toDenseMatrix());
    PRINT_MATRIX(W2.toDenseMatrix());
    BOOST_CHECK((W2_expected.diagonal()).isApprox(W2.diagonal()));
  }

  {
    // Test property: W^2 = getScalingMatrixSquared(w, W2)
    // but with a vector to represent the diagonal matrix
    JordanOperation::ComputeScaling(s, z, lambda, w);

    VectorXs wdiag_mat(w.size());
    JordanOperation::RetrieveScalingMatrixDiagonal(w, wdiag_mat);
    PRINT_VECTOR(w);
    PRINT_VECTOR(wdiag_mat);
    BOOST_CHECK((wdiag_mat).isApprox(w));

    VectorXs w2diag_mat(w.size());
    JordanOperation::RetrieveSquaredScalingMatrixDiagonal(w, w2diag_mat);
    VectorXs w2_diag_mat_expected;
    w2_diag_mat_expected.array() = w.array().square();
    PRINT_VECTOR(w2_diag_mat_expected);
    PRINT_VECTOR(w2diag_mat);
    BOOST_CHECK((w2diag_mat).isApprox(w2_diag_mat_expected));
  }
}
