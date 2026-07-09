//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE second_order_cone_jordan_operation

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
typedef SecondOrderConeJordanOperationTpl<Scalar, context::Options> JordanOperation;
typedef JordanOperation::Vector3s Vector3s;
typedef JordanOperation::Vector4s Vector4s;
typedef JordanOperation::Matrix3s Matrix3s;

BOOST_AUTO_TEST_CASE(is_in_cone)
{
  {
    Vector3s e = JordanOperation::GetConeIdentityElement();
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(e));
  }

  {
    Vector3s x = JordanOperation::GetConeRandomElement();
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(x));
  }

  {
    Vector3s x = Vector3s::Random();
    x[2] = x.template head<2>().norm() - 1e-8; // x not in cone
    BOOST_CHECK(JordanOperation::IsInSymmetricCone(x) == false);
  }
}

BOOST_AUTO_TEST_CASE(jordan_product)
{
  Vector3s e = JordanOperation::GetConeIdentityElement();

  // Test fundamental jordan properties
  {
    {
      // -- commutativity: x o y = y o x
      Vector3s x = JordanOperation::GetConeRandomElement();
      Vector3s y = JordanOperation::GetConeRandomElement();
      Vector3s z1 = JordanOperation::JordanProduct(x, y);
      Vector3s z2 = JordanOperation::JordanProduct(y, x);
      BOOST_CHECK(z1.isApprox(z2));
    }
    {
      // -- jordan identity: x^2 o (x o y) = x o (x^2 o y)
      Vector3s x = JordanOperation::GetConeRandomElement();
      Vector3s y = JordanOperation::GetConeRandomElement();
      Vector3s x2 = JordanOperation::JordanProduct(x, x);

      Vector3s z1 = JordanOperation::JordanProduct(x, y);
      z1 = JordanOperation::JordanProduct(x2, z1);

      Vector3s z2 = JordanOperation::JordanProduct(x2, y);
      z2 = JordanOperation::JordanProduct(x, z2);

      PRINT_VECTOR(z1);
      PRINT_VECTOR(z2);
      BOOST_CHECK(z1.isApprox(z2));
    }
  }

  // Test inverse product: x^1 o x = x o x^-1 = e
  {
    Vector3s x = JordanOperation::GetConeRandomElement();
    Vector3s e_expected = JordanOperation::JordanInverseProduct(x, x);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));

    Vector3s xinv = JordanOperation::JordanInverseProduct(x, e);
    e_expected = JordanOperation::JordanProduct(xinv, x);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));

    e_expected = JordanOperation::JordanProduct(x, xinv);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));
  }

  // Test property: x^-1 o (x o y) = y
  {
    Vector3s x = JordanOperation::GetConeRandomElement();
    Vector3s y = JordanOperation::GetConeRandomElement();
    Vector3s xy = JordanOperation::JordanProduct(x, y);

    Vector3s y_expected = JordanOperation::JordanInverseProduct(x, xy);
    PRINT_VECTOR(y_expected);
    PRINT_VECTOR(y);
    BOOST_CHECK(y_expected.isApprox(y));
  }
}

BOOST_AUTO_TEST_CASE(jordan_quadratic_form)
{
  Vector3s e = JordanOperation::GetConeIdentityElement();

  {
    // We test the property: P(x^1/2)e = x
    Vector3s x = JordanOperation::GetConeRandomElement();
    Vector3s x_expected = JordanOperation::ApplyQuadraticForm(x, e);
    PRINT_VECTOR(x);
    PRINT_VECTOR(x_expected);
    BOOST_CHECK(x_expected.isApprox(x));
  }

  {
    // We test the property: P(x^-1/2)x = e
    Vector3s x = JordanOperation::GetConeRandomElement();
    Vector3s e_expected = JordanOperation::ApplyInverseQuadraticForm(x, x);
    PRINT_VECTOR(e_expected);
    BOOST_CHECK(e_expected.isApprox(e));
  }
}

BOOST_AUTO_TEST_CASE(jordan_scaling)
{
  Vector3s s = JordanOperation::GetConeRandomElement();
  Vector3s z = JordanOperation::GetConeRandomElement();

  Vector3s lambda;
  Vector4s w;

  JordanOperation::ComputeScaling(s, z, lambda, w);

  Vector3s lambda_expected1 = JordanOperation::ApplyInverseScaling(w, s); // W^-1 s
  Vector3s lambda_expected2 = JordanOperation::ApplyScaling(w, z);        // W z
  PRINT_VECTOR(lambda);
  PRINT_VECTOR(lambda_expected1);
  PRINT_VECTOR(lambda_expected2);
  BOOST_CHECK(lambda_expected1.isApprox(lambda));
  BOOST_CHECK(lambda_expected2.isApprox(lambda));
}

BOOST_AUTO_TEST_CASE(jordan_scaling_lambda)
{
  Vector3s s = JordanOperation::GetConeRandomElement();
  Vector3s z = JordanOperation::GetConeRandomElement();

  Vector3s lambda1, lambda2;
  Vector4s w, winv;

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
  Vector3s s = JordanOperation::GetConeRandomElement();
  Vector3s z = JordanOperation::GetConeRandomElement();

  Vector3s lambda;
  Vector4s w;
  Matrix3s W, Winv;

  JordanOperation::ComputeScaling(s, z, lambda, w);
  JordanOperation::RetrieveScalingMatrix(w, W);
  JordanOperation::RetrieveInverseScalingMatrix(w, Winv);

  {
    // Test property:
    // W x = ApplyScaling(w, x)
    // W^-1 x = ApplyInverseScaling(w, x)

    Vector3s x = JordanOperation::GetConeRandomElement();
    BOOST_CHECK(JordanOperation::ApplyScaling(w, x).isApprox(W * x));
    BOOST_CHECK(JordanOperation::ApplyInverseScaling(w, x).isApprox(Winv * x));
  }

  {
    // Test property:
    // W^-1 = Winv
    Matrix3s W, Winv;
    JordanOperation::RetrieveScalingMatrix(w, W);
    JordanOperation::RetrieveInverseScalingMatrix(w, Winv);
    PRINT_MATRIX(W);
    PRINT_MATRIX(Winv);
    PRINT_MATRIX(W.inverse());
    BOOST_CHECK(W.inverse().isApprox(Winv));
  }

  {
    // Test property: U = W^-1
    Vector3s lambda1, lambda2;
    Vector4s w, winv;
    JordanOperation::ComputeScaling(s, z, lambda1, w);
    JordanOperation::ComputeScaling(z, s, lambda2, winv);

    Matrix3s U, Winv;
    JordanOperation::RetrieveScalingMatrix(winv, U);
    JordanOperation::RetrieveInverseScalingMatrix(w, Winv);
    PRINT_MATRIX(U);
    PRINT_MATRIX(Winv);
    BOOST_CHECK(U.isApprox(Winv));
  }

  {
    // Test property: W^2 = getScalingMatrixSquared(w, W2)
    JordanOperation::ComputeScaling(s, z, lambda, w);
    JordanOperation::RetrieveScalingMatrix(w, W);
    Matrix3s W2;
    JordanOperation::RetrieveSquaredScalingMatrix(w, W2);
    Matrix3s W2_expected = W * W;
    PRINT_MATRIX(W2_expected);
    PRINT_MATRIX(W2);
    BOOST_CHECK(W2_expected.isApprox(W2));
  }
}
