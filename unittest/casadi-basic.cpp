//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/autodiff/casadi.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_eigen)
{
  Eigen::Matrix<casadi::SX, 3, 3> A, B;
  Eigen::Matrix<casadi::SX, 3, 1> a, b;
  Eigen::Matrix<casadi::SX, 3, 1> c = A * a - B.transpose() * b;
}

// A function working with Eigen::Matrix'es parameterized by the Scalar type
template <typename Scalar, typename T1, typename T2, typename T3, typename T4>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
eigenFun(Eigen::MatrixBase<T1> const& A,
         Eigen::MatrixBase<T2> const& a,
         Eigen::MatrixBase<T3> const& B,
         Eigen::MatrixBase<T4> const& b)
{
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> c(4);
  c.segment(1, 3) = A * a.segment(1, 3) - B.transpose() * b;
  c[0] = 0.;
  
  return c;
}

BOOST_AUTO_TEST_CASE(test_example)
{
  // Declare casadi symbolic matrix arguments
  casadi::SX cs_a = casadi::SX::sym("a", 4);
  casadi::SX cs_b = casadi::SX::sym("b", 3);
  
  // Declare Eigen matrices
  Eigen::Matrix<casadi::SX, 3, 3> A, B;
  Eigen::Matrix<casadi::SX, -1, 1> a (4), c (4);
  Eigen::Matrix<casadi::SX, 3, 1> b;
  
  // Let A, B be some numeric matrices
  for (Eigen::Index i = 0; i < A.rows(); ++i)
  {
    for (Eigen::Index j = 0; j < A.cols(); ++j)
    {
      A(i, j) = 10. * static_cast<double>(i) + static_cast<double>(j);
      B(i, j) = -10. * static_cast<double>(i) - static_cast<double>(j);
    }
  }
  
  // Let a, b be symbolic arguments of a function
  pinocchio::casadi::copy(cs_b, b);
  pinocchio::casadi::copy(cs_a, a);
  
  // Call the function taking Eigen matrices
  c = eigenFun<casadi::SX>(A, a, B, b);
  
  // Copy the result from Eigen matrices to casadi matrix
  casadi::SX cs_c = casadi::SX(casadi::Sparsity::dense(c.rows(), 1));
  pinocchio::casadi::copy(c, cs_c);
  
  // Display the resulting casadi matrix
  std::cout << "c = " << cs_c << std::endl;
  
  // Do some AD
  casadi::SX dc_da = jacobian(cs_c, cs_a);
  
  // Display the resulting jacobian
  std::cout << "dc/da = " << dc_da << std::endl;
  
  // Create a function which takes a, b and returns c and dc_da
  casadi::Function fun("fun", casadi::SXVector {cs_a, cs_b}, casadi::SXVector {cs_c, dc_da});
  std::cout << "fun = " << fun << std::endl;
  
  // Evaluate the function
  casadi::DMVector res = fun(casadi::DMVector {std::vector<double> {1., 2., 3., 4.}, std::vector<double> {-1., -2., -3.}});
  std::cout << "fun(a, b)=" << res << std::endl;
}

BOOST_AUTO_TEST_CASE(test_jacobian)
{
  casadi::SX cs_x = casadi::SX::sym("x", 3);
  
  casadi::SX cs_y = casadi::SX::sym("y", 1);
  cs_y(0) = cs_x(0) + cs_x(1) + cs_x(2);
  
  // Display the resulting expression
  std::cout << "y = " << cs_y << std::endl;
  
  // Do some AD
  casadi::SX dy_dx = jacobian(cs_x, cs_x);

  // Display the resulting jacobian
  std::cout << "dy/dx = " << dy_dx << std::endl;
}

BOOST_AUTO_TEST_CASE(test_copy_casadi_to_eigen)
{
  casadi::SX cs_mat = casadi::SX::sym("A", 3, 4);
  Eigen::Matrix<casadi::SX, 3, 4> eig_mat;

  pinocchio::casadi::copy(cs_mat, eig_mat);
  std::cout << eig_mat << std::endl;
}

BOOST_AUTO_TEST_CASE(test_copy_eigen_to_casadi)
{
  Eigen::Matrix<casadi::SX, 3, 4> eig_mat;
  pinocchio::casadi::sym(eig_mat, "A");

  casadi::SX cs_mat;

  pinocchio::casadi::copy(eig_mat, cs_mat);
  std::cout << cs_mat << std::endl;
}

BOOST_AUTO_TEST_CASE(test_casadi_codegen)
{
  casadi::SX x = casadi::SX::sym("x");
  casadi::SX y = casadi::SX::sym("y");
  casadi::Function fun("fun", casadi::SXVector {x, y}, casadi::SXVector {x + y});
  
  casadi::CodeGenerator gen("module");
  gen.add(fun);
  
  std::cout << gen.dump();
}

BOOST_AUTO_TEST_CASE(test_max)
{
  casadi::SX x = casadi::SX::sym("x");
  casadi::SX y = casadi::SX::sym("y");
  
  casadi::SX max_x_y = pinocchio::math::max(x,y);
  casadi::SX max_x_0 = pinocchio::math::max(x,0.);
  casadi::SX max_0_y = pinocchio::math::max(0.,y);
}


BOOST_AUTO_TEST_SUITE_END()
