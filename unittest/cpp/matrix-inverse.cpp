//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE matrix_inverse

#include <iostream>

#include <pinocchio/math.hpp>
#include <Eigen/LU>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

template<int size>
using MatrixTpl = Eigen::Matrix<double, size, size>;

using DynamicMatrix = MatrixTpl<Eigen::Dynamic>;

#ifndef NDEBUG
const int N = int(1e3);
#else
const int N = int(1e4);
#endif

template<int size>
void test_generated_inverse_impl()
{
  std::cout << "size: " << size << std::endl;
  typedef MatrixTpl<size> Matrix;
  for (int i = 0; i < N; ++i)
  {
    Matrix mat = Matrix::Random();
    mat = mat.transpose() * mat + 1e-8 * Matrix::Identity();
    make_symmetric(mat);
    BOOST_CHECK(is_symmetric(mat, 0));

    if (!(mat.determinant() > 1e-3))
    {
      i--;
      continue;
    }

    Matrix res = Matrix::Zero();
    internal::matrix_inversion_code_generated(mat, res);
    BOOST_CHECK((res * mat).isIdentity(1e-14 * res.norm()));

    BOOST_CHECK(mat.inverse().isApprox(res, 1e-14 * res.norm()));
  }
}

BOOST_AUTO_TEST_CASE(test_generated_inverse)
{
  test_generated_inverse_impl<1>();
  test_generated_inverse_impl<2>();
  test_generated_inverse_impl<3>();
  test_generated_inverse_impl<4>();
  test_generated_inverse_impl<5>();
  test_generated_inverse_impl<6>();
  test_generated_inverse_impl<7>();
  test_generated_inverse_impl<8>();
  test_generated_inverse_impl<9>();
  test_generated_inverse_impl<10>();
  test_generated_inverse_impl<11>();
  test_generated_inverse_impl<12>();
}

template<int size>
void test_matrix_inverse_on_dynamic_matrix_impl()
{
  typedef DynamicMatrix Matrix;
  for (int i = 0; i < N; ++i)
  {
    Matrix mat = Matrix::Random(size, size);
    mat = mat.transpose() * mat + 1e-8 * Matrix::Identity(size, size);
    make_symmetric(mat);
    BOOST_CHECK(is_symmetric(mat, 0));

    if (!(mat.determinant() > 1e-3))
    {
      i--;
      continue;
    }

    Matrix res = Matrix::Zero(size, size);
    internal::matrix_inversion(mat, res);

    const double max_mat_value = mat.array().abs().maxCoeff();
    const double max_res_value = res.array().abs().maxCoeff();
    const double max_value = std::max(max_mat_value, max_res_value);
    const double check_tol = max_value * 1e-12;

    BOOST_CHECK((res * mat).isIdentity(check_tol));
    BOOST_CHECK(mat.inverse().isApprox(res, check_tol));

    // Direct call to the method
    Matrix res2 = Matrix::Zero(size, size);
    pinocchio::internal::MatrixInversionDynamicMatrixImpl::run(mat, res2);
    BOOST_CHECK(res2 == res);
  }
}

BOOST_AUTO_TEST_CASE(test_matrix_inverse_on_dynamic_matrix)
{
  test_matrix_inverse_on_dynamic_matrix_impl<1>();
  test_matrix_inverse_on_dynamic_matrix_impl<2>();
  test_matrix_inverse_on_dynamic_matrix_impl<3>();
  test_matrix_inverse_on_dynamic_matrix_impl<4>();
  test_matrix_inverse_on_dynamic_matrix_impl<5>();
  test_matrix_inverse_on_dynamic_matrix_impl<6>();
  test_matrix_inverse_on_dynamic_matrix_impl<7>();
  test_matrix_inverse_on_dynamic_matrix_impl<8>();
  test_matrix_inverse_on_dynamic_matrix_impl<9>();
  test_matrix_inverse_on_dynamic_matrix_impl<10>();
  test_matrix_inverse_on_dynamic_matrix_impl<11>();
  test_matrix_inverse_on_dynamic_matrix_impl<12>();
}
