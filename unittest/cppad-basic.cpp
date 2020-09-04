//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"
#include <cppad/speed/det_by_minor.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

  BOOST_AUTO_TEST_CASE(test_example1_cppad)
  {
    using CppAD::AD;
    using CppAD::NearEqual;
    using Eigen::Matrix;
    using Eigen::Dynamic;
    //
    typedef Matrix< AD<double> , Dynamic, 1 > eigen_vector;
    //
    // some temporary indices
    size_t i, j;
    
    // domain and range space vectors
    size_t n  = 10, m = n;
    eigen_vector a_x(n), a_y(m);
    
    // set and declare independent variables and start tape recording
    for(j = 0; j < n; j++)
    {
      a_x[(Eigen::DenseIndex)j] = double(1 + j);
    }
    CppAD::Independent(a_x);
    
    // evaluate a component wise function
    a_y = a_x.array() + a_x.array().sin();
    
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);
    
    // compute the derivative of y w.r.t x using CppAD
    CPPAD_TESTVECTOR(double) x(n);
    for(j = 0; j < n; j++)
    {
      x[j] = double(j) + 1.0 / double(j+1);
    }
    CPPAD_TESTVECTOR(double) jac = f.Jacobian(x);
      
      // check Jacobian
    double eps = 100. * CppAD::numeric_limits<double>::epsilon();
    for(i = 0; i < m; i++)
    {
      for(j = 0; j < n; j++)
      {
        double check = 1.0 + cos(x[i]);
        if( i != j ) check = 0.0;
          BOOST_CHECK(NearEqual(jac[i * n + j], check, eps, eps));
      }
    }
  }


  BOOST_AUTO_TEST_CASE(test_example2_cppad)
  {
    using CppAD::AD;
    using CppAD::NearEqual;
    using Eigen::Matrix;
    using Eigen::Dynamic;
    //
    typedef Matrix< double     , Dynamic, Dynamic > eigen_matrix;
    typedef Matrix< AD<double> , Dynamic, Dynamic > eigen_ad_matrix;
    //
    typedef Matrix< double ,     Dynamic , 1>       eigen_vector;
    typedef Matrix< AD<double> , Dynamic , 1>       eigen_ad_vector;
    // some temporary indices
    size_t i, j;
    
    // domain and range space vectors
    size_t size = 3, n  = size * size, m = 1;
    eigen_ad_vector a_x(n), a_y(m);
    eigen_vector x(n);
    
    // set and declare independent variables and start tape recording
    for(i = 0; i < size; i++)
    {
      for(j = 0; j < size; j++)
      {     // lower triangular matrix
        a_x[(Eigen::DenseIndex)(i * size + j)] = x[(Eigen::DenseIndex)(i * size + j)] = 0.0;
        if( j <= i )
          a_x[(Eigen::DenseIndex)(i * size + j)] = x[(Eigen::DenseIndex)(i * size + j)] = double(1 + i + j);
      }
    }
    CppAD::Independent(a_x);
    
    // copy independent variable vector to a matrix
    eigen_ad_matrix a_X(size, size);
    eigen_matrix X(size, size);
    for(i = 0; i < size; i++)
    {
      for(j = 0; j < size; j++)
      {
        X((Eigen::DenseIndex)i, (Eigen::DenseIndex)j)   = x[(Eigen::DenseIndex)(i * size + j)];
        // If we used a_X(i, j) = X(i, j), a_X would not depend on a_x.
        a_X((Eigen::DenseIndex)i, (Eigen::DenseIndex)j) = a_x[(Eigen::DenseIndex)(i * size + j)];
      }
    }
    
    // Compute the log of determinant of X
    a_y[0] = log( a_X.determinant() );
    
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);
    
    // check function value
    double eps = 100. * CppAD::numeric_limits<double>::epsilon();
    CppAD::det_by_minor<double> det(size);
    BOOST_CHECK(NearEqual(Value(a_y[0]) , log(det(x)), eps, eps));
    
    // compute the derivative of y w.r.t x using CppAD
    eigen_vector jac = f.Jacobian(x);
    
    // check the derivative using the formula
    // d/dX log(det(X)) = transpose( inv(X) )
    eigen_matrix inv_X = X.inverse();
    for(i = 0; i < size; i++)
    {
      for(j = 0; j < size; j++)
        BOOST_CHECK(NearEqual(jac[(Eigen::DenseIndex)(i * size + j)],
                              inv_X((Eigen::DenseIndex)j, (Eigen::DenseIndex)i),
                              eps,
                              eps));
    }
  }

  BOOST_AUTO_TEST_CASE(test_sincos)
  {
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    
    typedef AD<double> AD_double;
    
    double x0 = 1.;
    CPPAD_TESTVECTOR(AD_double) x(1), y(1), z(1);
    x[0] = x0;
    CppAD::Independent(x);
    
    y[0] = CppAD::cos(x[0]);
    BOOST_CHECK(NearEqual(y[0],std::cos(x0),eps99,eps99));
    CppAD::ADFun<double> fcos(x, y);
  
    CPPAD_TESTVECTOR(double) x_eval(1);
    x_eval[0] = x0;
    CPPAD_TESTVECTOR(double) dy(1);
    dy = fcos.Jacobian(x_eval);
    BOOST_CHECK(NearEqual(dy[0],-std::sin(x0),eps99,eps99));

    CppAD::Independent(x);
    z[0] = CppAD::sin(x[0]);
    BOOST_CHECK(NearEqual(z[0],std::sin(x0),eps99,eps99));
    
    CppAD::ADFun<double> fsin(x, z);

    CPPAD_TESTVECTOR(double) dz(1);
    dz = fsin.Jacobian(x_eval);
    BOOST_CHECK(NearEqual(dz[0],std::cos(x0),eps99,eps99));
  }

  BOOST_AUTO_TEST_CASE(test_eigen_min)
  {
    using CppAD::AD;
    
    typedef double Scalar;
    typedef AD<double> ADScalar;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(2);
    ad_Y.resize(2);

    Eigen::Vector2d x_test(-1,1);
    Eigen::Vector2d y_test = x_test.array().min(Scalar(0.));
    
    CppAD::Independent(ad_X);
    //Function
    ad_Y = ad_X.array().min(Scalar(0.));
    CppAD::ADFun<Scalar> ad_fun(ad_X,ad_Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)2);
    Eigen::Map<Eigen::Vector2d>(x.data(),2,1) = x_test;

    CPPAD_TESTVECTOR(Scalar) y = ad_fun.Forward(0,x);

    BOOST_CHECK(Eigen::Map<Eigen::Vector2d>(y.data(),2,1).isApprox(y_test));
  }

  BOOST_AUTO_TEST_CASE(test_eigen_max)
  {
    using CppAD::AD;
    
    typedef double Scalar;
    typedef AD<double> ADScalar;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(2);
    ad_Y.resize(2);

    Eigen::Vector2d x_test(-1,1);
    Eigen::Vector2d y_test = x_test.array().max(Scalar(0.));
    
    CppAD::Independent(ad_X);
    //Function
    ad_Y = ad_X.array().max(Scalar(0.));
    CppAD::ADFun<Scalar> ad_fun(ad_X,ad_Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)2);
    Eigen::Map<Eigen::Vector2d>(x.data(),2,1) = x_test;

    CPPAD_TESTVECTOR(Scalar) y = ad_fun.Forward(0,x);

    BOOST_CHECK(Eigen::Map<Eigen::Vector2d>(y.data(),2,1).isApprox(y_test));
  }


  BOOST_AUTO_TEST_CASE(test_eigen_support)
  {
    using namespace CppAD;
    
    // use a special object for source code generation
    typedef AD<double> ADScalar;
    
    typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
    
    ADVector vec_zero(ADVector::Zero(100));
    BOOST_CHECK(vec_zero.isZero());
    
    ADVector vec_ones(100);
    vec_ones.fill(1);
    BOOST_CHECK(vec_ones.isOnes());
    
  }

  BOOST_AUTO_TEST_CASE(test_abs)
  {
    CppAD::AD<double> ad_value;
    ad_value = -1.;
    abs(ad_value);
  }

BOOST_AUTO_TEST_CASE(test_atan2)
{
  CppAD::AD<double> theta,x,y;
  x = pinocchio::math::cos(theta); y = pinocchio::math::sin(theta);
  
  pinocchio::math::atan2(y,x);
  
}

BOOST_AUTO_TEST_SUITE_END()
