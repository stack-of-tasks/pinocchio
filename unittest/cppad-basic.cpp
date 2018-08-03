//
// Copyright (c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/fwd.hpp"
#include <boost/variant.hpp> // to avoid C99 warnings

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/speed/det_by_minor.hpp>
#include <Eigen/Dense>

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
    typedef Matrix< AD<double> , Dynamic, 1 > a_vector;
    //
    // some temporary indices
    size_t i, j;
    
    // domain and range space vectors
    size_t n  = 10, m = n;
    a_vector a_x(n), a_y(m);
    
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
  
BOOST_AUTO_TEST_SUITE_END()
