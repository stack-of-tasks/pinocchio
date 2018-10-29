//
// Copyright (c) 2018 INRIA
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

#include "utils/macros.hpp"
#include "pinocchio/math/sincos.hpp"
#include <cstdlib>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template<typename Scalar>
void testSINCOS(int n)
{
  for(int k = 0; k < n; ++k)
  {
    Scalar sin_value, cos_value;
    Scalar alpha = (Scalar)std::rand()/(Scalar)RAND_MAX;
    se3::SINCOS(alpha,&sin_value,&cos_value);
    
    Scalar sin_value_ref = std::sin(alpha),
           cos_value_ref = std::cos(alpha);
    
    BOOST_CHECK(sin_value == sin_value_ref);
    BOOST_CHECK(cos_value == cos_value_ref);
  }
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_sincos)
{
#ifndef NDEBUG
  const int n = 1e3;
#else
  const int n = 1e6;
#endif
  testSINCOS<float>(n);
  testSINCOS<double>(n);
  testSINCOS<long double>(n);
}

BOOST_AUTO_TEST_SUITE_END()
