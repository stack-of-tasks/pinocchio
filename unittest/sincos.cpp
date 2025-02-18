//
// Copyright (c) 2018-2019 INRIA
//

#include "pinocchio/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include <cstdlib>

#include "utils/macros.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace
{
  template<typename Scalar>
  Scalar sinCosTolerance();

  template<>
  inline float sinCosTolerance<float>()
  {
    // I don't know how tolerance has been calculated.
    // 1e-7 doesn't work on ARM + Debug architecture.
    return 1.5e-7f;
  }

  template<>
  inline double sinCosTolerance<double>()
  {
    return 1e-15;
  }

  template<>
  inline long double sinCosTolerance<long double>()
  {
    return 1e-19;
  }
} // namespace

template<typename Scalar>
void testSINCOS(int n)
{
  for (int k = 0; k < n; ++k)
  {
    Scalar sin_value, cos_value;
    Scalar alpha = (Scalar)std::rand() / (Scalar)RAND_MAX;
    pinocchio::SINCOS(alpha, &sin_value, &cos_value);

    Scalar sin_value_ref = std::sin(alpha), cos_value_ref = std::cos(alpha);

    BOOST_CHECK_CLOSE_FRACTION(sin_value, sin_value_ref, sinCosTolerance<Scalar>());
    BOOST_CHECK_CLOSE_FRACTION(cos_value, cos_value_ref, sinCosTolerance<Scalar>());
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
