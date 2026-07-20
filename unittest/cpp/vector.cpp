//
// Copyright (c) 2020 INRIA
//

#define BOOST_TEST_MODULE vector

#include <pinocchio/math.hpp>

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_isNormalized)
{
  srand(0);

  using namespace pinocchio;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;

  const int max_size = 1000;
#ifdef NDEBUG
  const int max_test = 1e6;
#else
  const int max_test = 1e2;
#endif
  for (int i = 0; i < max_test; ++i)
  {
    const Eigen::Index size = rand() % max_size + 1; // random vector size
    Vector vec;
    vec = Vector::Random(size) + Vector::Constant(size, 2.);
    BOOST_CHECK(!isNormalized(vec));

    vec.normalize();
    BOOST_CHECK(isNormalized(vec));

    // Specific check for the Zero vector
    BOOST_CHECK(!isNormalized(Vector(Vector::Zero(size))));
  }
}
