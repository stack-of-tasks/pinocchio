//
// Copyright (c) 2020 INRIA
//

#include <pinocchio/math/matrix.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_isNormalized)
{
  srand(0);

  using namespace pinocchio;
  typedef Eigen::Matrix<double,Eigen::Dynamic,1> Vector;
  
  const int max_size = 1000;
  for(int i = 0; i < 1e6; ++i)
  {
    const Eigen::DenseIndex size = rand() % max_size + 1; // random vector size
    Vector vec;
    vec = Vector::Random(size) + Vector::Constant(size,2.);
    BOOST_CHECK(!isNormalized(vec));
    
    vec.normalize();
    BOOST_CHECK(isNormalized(vec));
    
    // Specific check for the Zero vector
    BOOST_CHECK(!isNormalized(Vector(Vector::Zero(size))));
  }
}

BOOST_AUTO_TEST_SUITE_END()
