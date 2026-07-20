//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE zero_cone

#include "pinocchio/constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_proj)
{
  const int num_tests = int(1000);
  const int dim = 10;

  const ZeroCone set;

  BOOST_CHECK(set.isInside(ZeroCone::Vector::Zero(dim)));
  BOOST_CHECK(!set.isInside(ZeroCone::Vector::Ones(dim)));
  BOOST_CHECK(set.project(ZeroCone::Vector::Zero(dim)) == ZeroCone::Vector::Zero(dim));
  BOOST_CHECK(set.project(ZeroCone::Vector::Ones(dim)) == ZeroCone::Vector::Zero(dim));

  for (int k = 0; k < num_tests; ++k)
  {
    const ZeroCone::Vector x = ZeroCone::Vector::Random(dim);
    if (!x.isZero())
      BOOST_CHECK(!set.isInside(x));

    const auto proj_x = set.project(x);
    const auto proj_proj_x = set.project(proj_x);

    BOOST_CHECK(set.isInside(proj_x, 1e-12));
    BOOST_CHECK(set.isInside(proj_proj_x, 1e-12));
    BOOST_CHECK(proj_x == proj_proj_x);
    if (set.isInside(x))
      BOOST_CHECK(x == proj_x);

    BOOST_CHECK(fabs((x - proj_x).dot(proj_x)) <= 1e-12); // orthogonal projection
  }
}
