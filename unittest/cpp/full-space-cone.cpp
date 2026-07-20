//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE full_space_cone

#include "pinocchio/constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_proj)
{
  const int num_tests = int(1e6);
  const int dim = 10;

  const FullSpaceCone set;

  BOOST_CHECK(set.isInside(FullSpaceCone::Vector::Zero(dim)));
  BOOST_CHECK(set.project(FullSpaceCone::Vector::Zero(dim)) == FullSpaceCone::Vector::Zero(dim));

  for (int k = 0; k < num_tests; ++k)
  {
    const FullSpaceCone::Vector x = FullSpaceCone::Vector::Random(dim);

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
