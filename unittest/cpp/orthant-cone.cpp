//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE orthant_cone

#include "pinocchio/constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_orthant)
{
  const int num_tests = int(1e6);
  const int dim = 10;

  const NonNegativeOrthantCone orthant;
  typedef typename NonNegativeOrthantCone::Vector Vector;

  BOOST_CHECK(orthant.isInside(Vector::Zero(dim)));
  BOOST_CHECK(orthant.project(Vector::Zero(dim)) == Vector::Zero(dim));
  BOOST_CHECK(orthant.dual() == orthant);

  for (int k = 0; k < num_tests; ++k)
  {
    const Vector x = Vector::Random(dim);

    // Cone
    const auto proj_x = orthant.project(x);
    const auto proj_proj_x = orthant.project(proj_x);

    BOOST_CHECK(orthant.isInside(proj_x, 1e-12));
    BOOST_CHECK(orthant.isInside(proj_proj_x, 1e-12));
    BOOST_CHECK(proj_x == proj_proj_x);
    if (orthant.isInside(x))
      BOOST_CHECK(x == proj_x);

    BOOST_CHECK(fabs((x - proj_x).dot(proj_x)) <= 1e-12); // orthogonal projection
  }

  for (int k = 0; k < num_tests; ++k)
  {
    const Vector x = Vector::Random(dim);
    const Vector x_proj = orthant.project(x);

    BOOST_CHECK((x_proj.array() >= 0).all());
  }
}
