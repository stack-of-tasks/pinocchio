//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE box_set

#include "pinocchio/constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_proj)
{
  const int num_tests = int(1e6);
  const int dim = 10;

  using Vector = BoxSet::Vector;
  const Vector lb = -Vector::Ones(dim);
  const Vector ub = Vector::Ones(dim);
  const BoxSet box_constraint(lb, ub);

  BOOST_CHECK(box_constraint.isInside(Vector::Zero(dim)));
  BOOST_CHECK(box_constraint.project(Vector::Zero(dim)) == Vector::Zero(dim));

  for (int k = 0; k < num_tests; ++k)
  {
    const Vector x = Vector::Random(dim);

    // Cone
    const auto proj_x = box_constraint.project(x);
    const auto proj_proj_x = box_constraint.project(proj_x);

    BOOST_CHECK(box_constraint.isInside(proj_x, 1e-12));
    BOOST_CHECK(box_constraint.isInside(proj_proj_x, 1e-12));
    BOOST_CHECK(proj_x == proj_proj_x);
    if (box_constraint.isInside(x))
      BOOST_CHECK(x == proj_x);

    BOOST_CHECK(fabs((x - proj_x).dot(proj_x)) <= 1e-12); // orthogonal projection
  }
}

BOOST_AUTO_TEST_CASE(test_scaled_proj)
{
  const int num_tests = int(1e6);
  const int dim = 10;

  using Vector = BoxSet::Vector;
  const Vector lb = -Vector::Ones(dim);
  const Vector ub = Vector::Ones(dim);
  BoxSet box_constraint(lb, ub);

  Vector scale = Vector::Random(dim);
  scale.array() = scale.array().max(Vector::Ones(dim).array() * 0.1); // ensure scale is positive
  Vector lb_scaled = lb.cwiseQuotient(scale);
  Vector ub_scaled = ub.cwiseQuotient(scale);
  const BoxSet scaled_box_constraint(lb, ub);

  BOOST_CHECK(box_constraint.isInside(Vector::Zero(dim)));
  Vector res(dim);
  box_constraint.scaledProject(Vector::Zero(dim), scale, res);
  BOOST_CHECK(res == Vector::Zero(dim));

  for (int k = 0; k < num_tests; ++k)
  {
    const Vector x = Vector::Random(dim);

    // Cone
    box_constraint.scaledProject(x, scale, res);
    const auto proj_x = res;
    box_constraint.scaledProject(proj_x, scale, res);
    const auto proj_proj_x = res;

    BOOST_CHECK(scaled_box_constraint.isInside(proj_x, 1e-12));
    BOOST_CHECK(scaled_box_constraint.isInside(proj_proj_x, 1e-12));
    BOOST_CHECK(proj_x == proj_proj_x);
    if (scaled_box_constraint.isInside(x))
      BOOST_CHECK(x == proj_x);

    BOOST_CHECK(fabs((x - proj_x).dot(proj_x)) <= 1e-12); // orthogonal projection
  }
}
