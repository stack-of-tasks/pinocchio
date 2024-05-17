//
// Copyright (c) 2022 INRIA
//

#include <iostream>
#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_proj)
{
  const int num_tests = int(1e5);
  const double mu = .4;

  const CoulombFrictionCone cone(mu);
  const DualCoulombFrictionCone dual_cone = cone.dual();

  BOOST_CHECK(cone.isInside(Eigen::Vector3d::Zero()));
  BOOST_CHECK(cone.project(Eigen::Vector3d::Zero()) == Eigen::Vector3d::Zero());

  BOOST_CHECK(dual_cone.isInside(Eigen::Vector3d::Zero()));
  BOOST_CHECK(dual_cone.project(Eigen::Vector3d::Zero()) == Eigen::Vector3d::Zero());

  for (int k = 0; k < num_tests; ++k)
  {
    const Eigen::Vector3d x = Eigen::Vector3d::Random();

    // Cone
    const Eigen::Vector3d proj_x = cone.project(x);
    const Eigen::Vector3d proj_proj_x = cone.project(proj_x);

    BOOST_CHECK(cone.isInside(proj_x, 1e-12));
    BOOST_CHECK(cone.isInside(proj_proj_x, 1e-12));
    BOOST_CHECK(proj_x.isApprox(proj_proj_x));
    if (cone.isInside(x))
      BOOST_CHECK(x == proj_x);

    BOOST_CHECK(fabs((x - proj_x).dot(proj_x)) <= 1e-12); // orthogonal projection

    // Dual cone
    const Eigen::Vector3d dual_proj_x = dual_cone.project(x);
    const Eigen::Vector3d dual_proj_proj_x = dual_cone.project(dual_proj_x);

    BOOST_CHECK(dual_cone.isInside(dual_proj_x, 1e-12));
    BOOST_CHECK(dual_cone.isInside(dual_proj_proj_x, 1e-12));
    BOOST_CHECK(dual_proj_x.isApprox(dual_proj_proj_x));

    if (dual_cone.isInside(x))
      BOOST_CHECK(x == dual_proj_x);

    BOOST_CHECK(fabs((x - dual_proj_x).dot(dual_proj_x)) <= 1e-12); // orthogonal projection

    // Radial projection
    {
      const Eigen::Vector3d radial_proj_x = cone.computeRadialProjection(x);
      const Eigen::Vector3d radial_proj_radial_proj_x = cone.computeRadialProjection(radial_proj_x);
      BOOST_CHECK(cone.isInside(radial_proj_x, 1e-12));
      BOOST_CHECK(radial_proj_x[2] == x[2] || radial_proj_x[2] == 0.);
      if (radial_proj_x[2] == x[2])
        BOOST_CHECK(
          std::fabs(radial_proj_x.head<2>().normalized().dot(x.head<2>().normalized()) - 1.)
          <= 1e-6);
      else
        BOOST_CHECK(radial_proj_x.head<2>().isZero());
      BOOST_CHECK(radial_proj_radial_proj_x.isApprox(radial_proj_radial_proj_x));
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
