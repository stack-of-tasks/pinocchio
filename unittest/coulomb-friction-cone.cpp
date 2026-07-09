//
// Copyright (c) 2022-2024 INRIA
//

#define BOOST_TEST_MODULE coulomb_friction_cone

#include "pinocchio/constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

Eigen::Vector3d positiveRandomScaling()
{
  const Eigen::Vector2d random_vec = Eigen::abs(Eigen::Vector2d::Random().array());
  return Eigen::Vector3d(random_vec[0], random_vec[0], random_vec[1]);
}

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

BOOST_AUTO_TEST_CASE(test_weighted_projection)
{
  const int num_tests = int(1e4);
  const double mu = 1;

  const CoulombFrictionCone cone(mu);

  // Test with idendity scaling
  {
    const auto Ones4d = Eigen::Vector3d::Ones();
    for (int k = 0; k < num_tests; ++k)
    {
      const Eigen::Vector3d x = Eigen::Vector3d::Random();
      const Eigen::Vector3d proj_x = cone.weightedProject(x, Ones4d);
      const Eigen::Vector3d proj_x_ref = cone.project(x);

      BOOST_CHECK(proj_x.isApprox(proj_x_ref));
    }
  }

  // Test with any positive scaling
  {
    for (int k = 0; k < num_tests; ++k)
    {
      const Eigen::Vector3d scaling = positiveRandomScaling() + Eigen::Vector3d::Constant(1e-8);
      BOOST_CHECK((scaling.array() > 0).all());

      const Eigen::Vector3d scaling_sqrt = Eigen::sqrt(scaling.array());
      const Eigen::Vector3d scaling_sqrt_inv = Eigen::inverse(scaling_sqrt.array());

      const Eigen::Vector3d x = Eigen::Vector3d::Random();
      const Eigen::Vector3d proj_x = cone.weightedProject(x, scaling);

      const double mu_scale = math::sqrt(scaling[0] / scaling[2]) * mu;
      const CoulombFrictionCone cone_scale(mu_scale);
      const Eigen::Vector3d x_scale = scaling_sqrt.asDiagonal() * x;
      const Eigen::Vector3d proj_x_ref_scale = cone_scale.project(x_scale);
      const Eigen::Vector3d proj_x_ref = scaling_sqrt_inv.array() * proj_x_ref_scale.array();

      //      std::cout << "proj_x: " << proj_x.transpose() << std::endl;
      //      std::cout << "proj_x_ref: " << proj_x_ref.transpose() << std::endl;
      BOOST_CHECK(proj_x.isApprox(proj_x_ref));
    }
  }
}
