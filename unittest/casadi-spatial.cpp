//
// Copyright (c) 2019 INRIA
//

#include <boost/variant.hpp> // to avoid C99 warnings

#include <casadi/casadi.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_se3)
{
  typedef pinocchio::SE3Tpl<casadi::SX> SE3;
  SE3 M1 = SE3::Identity();
  SE3 M2 = SE3::Random();
  
  SE3 M3 = M2 * M1;
  SE3 M1inv = M1.inverse();
}

BOOST_AUTO_TEST_CASE(test_motion)
{
  typedef pinocchio::MotionTpl<casadi::SX> Motion;
  Motion v1 = Motion::Zero();
  Motion v2 = Motion::Random();
  
  Motion v3 = v1 + v2;
}

BOOST_AUTO_TEST_SUITE_END()
