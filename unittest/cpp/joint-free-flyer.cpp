//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#define BOOST_TEST_MODULE joint_free_flyer

#include "pinocchio/multibody/joint.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(joint_free_flyer_spatial)
{
  Motion v(Motion::Random());

  JointMotionSubspaceIdentityTpl<double, 0> constraint;
  Motion Sv = constraint * v.toVector();

  BOOST_CHECK(Sv == v);
}
