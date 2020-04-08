//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(JointFreeFlyer)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  Motion v(Motion::Random());
  
  ConstraintIdentityTpl<double,0> constraint;
  Motion Sv = constraint * v.toVector();
  
  BOOST_CHECK(Sv == v);
}

BOOST_AUTO_TEST_SUITE_END()
