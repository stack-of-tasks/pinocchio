//
// Copyright (c) 2020 INRIA
//

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "pinocchio/algorithm/utils/motion.hpp"

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_motion)
{
  using namespace pinocchio;

  const SE3 placement = SE3::Random();
  const Motion m_in = Motion::Random();

  // test case 1-2-3
  {
    BOOST_CHECK(changeReferenceFrame(placement, m_in, LOCAL, LOCAL) == m_in);
    BOOST_CHECK(changeReferenceFrame(placement, m_in, WORLD, WORLD) == m_in);
    BOOST_CHECK(
      changeReferenceFrame(placement, m_in, LOCAL_WORLD_ALIGNED, LOCAL_WORLD_ALIGNED) == m_in);
  }

  const ReferenceFrame all_choices[3] = {LOCAL, WORLD, LOCAL_WORLD_ALIGNED};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      BOOST_CHECK(
        changeReferenceFrame(SE3::Identity(), m_in, all_choices[i], all_choices[j]) == m_in);

  // LOCAL/WORLD and WORLD/LOCAL
  {
    Motion m_sol_w = placement.act(m_in);
    BOOST_CHECK(changeReferenceFrame(placement, m_in, LOCAL, WORLD) == m_sol_w);
    BOOST_CHECK(changeReferenceFrame(placement, m_sol_w, WORLD, LOCAL).isApprox(m_in));
  }

  // LOCAL/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/LOCAL
  {
    const SE3 placement_LWA(placement.rotation(), SE3::Vector3::Zero());
    Motion m_sol_lwa = placement_LWA.act(m_in);
    BOOST_CHECK(changeReferenceFrame(placement, m_in, LOCAL, LOCAL_WORLD_ALIGNED) == m_sol_lwa);
    BOOST_CHECK(
      changeReferenceFrame(placement, m_sol_lwa, LOCAL_WORLD_ALIGNED, LOCAL).isApprox(m_in));
  }

  // WORLD/LOCAL_WORLD_ALIGNED and LOCAL_WORLD_ALIGNED/WORLD
  {
    const SE3 placement_W(SE3::Matrix3::Identity(), placement.translation());
    Motion m_sol_w = placement_W.act(m_in);
    BOOST_CHECK(
      changeReferenceFrame(placement, m_in, LOCAL_WORLD_ALIGNED, WORLD).isApprox(m_sol_w));
    BOOST_CHECK(
      changeReferenceFrame(placement, m_sol_w, WORLD, LOCAL_WORLD_ALIGNED).isApprox(m_in));
  }
}

BOOST_AUTO_TEST_SUITE_END()
