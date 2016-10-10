//
// Copyright (c) 2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "pinocchio/spatial/explog.hpp"

using namespace se3;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(exp)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random()); v.linear().setZero();
  
  SE3::Matrix3 R = exp3(v.angular());
  M = exp6(v);
  
  BOOST_CHECK(R.isApprox(M.rotation()));
  
}

BOOST_AUTO_TEST_CASE(explog)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  SE3 M_res = exp6(log6(M));
  
  BOOST_CHECK(M_res.isApprox(M));
  
  Motion v_res = log6(exp6(v));
  BOOST_CHECK(v_res.toVector().isApprox(v.toVector()));
}

BOOST_AUTO_TEST_CASE (test_basic)
{
  typedef se3::SE3::Vector3 Vector3;
  typedef se3::SE3::Matrix3 Matrix3;
  typedef Eigen::Matrix4d Matrix4;
  typedef se3::Motion::Vector6 Vector6;
  
  const double EPSILON = 1e-12;
  
  // exp3 and log3.
  Vector3 v3(Vector3::Random());
  Matrix3 R(se3::exp3(v3));
  BOOST_CHECK(R.transpose().isApprox(R.inverse(), EPSILON));
  BOOST_CHECK_SMALL(R.determinant() - 1.0, EPSILON);
  Vector3 v3FromLog(se3::log3(R));
  BOOST_CHECK(v3.isApprox(v3FromLog, EPSILON));
  
  // exp6 and log6.
  se3::Motion nu = se3::Motion::Random();
  se3::SE3 m = se3::exp6(nu);
  BOOST_CHECK(m.rotation().transpose().isApprox(m.rotation().inverse(),
                                                EPSILON));
  BOOST_CHECK_SMALL(m.rotation().determinant() - 1.0, EPSILON);
  se3::Motion nuFromLog(se3::log6(m));
  BOOST_CHECK(nu.linear().isApprox(nuFromLog.linear(), EPSILON));
  BOOST_CHECK(nu.angular().isApprox(nuFromLog.angular(), EPSILON));
  
  Vector6 v6(Vector6::Random());
  se3::SE3 m2(se3::exp6(v6));
  BOOST_CHECK(m2.rotation().transpose().isApprox(m2.rotation().inverse(),
                                                 EPSILON));
  BOOST_CHECK_SMALL(m2.rotation().determinant() - 1.0, EPSILON);
  Matrix4 M = m2.toHomogeneousMatrix();
  se3::Motion nu2FromLog(se3::log6(M));
  Vector6 v6FromLog(nu2FromLog.toVector());
  BOOST_CHECK(v6.isApprox(v6FromLog, EPSILON));
}

BOOST_AUTO_TEST_SUITE_END()
