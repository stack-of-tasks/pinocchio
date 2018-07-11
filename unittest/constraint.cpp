//
// Copyright (c) 2015-2016,2018 CNRS
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

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/force-set.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

#include "utils/macros.hpp"

#include <cmath>
#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE (test_ForceSet)
{
  using namespace se3;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  ForceSet F(12);
  ForceSet F2(Eigen::Matrix<double,3,2>::Zero(),Eigen::Matrix<double,3,2>::Zero());
  F.block(10,2) = F2;
  BOOST_CHECK_EQUAL(F.matrix().col(10).norm() , 0.0 );
  BOOST_CHECK(std::isnan(F.matrix()(0,9)));

  ForceSet F3(Eigen::Matrix<double,3,12>::Random(),Eigen::Matrix<double,3,12>::Random());
  ForceSet F4 = amb.act(F3);
  SE3::Matrix6 aXb= amb;
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix()).isApprox(F4.matrix(), 1e-12));


  ForceSet bF = bmc.act(F3);
  ForceSet aF = amb.act(bF); 
  ForceSet aF2 = amc.act(F3);
  BOOST_CHECK(aF.matrix().isApprox(aF2.matrix(), 1e-12));

  ForceSet F36 = amb.act(F3.block(3,6));
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)).isApprox(F36.matrix(), 1e-12));

  
  ForceSet F36full(12); F36full.block(3,6) = amb.act(F3.block(3,6)); 
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)).isApprox(F36full.matrix().block(0,3,6,6),
                                                            1e-12));
}

BOOST_AUTO_TEST_CASE ( test_ConstraintRX )
{
  using namespace se3;

  Inertia Y = Inertia::Random();
  JointDataRX::Constraint_t S;

  ForceSet F1(1); F1.block(0,1) = Y*S;
  BOOST_CHECK(F1.matrix().isApprox(Y.matrix().col(3), 1e-12));

  ForceSet F2( Eigen::Matrix<double,3,9>::Random(),Eigen::Matrix<double,3,9>::Random() );
  Eigen::MatrixXd StF2 = S.transpose()*F2.block(5,3).matrix();
  BOOST_CHECK(StF2.isApprox(S.matrix().transpose()*F2.matrix().block(0,5,6,3)
                            , 1e-12));
}

BOOST_AUTO_TEST_SUITE_END()
