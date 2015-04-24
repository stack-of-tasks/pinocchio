//
// Copyright (c) 2015 CNRS
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

#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/force-set.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ConstraintTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

BOOST_AUTO_TEST_SUITE ( Constraint )

BOOST_AUTO_TEST_CASE ( test_ForceSet )
{
  using namespace se3;
  typedef Eigen::Matrix<double,4,4> Matrix4;
  typedef SE3::Matrix6 Matrix6;
  typedef SE3::Vector3 Vector3;
  typedef Force::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  ForceSet F(12);
  ForceSet F2(Eigen::Matrix<double,3,2>::Zero(),Eigen::Matrix<double,3,2>::Zero());
  F.block(10,2) = F2;
  BOOST_CHECK_EQUAL(F.matrix().col(10).norm() , 0.0 );
  assert( isnan(F.matrix()(0,9)) );

  std::cout << "F10 = " << F2.matrix() << std::endl;
  std::cout << "F = " << F.matrix() << std::endl;

  ForceSet F3(Eigen::Matrix<double,3,12>::Random(),Eigen::Matrix<double,3,12>::Random());
  ForceSet F4 = amb.act(F3);
  SE3::Matrix6 aXb= amb;
  is_matrix_absolutely_closed((aXb.transpose().inverse()*F3.matrix()), F4.matrix(), 1e-12);


  ForceSet bF = bmc.act(F3);
  ForceSet aF = amb.act(bF); 
  ForceSet aF2 = amc.act(F3);
  is_matrix_absolutely_closed(aF.matrix(), aF2.matrix(), 1e-12);

  ForceSet F36 = amb.act(F3.block(3,6));
  is_matrix_absolutely_closed((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)), F36.matrix(), 1e-12);

  
  ForceSet F36full(12); F36full.block(3,6) = amb.act(F3.block(3,6)); 
  is_matrix_absolutely_closed((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)),
  														F36full.matrix().block(0,3,6,6),
  														1e-12);
}

BOOST_AUTO_TEST_CASE ( test_ConstraintRX )
{
  using namespace se3;

  Inertia Y = Inertia::Random();
  JointRX::ConstraintRevolute S;

  std::cout << "Y = \n" << Y.matrix() << std::endl;
  std::cout << "S = \n" << ((ConstraintXd)S).matrix() << std::endl;

  ForceSet F(1); F.block(0,1) = Y*S;
  std::cout << "Y*S = \n" << (Y*S).matrix() << std::endl;
  std::cout << "F=Y*S = \n" << F.matrix() << std::endl;
  is_matrix_absolutely_closed(F.matrix(), Y.toMatrix().col(3), 1e-12);

  ForceSet F2( Eigen::Matrix<double,3,9>::Random(),Eigen::Matrix<double,3,9>::Random() );
  Eigen::MatrixXd StF2 = S.transpose()*F2.block(5,3);
  is_matrix_absolutely_closed(StF2,
                              ConstraintXd(S).matrix().transpose()*F2.matrix().block(0,5,6,3),
                              1e-12);
}

BOOST_AUTO_TEST_SUITE_END ()


