//
// Copyright (c) 2015-2018 CNRS
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

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_com )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  
  crba(model,data,q);
  

	/* Test COM against CRBA*/
  Vector3d com = centerOfMass(model,data,q);
  BOOST_CHECK(data.com[0].isApprox(getComFromCrba(model,data), 1e-12));

	/* Test COM against Jcom (both use different way of compute the COM. */
  com = centerOfMass(model,data,q);
  jacobianCenterOfMass(model,data,q);
  BOOST_CHECK(com.isApprox(data.com[0], 1e-12));

	/* Test COM against Jcom (both use different way of compute the COM. */
  centerOfMass(model,data,q,v,a);
  BOOST_CHECK(com.isApprox(data.com[0], 1e-12));

  /* Test vCoM againt nle algorithm without gravity field */
  a.setZero();
  model.gravity.setZero();
  centerOfMass(model,data,q,v,a);
  nonLinearEffects(model, data, q, v);
  
  se3::SE3::Vector3 acom_from_nle (data.nle.head <3> ()/data.mass[0]);
  BOOST_CHECK((data.liMi[1].rotation() * acom_from_nle).isApprox(data.acom[0], 1e-12));

	/* Test Jcom against CRBA  */
  Eigen::MatrixXd Jcom = jacobianCenterOfMass(model,data,q);
  BOOST_CHECK(data.Jcom.isApprox(getJacobianComFromCrba(model,data), 1e-12));

  /* Test CoM vecolity againt jacobianCenterOfMass */
  BOOST_CHECK((Jcom * v).isApprox(data.vcom[0], 1e-12));
  
  
  centerOfMass(model,data,q,v);
  /* Test CoM vecolity againt jacobianCenterOfMass */
  BOOST_CHECK((Jcom * v).isApprox(data.vcom[0], 1e-12));


//  std::cout << "com = [ " << data.com[0].transpose() << " ];" << std::endl;
//  std::cout << "mass = [ " << data.mass[0] << " ];" << std::endl;
//  std::cout << "Jcom = [ " << data.Jcom << " ];" << std::endl;
//  std::cout << "M3 = [ " << data.M.topRows<3>() << " ];" << std::endl;
}


//BOOST_AUTO_TEST_CASE ( test_timings )
//{
//  using namespace Eigen;
//  using namespace se3;
//
//  se3::Model model;
//  se3::buildModels::humanoidRandom(model);
//  se3::Data data(model);
//
//  long flag = BOOST_BINARY(1111);
//  PinocchioTicToc timer(PinocchioTicToc::US); 
//  #ifdef NDEBUG
//    #ifdef _INTENSE_TESTING_
//      const size_t NBT = 1000*1000;
//    #else
//      const size_t NBT = 10;
//    #endif
//  #else 
//    const size_t NBT = 1;
//    std::cout << "(the time score in debug mode is not relevant)  " ;
//  #endif
//
//  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
//  if(verbose) std::cout <<"--" << std::endl;
//  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
//
//  if( flag >> 0 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      centerOfMass(model,data,q);
//    }
//    if(verbose) std::cout << "COM =\t";
//    timer.toc(std::cout,NBT);
//  }
//
//  if( flag >> 1 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      centerOfMass(model,data,q,false);
//    }
//    if(verbose) std::cout << "Without sub-tree =\t";
//    timer.toc(std::cout,NBT);
//  }
//  
//  if( flag >> 2 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      jacobianCenterOfMass(model,data,q);
//    }
//    if(verbose) std::cout << "Jcom =\t";
//    timer.toc(std::cout,NBT);
//  }
//}

BOOST_AUTO_TEST_SUITE_END ()
