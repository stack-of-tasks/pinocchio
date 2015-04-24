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

#ifdef NDEBUG
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/tools/timer.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ComTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"


BOOST_AUTO_TEST_SUITE ( ComTest)

BOOST_AUTO_TEST_CASE ( test_com )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  VectorXd q = VectorXd::Zero(model.nq);
  data.M.fill(0);  crba(model,data,q);

	/* Test COM against CRBA*/
  Vector3d com = centerOfMass(model,data,q);
  is_matrix_absolutely_closed(data.com[0], getComFromCrba(model,data), 1e-12);


	/* Test COM against Jcom (both use different way of compute the COM. */
  com = centerOfMass(model,data,q);
  jacobianCenterOfMass(model,data,q);
  is_matrix_absolutely_closed(com, data.com[0], 1e-12);

	/* Test Jcom against CRBA  */
  Eigen::MatrixXd Jcom = jacobianCenterOfMass(model,data,q);
  is_matrix_absolutely_closed(Jcom, getJacobianComFromCrba(model,data), 1e-12);


  std::cout << "com = [ " << data.com[0].transpose() << " ];" << std::endl;
  std::cout << "mass = [ " << data.mass[0] << " ];" << std::endl;
  std::cout << "Jcom = [ " << data.Jcom << " ];" << std::endl;
  std::cout << "M3 = [ " << data.M.topRows<3>() << " ];" << std::endl;
}


BOOST_AUTO_TEST_CASE ( test_timings )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  long flag = BOOST_BINARY(1111);
  StackTicToc timer(StackTicToc::US); 
  #ifdef NDEBUG
    #ifdef _INTENSE_TESTING_
      const int NBT = 1000*1000;
    #else
      const int NBT = 10;
    #endif
  #else 
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant)  " ;
  #endif

  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
  if(verbose) std::cout <<"--" << std::endl;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  if( flag >> 0 & 1 )
  {
    timer.tic();
    SMOOTH(NBT)
    {
      centerOfMass(model,data,q);
    }
    if(verbose) std::cout << "COM =\t";
    timer.toc(std::cout,NBT);
  }

  if( flag >> 1 & 1 )
  {
    timer.tic();
    SMOOTH(NBT)
    {
      centerOfMass(model,data,q,false);
    }
    if(verbose) std::cout << "Without sub-tree =\t";
    timer.toc(std::cout,NBT);
  }
  
  if( flag >> 2 & 1 )
  {
    timer.tic();
    SMOOTH(NBT)
    {
      jacobianCenterOfMass(model,data,q);
    }
    if(verbose) std::cout << "Jcom =\t";
    timer.toc(std::cout,NBT);
  }
}

BOOST_AUTO_TEST_SUITE_END ()
