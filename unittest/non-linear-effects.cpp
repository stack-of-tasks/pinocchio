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

/*
 * Unittest of the RNE algorithm. The code simply test that the algorithm does
 * not cause any serious errors. The numerical values are not cross validated
 * in any way.
 *
 */

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE NLETests
#include <boost/test/unit_test.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

#include <iostream>

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif



BOOST_AUTO_TEST_SUITE ( NLE )

BOOST_AUTO_TEST_CASE ( test_against_rnea )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  se3::Data data_nle(model);
  se3::Data data_rnea(model);

  VectorXd q (VectorXd::Random(model.nq));
  VectorXd v (VectorXd::Random(model.nv));

  VectorXd tau_nle (VectorXd::Zero (model.nv));
  VectorXd tau_rnea (VectorXd::Zero (model.nv));

  // -------
  q.setZero ();
  v.setZero ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  BOOST_CHECK( tau_nle.isApprox(tau_rnea,1e-12) );

  // -------
  q.setZero ();
  v.setOnes ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  BOOST_CHECK( tau_nle.isApprox(tau_rnea,1e-12) );

  // -------
  q.setOnes ();
  v.setOnes ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  BOOST_CHECK( tau_nle.isApprox(tau_rnea,1e-12) );

  // -------
  q.setRandom ();
  v.setRandom ();

  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));

  BOOST_CHECK( tau_nle.isApprox(tau_rnea,1e-12) );
}

BOOST_AUTO_TEST_SUITE_END ()

//int main()
//{
//#ifdef __SSE3__
//  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
//#endif
//
//  using namespace Eigen;
//  using namespace se3;
//
//  se3::Model model; buildModels::humanoidSimple(model);
//  
//  se3::Data data(model);
//
//  VectorXd q (VectorXd::Random(model.nq));
//  VectorXd v (VectorXd::Random(model.nv));
// 
//#ifdef NDEBUG
//  int NBT = 10000;
//#else
//  int NBT = 1;
//  std::cout << "(the time score in debug mode is not relevant)  " ;
//#endif
//
//  StackTicToc timer(StackTicToc::US); timer.tic();
//  SMOOTH(NBT)
//    {
//      nonLinearEffects(model,data,q,v);
//    }
//  timer.toc(std::cout,NBT);
//
//  return 0;
//}
