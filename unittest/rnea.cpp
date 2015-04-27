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
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

//#define __SSE3__
#include <fenv.h>

#ifdef __SSE3__
#include <pmmintrin.h>
#endif

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE RneaTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

BOOST_AUTO_TEST_SUITE ( Rnea )

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  #ifdef __SSE3__
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
  #endif
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  
  se3::Data data(model);
  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;

  VectorXd q = VectorXd::Random(model.nq);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  #ifdef NDEBUG
    int NBT = 10000;
  #else
    int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant)  " ;
  #endif

  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(NBT)
    {
      rnea(model,data,q,v,a);
    }
  timer.toc(std::cout,NBT);

}
BOOST_AUTO_TEST_SUITE_END ()
