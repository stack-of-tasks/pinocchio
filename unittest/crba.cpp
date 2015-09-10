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
 * Test the CRBA algorithm. The code validates both the computation times and
 * the value by comparing the results of the CRBA with the reconstruction of
 * the mass matrix using the RNEA.
 * For a strong timing benchmark, see benchmark/timings.
 *
 */

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CrbaTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

BOOST_AUTO_TEST_SUITE ( CrbaTest)

BOOST_AUTO_TEST_CASE ( test_crba )
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);
  
  #ifdef NDEBUG
    #ifdef _INTENSE_TESTING_
      const size_t NBT = 1000*1000;
    #else
      const size_t NBT = 10;
    #endif

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
 
    StackTicToc timer(StackTicToc::US); timer.tic();
    SMOOTH(NBT)
      {
        crba(model,data,q);
      }
    timer.toc(std::cout,NBT);
  
  #else
    const size_t NBT = 1;
    using namespace Eigen;
    using namespace se3;

    Eigen::MatrixXd M(model.nv,model.nv);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    data.M.fill(0);  crba(model,data,q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    /* Joint inertia from iterative crba. */
    const Eigen::VectorXd bias = rnea(model,data,q,v,a);
    for(int i=0;i<model.nv;++i)
      { 
        M.col(i) = rnea(model,data,q,v,Eigen::VectorXd::Unit(model.nv,i)) - bias;
      }

    // std::cout << "Mcrb = [ " << data.M << "  ];" << std::endl;
    // std::cout << "Mrne = [  " << M << " ]; " << std::endl;
    is_matrix_absolutely_closed(M,data.M,1e-12);
    
    std::cout << "(the time score in debug mode is not relevant)  " ;
    
    q = Eigen::VectorXd::Zero(model.nq);
   
    StackTicToc timer(StackTicToc::US); timer.tic();
    SMOOTH(NBT)
      {
        crba(model,data,q);
      }
    timer.toc(std::cout,NBT);
  
  #endif // ifndef NDEBUG

}

BOOST_AUTO_TEST_SUITE_END ()

