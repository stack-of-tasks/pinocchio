//
// Copyright (c) 2015-2016 CNRS
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
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

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
    Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);
    q.segment <4> (3).normalize();
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
    BOOST_CHECK(M.isApprox(data.M,1e-12));
    
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
  
BOOST_AUTO_TEST_CASE (test_ccrb)
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model), data_ref(model);
  
  Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  Eigen::VectorXd v = Eigen::VectorXd::Ones(model.nv);
  
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.Ycrb[0] = data_ref.liMi[1].act(data_ref.Ycrb[1]);
  
  se3::SE3 cMo (se3::SE3::Matrix3::Identity(), -getComFromCrba(model, data_ref));
  
  ccrba(model, data, q, v);
  BOOST_CHECK(data.com[0].isApprox(-cMo.translation(),1e-12));
  BOOST_CHECK(data.Ycrb[0].matrix().isApprox(data_ref.Ycrb[0].matrix(),1e-12));
  
  se3::Inertia Ig_ref (cMo.act(data.Ycrb[0]));
  BOOST_CHECK(data.Ig.matrix().isApprox(Ig_ref.matrix(),1e-12));
  
  se3::SE3 oM1 (data_ref.liMi[1]);
  se3::SE3 cM1 (cMo * oM1);
  
  se3::Data::Matrix6x Ag_ref (cM1.inverse().toActionMatrix().transpose() * data_ref.M.topRows <6> ());
  BOOST_CHECK(data.Ag.isApprox(Ag_ref,1e-12));
}

BOOST_AUTO_TEST_SUITE_END ()

