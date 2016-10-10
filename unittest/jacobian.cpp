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

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  VectorXd q = VectorXd::Zero(model.nq);
  computeJacobians(model,data,q);

  Model::Index idx = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1); 
  Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);
  getJacobian<false>(model,data,idx,Jrh);

   /* Test J*q == v */
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Zero(model.nv);
  rnea( model,data,q,qdot,qddot );
  Motion v = data.oMi[idx].act( data.v[idx] );
  BOOST_CHECK(v.toVector().isApprox(Jrh*qdot,1e-12));


  /* Test local jacobian: rhJrh == rhXo oJrh */ 
  Data::Matrix6x rhJrh(6,model.nv); rhJrh.fill(0);
  getJacobian<true>(model,data,idx,rhJrh);
  Data::Matrix6x XJrh(6,model.nv); 
  motionSet::se3Action( data.oMi[idx].inverse(), Jrh,XJrh );
  BOOST_CHECK(XJrh.isApprox(rhJrh,1e-12));


  data.J.fill(0);
  XJrh = jacobian(model,data,q,idx);
  BOOST_CHECK(XJrh.isApprox(rhJrh,1e-12));

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
      const size_t NBT = 1000*1000;
    #else
      const size_t NBT = 10;
    #endif
  #else 
    const size_t NBT = 1;
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
      computeJacobians(model,data,q);
    }
    if(verbose) std::cout << "Compute =\t";
    timer.toc(std::cout,NBT);
  }

  if( flag >> 1 & 1 )
  {
    computeJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJacobian<false>(model,data,idx,Jrh);
    }
    if(verbose) std::cout << "Copy =\t";
    timer.toc(std::cout,NBT);
  }
  
  if( flag >> 2 & 1 )
  {
    computeJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJacobian<true>(model,data,idx,Jrh);
    }
    if(verbose) std::cout << "Change frame =\t";
    timer.toc(std::cout,NBT);
  }
  
  if( flag >> 3 & 1 )
  {
    computeJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      jacobian(model,data,q,idx);
    }
    if(verbose) std::cout << "Single jacobian =\t";
    timer.toc(std::cout,NBT);
  }
}

BOOST_AUTO_TEST_SUITE_END ()

