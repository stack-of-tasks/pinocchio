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

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/tools/timer.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ComTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


BOOST_AUTO_TEST_SUITE ( ComTest)

BOOST_AUTO_TEST_CASE ( test_com )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
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
}

BOOST_AUTO_TEST_CASE ( test_jacobian_com )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model), data_ref(model);
  
  Model::JointIndex root_id = model.getJointId("larm1_joint");
  const int root_idx_v = idx_v(model.joints[root_id]);
  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
//  v.head(idx_v(model.joints[root_id-1])).setZero();
  
  centerOfMass(model,data_ref,q,v);
  jacobianCenterOfMass(model,data_ref,q);
  
  Data::Matrix3x Jsubtree_com(3,model.nv);
//  getSubtreeJacobianCenterOfMass(model,data,root_id,Jsubtree_com);
  computeSubtreeJacobianCenterOfMass(model,data,root_id,q);
  
  Model::IndexVector & subtree = model.subtrees[root_id];
  for(Model::IndexVector::iterator it = subtree.begin(); it != subtree.end(); ++it)
  {
    BOOST_CHECK(data_ref.oMi[*it].isApprox(data.oMi[*it]));
    BOOST_CHECK(data_ref.com[*it].isApprox(data.com[*it]));
  }
  BOOST_CHECK(data_ref.J.middleCols(root_idx_v,data.nvSubtree[root_id]).isApprox(data.J.middleCols(root_idx_v,data.nvSubtree[root_id])));
  
  Motion::Vector3 vcom_subtree(data_ref.oMi[root_id].rotation()*data_ref.vcom[root_id]);
  BOOST_CHECK(vcom_subtree.isApprox(data.Jcom*v,1e-12));
  
  
}


//BOOST_AUTO_TEST_CASE ( test_timings )
//{
//  using namespace Eigen;
//  using namespace se3;
//
//  se3::Model model;
//  se3::buildModels::humanoidSimple(model);
//  se3::Data data(model);
//
//  long flag = BOOST_BINARY(1111);
//  StackTicToc timer(StackTicToc::US); 
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
