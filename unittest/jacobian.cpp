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
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template<typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived> & x)
{
  return ((x - x).array() == (x - x).array()).all();
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);

  VectorXd q = VectorXd::Zero(model.nq);
  computeJointJacobians(model,data,q);

  Model::Index idx = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1); 
  Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);
  getJointJacobian<WORLD>(model,data,idx,Jrh);

   /* Test J*q == v */
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Zero(model.nv);
  rnea( model,data,q,qdot,qddot );
  Motion v = data.oMi[idx].act( data.v[idx] );
  BOOST_CHECK(v.toVector().isApprox(Jrh*qdot,1e-12));


  /* Test local jacobian: rhJrh == rhXo oJrh */ 
  Data::Matrix6x rhJrh(6,model.nv); rhJrh.fill(0);
  getJointJacobian<LOCAL>(model,data,idx,rhJrh);
  Data::Matrix6x XJrh(6,model.nv); 
  motionSet::se3Action( data.oMi[idx].inverse(), Jrh,XJrh );
  BOOST_CHECK(XJrh.isApprox(rhJrh,1e-12));

  jointJacobian(model,data,q,idx,XJrh);
  BOOST_CHECK(XJrh.isApprox(rhJrh,1e-12));
  
  /* Test computeJointJacobians with pre-computation of the forward kinematics */
  Data data_fk(model);
  forwardKinematics(model, data_fk, q);
  computeJointJacobians(model, data_fk);
  
  BOOST_CHECK(data_fk.J.isApprox(data.J));

}

BOOST_AUTO_TEST_CASE ( test_jacobian_time_variation )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);
  se3::Data data_ref(model);
  
  VectorXd q = randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) );
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);
  
  computeJointJacobiansTimeVariation(model,data,q,v);
  
  BOOST_CHECK(isFinite(data.dJ));
  
  forwardKinematics(model,data_ref,q,v,a);
  Model::Index idx = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1);
  
  Data::Matrix6x J(6,model.nv); J.fill(0.);
  Data::Matrix6x dJ(6,model.nv); dJ.fill(0.);
  
  // Regarding to the world origin
  getJointJacobian<WORLD>(model,data,idx,J);
  getJointJacobianTimeVariation<WORLD>(model,data,idx,dJ);
  
  Motion v_idx(J*v);
  BOOST_CHECK(v_idx.isApprox(data_ref.oMi[idx].act(data_ref.v[idx])));
  
  Motion a_idx(J*a + dJ*v);
  const Motion & a_ref = data_ref.oMi[idx].act(data_ref.a[idx]);
  BOOST_CHECK(a_idx.isApprox(a_ref));
  
  
  // Regarding to the local frame
  getJointJacobian<LOCAL>(model,data,idx,J);
  getJointJacobianTimeVariation<LOCAL>(model,data,idx,dJ);
  
  v_idx = (Motion::Vector6)(J*v);
  BOOST_CHECK(v_idx.isApprox(data_ref.v[idx]));
  
  a_idx = (Motion::Vector6)(J*a + dJ*v);
  BOOST_CHECK(a_idx.isApprox(data_ref.a[idx]));
  
  // compare to finite differencies
  {
    Data data_ref(model), data_ref_plus(model);
    
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model,q,alpha*v);
    
    Data::Matrix6x J_ref(6,model.nv); J_ref.fill(0.);
    computeJointJacobians(model,data_ref,q);
    getJointJacobian<WORLD>(model,data_ref,idx,J_ref);
    
    Data::Matrix6x J_ref_plus(6,model.nv); J_ref_plus.fill(0.);
    computeJointJacobians(model,data_ref_plus,q_plus);
    getJointJacobian<WORLD>(model,data_ref_plus,idx,J_ref_plus);
    
    Data::Matrix6x dJ_ref(6,model.nv); dJ_ref.fill(0.);
    dJ_ref = (J_ref_plus - J_ref)/alpha;
    
    computeJointJacobiansTimeVariation(model,data,q,v);
    Data::Matrix6x dJ(6,model.nv); dJ.fill(0.);
    getJointJacobianTimeVariation<WORLD>(model,data,idx,dJ);
    
    BOOST_CHECK(dJ.isApprox(dJ_ref,sqrt(alpha)));
  }
}



BOOST_AUTO_TEST_CASE ( test_timings )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);

  long flag = BOOST_BINARY(1111);
  PinocchioTicToc timer(PinocchioTicToc::US); 
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
      computeJointJacobians(model,data,q);
    }
    if(verbose) std::cout << "Compute =\t";
    timer.toc(std::cout,NBT);
  }

  if( flag >> 1 & 1 )
  {
    computeJointJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJointJacobian<WORLD>(model,data,idx,Jrh);
    }
    if(verbose) std::cout << "Copy =\t";
    timer.toc(std::cout,NBT);
  }
  
  if( flag >> 2 & 1 )
  {
    computeJointJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      getJointJacobian<LOCAL>(model,data,idx,Jrh);
    }
    if(verbose) std::cout << "Change frame =\t";
    timer.toc(std::cout,NBT);
  }
  
  if( flag >> 3 & 1 )
  {
    computeJointJacobians(model,data,q);
    Model::Index idx = model.existJointName("rarm6")?model.getJointId("rarm6"):(Model::Index)(model.njoints-1); 
    Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);

    timer.tic();
    SMOOTH(NBT)
    {
      jointJacobian(model,data,q,idx,Jrh);
    }
    if(verbose) std::cout << "Single jacobian =\t";
    timer.toc(std::cout,NBT);
  }
}

BOOST_AUTO_TEST_SUITE_END ()

