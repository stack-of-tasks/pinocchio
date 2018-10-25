//
// Copyright (c) 2017-2018 CNRS
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
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_all)
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  forwardKinematics(model,data_ref,q,v,a);
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  
  for(size_t i = 1; i < (size_t)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i].isApprox(data_ref.oMi[i]));
    BOOST_CHECK(data.v[i].isApprox(data_ref.v[i]));
    BOOST_CHECK(data.ov[i].isApprox(data_ref.oMi[i].act(data_ref.v[i])));
    BOOST_CHECK(data.a[i].isApprox(data_ref.a[i]));
    BOOST_CHECK(data.oa[i].isApprox(data_ref.oMi[i].act(data_ref.a[i])));
  }
  
  computeJointJacobians(model,data_ref,q);
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  
  computeJointJacobiansTimeVariation(model, data_ref, q, v);
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
}

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_velocity)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  
  const Model::JointIndex jointId = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  Data::Matrix6x partial_dq(6,model.nv); partial_dq.setZero();
  Data::Matrix6x partial_dq_local(6,model.nv); partial_dq_local.setZero();
  Data::Matrix6x partial_dv(6,model.nv); partial_dv.setZero();
  Data::Matrix6x partial_dv_local(6,model.nv); partial_dv_local.setZero();
  
  getJointVelocityDerivatives<WORLD>(model,data,jointId,
                                     partial_dq,partial_dv);
  
  getJointVelocityDerivatives<LOCAL>(model,data,jointId,
                                     partial_dq_local,partial_dv_local);
  
  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  computeJointJacobians(model,data_ref,q);
  getJointJacobian<WORLD>(model,data_ref,jointId,J_ref);
  getJointJacobian<LOCAL>(model,data_ref,jointId,J_ref_local);
  
  BOOST_CHECK(partial_dv.isApprox(J_ref));
  BOOST_CHECK(partial_dv_local.isApprox(J_ref_local));
  
  // Check against finite differences
  Data::Matrix6x partial_dq_fd(6,model.nv); partial_dq_fd.setZero();
  Data::Matrix6x partial_dq_fd_local(6,model.nv); partial_dq_fd_local.setZero();
  Data::Matrix6x partial_dv_fd(6,model.nv); partial_dv_fd.setZero();
  Data::Matrix6x partial_dv_fd_local(6,model.nv); partial_dv_fd_local.setZero();
  const double alpha = 1e-6;
  
  // dvel/dv
  Eigen::VectorXd v_plus(v);
  Data data_plus(model);
  forwardKinematics(model,data_ref,q,v);
  Motion v0(data_ref.oMi[jointId].act(data_ref.v[jointId]));
  Motion v0_local(data_ref.v[jointId]);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus);
    
    partial_dv_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.v[jointId]) - v0).toVector()/alpha;
    partial_dv_fd_local.col(k) = (data_plus.v[jointId] - v0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(partial_dv.isApprox(partial_dv_fd,sqrt(alpha)));
  BOOST_CHECK(partial_dv_local.isApprox(partial_dv_fd_local,sqrt(alpha)));
  

  // dvel/dq
  Eigen::VectorXd q_plus(q), v_eps(Eigen::VectorXd::Zero(model.nv));
  forwardKinematics(model,data_ref,q,v);
  v0 = data_ref.oMi[jointId].act(data_ref.v[jointId]);
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_plus,q_plus,v);
    
    partial_dq_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.v[jointId]) - v0).toVector()/alpha;
    partial_dq_fd_local.col(k) = (data_plus.v[jointId] - v0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }
  
  BOOST_CHECK(partial_dq.isApprox(partial_dq_fd,sqrt(alpha)));
  
  BOOST_CHECK(partial_dq_local.isApprox(partial_dq_fd_local,sqrt(alpha)));
  
//  computeJointJacobiansTimeVariation(model,data_ref,q,v);
//  Data::Matrix6x dJ_ref(6,model.nv); dJ_ref.setZero();
//  getJointJacobianTimeVariation<WORLD>(model,data_ref,jointId,dJ_ref);
//  BOOST_CHECK(partial_dq.isApprox(dJ_ref));
//  
//  std::cout << "partial_dq\n" << partial_dq << std::endl;
//  std::cout << "dJ_ref\n" << dJ_ref << std::endl;
  
}

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_acceleration)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  
  
  const Model::JointIndex jointId = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  Data::Matrix6x v_partial_dq(6,model.nv); v_partial_dq.setZero();
  Data::Matrix6x v_partial_dq_local(6,model.nv); v_partial_dq_local.setZero();
  Data::Matrix6x a_partial_dq(6,model.nv); a_partial_dq.setZero();
  Data::Matrix6x a_partial_dq_local(6,model.nv); a_partial_dq_local.setZero();
  Data::Matrix6x a_partial_dv(6,model.nv); a_partial_dv.setZero();
  Data::Matrix6x a_partial_dv_local(6,model.nv); a_partial_dv_local.setZero();
  Data::Matrix6x a_partial_da(6,model.nv); a_partial_da.setZero();
  Data::Matrix6x a_partial_da_local(6,model.nv); a_partial_da_local.setZero();
  
  getJointAccelerationDerivatives<WORLD>(model,data,jointId,
                                         v_partial_dq,
                                         a_partial_dq,a_partial_dv,a_partial_da);
  
  getJointAccelerationDerivatives<LOCAL>(model,data,jointId,
                                         v_partial_dq_local,
                                         a_partial_dq_local,a_partial_dv_local,a_partial_da_local);
  
  // Check v_partial_dq against getJointVelocityDerivatives
  {
    Data data_v(model);
    computeForwardKinematicsDerivatives(model,data_v,q,v,a);
    
    Data::Matrix6x v_partial_dq_ref(6,model.nv); v_partial_dq_ref.setZero();
    Data::Matrix6x v_partial_dv_ref(6,model.nv); v_partial_dv_ref.setZero();
    Data::Matrix6x v_partial_dq_ref_local(6,model.nv); v_partial_dq_ref_local.setZero();
    
    getJointVelocityDerivatives<WORLD>(model,data_v,jointId,
                                       v_partial_dq_ref,v_partial_dv_ref);
    
    BOOST_CHECK(v_partial_dq.isApprox(v_partial_dq_ref));
    getJointVelocityDerivatives<LOCAL>(model,data_v,jointId,
                                       v_partial_dq_ref_local,v_partial_dv_ref);
    
    BOOST_CHECK(v_partial_dq_local.isApprox(v_partial_dq_ref_local));
  }
  
  
  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  computeJointJacobians(model,data_ref,q);
  getJointJacobian<WORLD>(model,data_ref,jointId,J_ref);
  getJointJacobian<LOCAL>(model,data_ref,jointId,J_ref_local);
  
  BOOST_CHECK(a_partial_da.isApprox(J_ref));
  BOOST_CHECK(a_partial_da_local.isApprox(J_ref_local));
  
  // Check against finite differences
  Data::Matrix6x a_partial_da_fd(6,model.nv); a_partial_da_fd.setZero();
  Data::Matrix6x a_partial_da_fd_local(6,model.nv); a_partial_da_fd_local.setZero();
  const double alpha = 1e-8;
  
  Eigen::VectorXd v_plus(v), a_plus(a);
  Data data_plus(model);
  forwardKinematics(model,data_ref,q,v,a);
  
  // dacc/da
  Motion a0(data_ref.oMi[jointId].act(data_ref.a[jointId]));
  Motion a0_local(data_ref.a[jointId]);
  for(int k = 0; k < model.nv; ++k)
  {
    a_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v,a_plus);
    
    a_partial_da_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;
    a_partial_da_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    a_plus[k] -= alpha;
  }
  BOOST_CHECK(a_partial_da.isApprox(a_partial_da_fd,sqrt(alpha)));
  
  BOOST_CHECK(a_partial_da_local.isApprox(a_partial_da_fd_local,sqrt(alpha)));
  motionSet::se3Action(data_ref.oMi[jointId].inverse(),a_partial_da,a_partial_da_local);
  BOOST_CHECK(a_partial_da_local.isApprox(a_partial_da_fd_local,sqrt(alpha)));
  
  // dacc/dv
  Data::Matrix6x a_partial_dv_fd(6,model.nv); a_partial_dv_fd.setZero();
  Data::Matrix6x a_partial_dv_fd_local(6,model.nv); a_partial_dv_fd_local.setZero();
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus,a);
    
    a_partial_dv_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;
    a_partial_dv_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(a_partial_dv.isApprox(a_partial_dv_fd,sqrt(alpha)));
  
  BOOST_CHECK(a_partial_dv_local.isApprox(a_partial_dv_fd_local,sqrt(alpha)));
  motionSet::se3Action(data_ref.oMi[jointId].inverse(),a_partial_dv,a_partial_dv_local);
  BOOST_CHECK(a_partial_dv_local.isApprox(a_partial_dv_fd_local,sqrt(alpha)));
  
  // dacc/dq
  a_partial_dq.setZero();
  a_partial_dv.setZero();
  a_partial_da.setZero();
  
  a_partial_dq_local.setZero();
  a_partial_dv_local.setZero();
  a_partial_da_local.setZero();
  
  Data::Matrix6x a_partial_dq_fd(6,model.nv); a_partial_dq_fd.setZero();
  Data::Matrix6x a_partial_dq_fd_local(6,model.nv); a_partial_dq_fd_local.setZero();
//  a.setZero();
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  getJointAccelerationDerivatives<WORLD>(model,data,jointId,
                                         v_partial_dq,
                                         a_partial_dq,a_partial_dv,a_partial_da);
  
  
  getJointAccelerationDerivatives<LOCAL>(model,data,jointId,
                                         v_partial_dq_local,
                                         a_partial_dq_local,a_partial_dv_local,a_partial_da_local);
  
  Eigen::VectorXd q_plus(q), v_eps(Eigen::VectorXd::Zero(model.nv));
  forwardKinematics(model,data_ref,q,v,a);
  a0 = data_ref.oMi[jointId].act(data_ref.a[jointId]);
  a0_local = data_ref.a[jointId];
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_plus,q_plus,v,a);
    
    a_partial_dq_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;
    a_partial_dq_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }
  
  BOOST_CHECK(a_partial_dq.isApprox(a_partial_dq_fd,sqrt(alpha)));

  BOOST_CHECK(a_partial_dq_local.isApprox(a_partial_dq_fd_local,sqrt(alpha)));
 
}

BOOST_AUTO_TEST_SUITE_END()
