//
// Copyright (c) 2017-2020 CNRS INRIA
//

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
  using namespace pinocchio;

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
  using namespace pinocchio;
  
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
  Data::Matrix6x partial_dq_local_world_aligned(6,model.nv); partial_dq_local_world_aligned.setZero();
  Data::Matrix6x partial_dq_local(6,model.nv); partial_dq_local.setZero();
  Data::Matrix6x partial_dv(6,model.nv); partial_dv.setZero();
  Data::Matrix6x partial_dv_local_world_aligned(6,model.nv); partial_dv_local_world_aligned.setZero();
  Data::Matrix6x partial_dv_local(6,model.nv); partial_dv_local.setZero();
  
  getJointVelocityDerivatives(model,data,jointId,WORLD,
                              partial_dq,partial_dv);
  
  getJointVelocityDerivatives(model,data,jointId,LOCAL_WORLD_ALIGNED,
                              partial_dq_local_world_aligned,partial_dv_local_world_aligned);
  
  getJointVelocityDerivatives(model,data,jointId,LOCAL,
                              partial_dq_local,partial_dv_local);
  
  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local_world_aligned(6,model.nv); J_ref_local_world_aligned.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  computeJointJacobians(model,data_ref,q);
  getJointJacobian(model,data_ref,jointId,WORLD,J_ref);
  getJointJacobian(model,data_ref,jointId,LOCAL_WORLD_ALIGNED,J_ref_local_world_aligned);
  getJointJacobian(model,data_ref,jointId,LOCAL,J_ref_local);
  
  BOOST_CHECK(partial_dv.isApprox(J_ref));
  BOOST_CHECK(partial_dv_local_world_aligned.isApprox(J_ref_local_world_aligned));
  BOOST_CHECK(partial_dv_local.isApprox(J_ref_local));
  
  // Check against finite differences
  Data::Matrix6x partial_dq_fd(6,model.nv); partial_dq_fd.setZero();
  Data::Matrix6x partial_dq_fd_local_world_aligned(6,model.nv); partial_dq_fd_local_world_aligned.setZero();
  Data::Matrix6x partial_dq_fd_local(6,model.nv); partial_dq_fd_local.setZero();
  Data::Matrix6x partial_dv_fd(6,model.nv); partial_dv_fd.setZero();
  Data::Matrix6x partial_dv_fd_local_world_aligned(6,model.nv); partial_dv_fd_local_world_aligned.setZero();
  Data::Matrix6x partial_dv_fd_local(6,model.nv); partial_dv_fd_local.setZero();
  const double alpha = 1e-8;
  
  // dvel/dv
  Eigen::VectorXd v_plus(v);
  Data data_plus(model);
  forwardKinematics(model,data_ref,q,v);
  SE3 oMi_rot(SE3::Identity());
  oMi_rot.rotation() = data_ref.oMi[jointId].rotation();
  Motion v0(data_ref.oMi[jointId].act(data_ref.v[jointId]));
  Motion v0_local_world_aligned(oMi_rot.act(data_ref.v[jointId]));
  Motion v0_local(data_ref.v[jointId]);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus);
    
    partial_dv_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.v[jointId]) - v0).toVector()/alpha;
    partial_dv_fd_local_world_aligned.col(k) = (oMi_rot.act(data_plus.v[jointId]) - v0_local_world_aligned).toVector()/alpha;
    partial_dv_fd_local.col(k) = (data_plus.v[jointId] - v0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(partial_dv.isApprox(partial_dv_fd,sqrt(alpha)));
  BOOST_CHECK(partial_dv_local_world_aligned.isApprox(partial_dv_fd_local_world_aligned,sqrt(alpha)));
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
    
    SE3 oMi_plus_rot = data_plus.oMi[jointId];
    oMi_plus_rot.translation().setZero();
   
    Motion v_plus_local_world_aligned = oMi_plus_rot.act(data_plus.v[jointId]);
    SE3::Vector3 trans = data_plus.oMi[jointId].translation() - data_ref.oMi[jointId].translation();
    v_plus_local_world_aligned.linear() -= v_plus_local_world_aligned.angular().cross(trans);
    partial_dq_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.v[jointId]) - v0).toVector()/alpha;
    partial_dq_fd_local_world_aligned.col(k) = (v_plus_local_world_aligned - v0_local_world_aligned).toVector()/alpha;
    partial_dq_fd_local.col(k) = (data_plus.v[jointId] - v0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }
  
  BOOST_CHECK(partial_dq.isApprox(partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(partial_dq_local_world_aligned.isApprox(partial_dq_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(partial_dq_local.isApprox(partial_dq_fd_local,sqrt(alpha)));
  
}

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_acceleration)
{
  using namespace Eigen;
  using namespace pinocchio;
  
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
  Data::Matrix6x v_partial_dq_local_world_aligned(6,model.nv); v_partial_dq_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dq(6,model.nv); a_partial_dq.setZero();
  Data::Matrix6x a_partial_dq_local_world_aligned(6,model.nv); a_partial_dq_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dq_local(6,model.nv); a_partial_dq_local.setZero();
  Data::Matrix6x a_partial_dv(6,model.nv); a_partial_dv.setZero();
  Data::Matrix6x a_partial_dv_local_world_aligned(6,model.nv); a_partial_dv_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dv_local(6,model.nv); a_partial_dv_local.setZero();
  Data::Matrix6x a_partial_da(6,model.nv); a_partial_da.setZero();
  Data::Matrix6x a_partial_da_local_world_aligned(6,model.nv); a_partial_da_local_world_aligned.setZero();
  Data::Matrix6x a_partial_da_local(6,model.nv); a_partial_da_local.setZero();
  
  getJointAccelerationDerivatives(model,data,jointId,WORLD,
                                  v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);
                                  
  getJointAccelerationDerivatives(model,data,jointId,LOCAL_WORLD_ALIGNED,
                                  v_partial_dq_local_world_aligned,
                                  a_partial_dq_local_world_aligned,
                                  a_partial_dv_local_world_aligned,
                                  a_partial_da_local_world_aligned);
  
  getJointAccelerationDerivatives(model,data,jointId,LOCAL,
                                  v_partial_dq_local,
                                  a_partial_dq_local,a_partial_dv_local,a_partial_da_local);
  
  // Check v_partial_dq against getJointVelocityDerivatives
  {
    Data data_v(model);
    computeForwardKinematicsDerivatives(model,data_v,q,v,a);
    
    Data::Matrix6x v_partial_dq_ref(6,model.nv); v_partial_dq_ref.setZero();
    Data::Matrix6x v_partial_dq_ref_local_world_aligned(6,model.nv); v_partial_dq_ref_local_world_aligned.setZero();
    Data::Matrix6x v_partial_dq_ref_local(6,model.nv); v_partial_dq_ref_local.setZero();
    Data::Matrix6x v_partial_dv_ref(6,model.nv); v_partial_dv_ref.setZero();
    Data::Matrix6x v_partial_dv_ref_local_world_aligned(6,model.nv); v_partial_dv_ref_local_world_aligned.setZero();
    Data::Matrix6x v_partial_dv_ref_local(6,model.nv); v_partial_dv_ref_local.setZero();
    
    getJointVelocityDerivatives(model,data_v,jointId,WORLD,
                                v_partial_dq_ref,v_partial_dv_ref);
                                
    BOOST_CHECK(v_partial_dq.isApprox(v_partial_dq_ref));
    BOOST_CHECK(a_partial_da.isApprox(v_partial_dv_ref));
    
    getJointVelocityDerivatives(model,data_v,jointId,LOCAL_WORLD_ALIGNED,
                                v_partial_dq_ref_local_world_aligned,v_partial_dv_ref_local_world_aligned);
                                                            
    BOOST_CHECK(v_partial_dq_local_world_aligned.isApprox(v_partial_dq_ref_local_world_aligned));
    BOOST_CHECK(a_partial_da_local_world_aligned.isApprox(v_partial_dv_ref_local_world_aligned));
    
    getJointVelocityDerivatives(model,data_v,jointId,LOCAL,
                                v_partial_dq_ref_local,v_partial_dv_ref_local);
    
    BOOST_CHECK(v_partial_dq_local.isApprox(v_partial_dq_ref_local));
    BOOST_CHECK(a_partial_da_local.isApprox(v_partial_dv_ref_local));
  }
  
  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  Data::Matrix6x J_ref_local_world_aligned(6,model.nv); J_ref_local_world_aligned.setZero();
  computeJointJacobians(model,data_ref,q);
  getJointJacobian(model,data_ref,jointId,WORLD,J_ref);
  getJointJacobian(model,data_ref,jointId,LOCAL_WORLD_ALIGNED,J_ref_local_world_aligned);
  getJointJacobian(model,data_ref,jointId,LOCAL,J_ref_local);
  
  BOOST_CHECK(a_partial_da.isApprox(J_ref));
  BOOST_CHECK(a_partial_da_local_world_aligned.isApprox(J_ref_local_world_aligned));
  BOOST_CHECK(a_partial_da_local.isApprox(J_ref_local));
  
  // Check against finite differences
  Data::Matrix6x a_partial_da_fd(6,model.nv); a_partial_da_fd.setZero();
  Data::Matrix6x a_partial_da_fd_local_world_aligned(6,model.nv); a_partial_da_fd_local_world_aligned.setZero();
  Data::Matrix6x a_partial_da_fd_local(6,model.nv); a_partial_da_fd_local.setZero();
  const double alpha = 1e-8;
  
  Eigen::VectorXd v_plus(v), a_plus(a);
  Data data_plus(model);
  forwardKinematics(model,data_ref,q,v,a);
  SE3 oMi_rot(SE3::Identity());
  oMi_rot.rotation() = data_ref.oMi[jointId].rotation();
  
  // dacc/da
  Motion a0(data_ref.oMi[jointId].act(data_ref.a[jointId]));
  Motion a0_local_world_aligned(oMi_rot.act(data_ref.a[jointId]));
  Motion a0_local(data_ref.a[jointId]);
  for(int k = 0; k < model.nv; ++k)
  {
    a_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v,a_plus);
    
    a_partial_da_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;
    a_partial_da_fd_local_world_aligned.col(k) = (oMi_rot.act(data_plus.a[jointId]) - a0_local_world_aligned).toVector()/alpha;
    a_partial_da_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    a_plus[k] -= alpha;
  }
  BOOST_CHECK(a_partial_da.isApprox(a_partial_da_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_da_local_world_aligned.isApprox(a_partial_da_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(a_partial_da_local.isApprox(a_partial_da_fd_local,sqrt(alpha)));
  motionSet::se3Action(data_ref.oMi[jointId].inverse(),a_partial_da,a_partial_da_local);
  BOOST_CHECK(a_partial_da_local.isApprox(a_partial_da_fd_local,sqrt(alpha)));
  
  // dacc/dv
  Data::Matrix6x a_partial_dv_fd(6,model.nv); a_partial_dv_fd.setZero();
  Data::Matrix6x a_partial_dv_fd_local_world_aligned(6,model.nv); a_partial_dv_fd_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dv_fd_local(6,model.nv); a_partial_dv_fd_local.setZero();
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus,a);
    
    a_partial_dv_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;    a_partial_dv_fd_local_world_aligned.col(k) = (oMi_rot.act(data_plus.a[jointId]) - a0_local_world_aligned).toVector()/alpha;
    a_partial_dv_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(a_partial_dv.isApprox(a_partial_dv_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_dv_local_world_aligned.isApprox(a_partial_dv_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(a_partial_dv_local.isApprox(a_partial_dv_fd_local,sqrt(alpha)));
  motionSet::se3Action(data_ref.oMi[jointId].inverse(),a_partial_dv,a_partial_dv_local);
  BOOST_CHECK(a_partial_dv_local.isApprox(a_partial_dv_fd_local,sqrt(alpha)));
  
  // dacc/dq
  a_partial_dq.setZero();
  a_partial_dv.setZero();
  a_partial_da.setZero();
  
  a_partial_dq_local_world_aligned.setZero();
  a_partial_dv_local_world_aligned.setZero();
  a_partial_da_local_world_aligned.setZero();
  
  a_partial_dq_local.setZero();
  a_partial_dv_local.setZero();
  a_partial_da_local.setZero();
  
  Data::Matrix6x a_partial_dq_fd(6,model.nv); a_partial_dq_fd.setZero();
  Data::Matrix6x a_partial_dq_fd_local_world_aligned(6,model.nv); a_partial_dq_fd_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dq_fd_local(6,model.nv); a_partial_dq_fd_local.setZero();
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  getJointAccelerationDerivatives(model,data,jointId,WORLD,
                                  v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);
                                  
  getJointAccelerationDerivatives(model,data,jointId,LOCAL_WORLD_ALIGNED,
                                  v_partial_dq_local_world_aligned,
                                  a_partial_dq_local_world_aligned,
                                  a_partial_dv_local_world_aligned,
                                  a_partial_da_local_world_aligned);
  
  getJointAccelerationDerivatives(model,data,jointId,LOCAL,
                                  v_partial_dq_local,
                                  a_partial_dq_local,a_partial_dv_local,a_partial_da_local);
  
  Eigen::VectorXd q_plus(q), v_eps(Eigen::VectorXd::Zero(model.nv));
  forwardKinematics(model,data_ref,q,v,a);
  a0 = data_ref.oMi[jointId].act(data_ref.a[jointId]);
  oMi_rot.rotation() = data_ref.oMi[jointId].rotation();
  a0_local_world_aligned = oMi_rot.act(data_ref.a[jointId]);
  a0_local = data_ref.a[jointId];
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_plus,q_plus,v,a);
    
    SE3 oMi_plus_rot = data_plus.oMi[jointId];
    oMi_plus_rot.translation().setZero();
    
    Motion a_plus_local_world_aligned = oMi_plus_rot.act(data_plus.a[jointId]);
    const SE3::Vector3 trans = data_plus.oMi[jointId].translation() - data_ref.oMi[jointId].translation();
    a_plus_local_world_aligned.linear() -= a_plus_local_world_aligned.angular().cross(trans);
    a_partial_dq_fd.col(k) = (data_plus.oMi[jointId].act(data_plus.a[jointId]) - a0).toVector()/alpha;
    a_partial_dq_fd_local_world_aligned.col(k) = (a_plus_local_world_aligned - a0_local_world_aligned).toVector()/alpha;
    a_partial_dq_fd_local.col(k) = (data_plus.a[jointId] - a0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }
  
  BOOST_CHECK(a_partial_dq.isApprox(a_partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_dq_local_world_aligned.isApprox(a_partial_dq_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(a_partial_dq_local.isApprox(a_partial_dq_fd_local,sqrt(alpha)));
 
}

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_against_classic_formula)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model,true);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  const Model::JointIndex jointId = model.existJointName("rarm4_joint")?model.getJointId("rarm4_joint"):(Model::Index)(model.njoints-1);
  Data::Matrix6x v_partial_dq(6,model.nv); v_partial_dq.setZero();
  Data::Matrix6x v_partial_dq_ref(6,model.nv); v_partial_dq_ref.setZero();
  Data::Matrix6x v_partial_dv_ref(6,model.nv); v_partial_dv_ref.setZero();
  Data::Matrix6x a_partial_dq(6,model.nv); a_partial_dq.setZero();
  Data::Matrix6x a_partial_dv(6,model.nv); a_partial_dv.setZero();
  Data::Matrix6x a_partial_da(6,model.nv); a_partial_da.setZero();
  
  // LOCAL: da/dv == dJ/dt + dv/dq
//  {
//    Data::Matrix6x rhs(6,model.nv); rhs.setZero();
//    
//    v_partial_dq.setZero();
//    a_partial_dq.setZero();
//    a_partial_dv.setZero();
//    a_partial_da.setZero();
//    
//    computeForwardKinematicsDerivatives(model,data_ref,q,v,a);
//    computeForwardKinematicsDerivatives(model,data,q,v,a);
//    
//    getJointAccelerationDerivatives<LOCAL>(model,data,jointId,
//                                           v_partial_dq,
//                                           a_partial_dq,a_partial_dv,a_partial_da);
//    
//    getJointJacobianTimeVariation<LOCAL>(model,data_ref,jointId,rhs);
//    
//    v_partial_dq_ref.setZero(); v_partial_dv_ref.setZero();
//    getJointVelocityDerivatives<LOCAL>(model,data_ref,jointId,
//                                       v_partial_dq_ref,v_partial_dv_ref);
//    rhs += v_partial_dq_ref;
//    BOOST_CHECK(a_partial_dv.isApprox(rhs,1e-12));
//    
//    std::cout << "a_partial_dv\n" << a_partial_dv << std::endl;
//    std::cout << "rhs\n" << rhs << std::endl;
//  }
  
  // WORLD: da/dv == dJ/dt + dv/dq
  {
    Data::Matrix6x rhs(6,model.nv); rhs.setZero();
    
    v_partial_dq.setZero();
    a_partial_dq.setZero();
    a_partial_dv.setZero();
    a_partial_da.setZero();
    
    computeForwardKinematicsDerivatives(model,data_ref,q,v,a);
    computeForwardKinematicsDerivatives(model,data,q,v,a);
    
    getJointAccelerationDerivatives(model,data,jointId,WORLD,
                                    v_partial_dq,
                                    a_partial_dq,a_partial_dv,a_partial_da);
    
    getJointJacobianTimeVariation(model,data_ref,jointId,WORLD,rhs);
    
    v_partial_dq_ref.setZero(); v_partial_dv_ref.setZero();
    getJointVelocityDerivatives(model,data_ref,jointId,WORLD,
                                v_partial_dq_ref,v_partial_dv_ref);
    rhs += v_partial_dq_ref;
    BOOST_CHECK(a_partial_dv.isApprox(rhs,1e-12));
    
//    std::cout << "a_partial_dv\n" << a_partial_dv << std::endl;
//    std::cout << "rhs\n" << rhs << std::endl;
  }
  
//  // WORLD: da/dq == d/dt(dv/dq)
//  {
//    const double alpha = 1e-8;
//    Eigen::VectorXd q_plus(model.nq), v_plus(model.nv);
//
//    Data data_plus(model);
//    v_plus = v + alpha * a;
//    q_plus = integrate(model,q,alpha*v);
//
//    computeForwardKinematicsDerivatives(model,data_plus,q_plus,v_plus,a);
//    computeForwardKinematicsDerivatives(model,data_ref,q,v,a);
//
//    Data::Matrix6x v_partial_dq_plus(6,model.nv); v_partial_dq_plus.setZero();
//    Data::Matrix6x v_partial_dv_plus(6,model.nv); v_partial_dv_plus.setZero();
//
//    v_partial_dq_ref.setZero(); v_partial_dv_ref.setZero();
//
//    v_partial_dq.setZero();
//    a_partial_dq.setZero();
//    a_partial_dv.setZero();
//    a_partial_da.setZero();
//
//    getJointVelocityDerivatives<WORLD>(model,data_ref,jointId,
//                                       v_partial_dq_ref,v_partial_dv_ref);
//    getJointVelocityDerivatives<WORLD>(model,data_plus,jointId,
//                                       v_partial_dq_plus,v_partial_dv_plus);
//
//    getJointAccelerationDerivatives<WORLD>(model,data_ref,jointId,
//                                           v_partial_dq,
//                                           a_partial_dq,a_partial_dv,a_partial_da);
//
//    Data::Matrix6x a_partial_dq_fd(6,model.nv); a_partial_dq_fd.setZero();
//    {
//      Data data_fd(model);
//      VectorXd q_fd(model.nq), v_eps(model.nv); v_eps.setZero();
//      for(int k = 0; k < model.nv; ++k)
//      {
//        v_eps[k] += alpha;
//        q_fd = integrate(model,q,v_eps);
//        forwardKinematics(model,data_fd,q_fd,v,a);
//        a_partial_dq_fd.col(k) = (data_fd.oMi[jointId].act(data_fd.a[jointId]) - data_ref.oa[jointId]).toVector()/alpha;
//        v_eps[k] = 0.;
//      }
//    }
//
//    Data::Matrix6x rhs = (v_partial_dq_plus - v_partial_dq_ref)/alpha;
//    BOOST_CHECK(a_partial_dq.isApprox(rhs,sqrt(alpha)));
//
//    std::cout << "a_partial_dq\n" << a_partial_dq << std::endl;
//    std::cout << "a_partial_dq_fd\n" << a_partial_dq_fd << std::endl;
//    std::cout << "rhs\n" << rhs << std::endl;
//  }
  
  // LOCAL: da/dq == d/dt(dv/dq)
  {
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq), v_plus(model.nv);
    
    Data data_plus(model);
    v_plus = v + alpha * a;
    q_plus = integrate(model,q,alpha*v);
    
    computeForwardKinematicsDerivatives(model,data_plus,q_plus,v_plus,a);
    computeForwardKinematicsDerivatives(model,data_ref,q,v,a);
    
    Data::Matrix6x v_partial_dq_plus(6,model.nv); v_partial_dq_plus.setZero();
    Data::Matrix6x v_partial_dv_plus(6,model.nv); v_partial_dv_plus.setZero();
    
    v_partial_dq_ref.setZero(); v_partial_dv_ref.setZero();
    
    v_partial_dq.setZero();
    a_partial_dq.setZero();
    a_partial_dv.setZero();
    a_partial_da.setZero();
    
    getJointVelocityDerivatives(model,data_ref,jointId,LOCAL,
                                       v_partial_dq_ref,v_partial_dv_ref);
    getJointVelocityDerivatives(model,data_plus,jointId,LOCAL,
                                v_partial_dq_plus,v_partial_dv_plus);
    
    getJointAccelerationDerivatives(model,data_ref,jointId,LOCAL,
                                    v_partial_dq,
                                    a_partial_dq,a_partial_dv,a_partial_da);
    
    Data::Matrix6x rhs = (v_partial_dq_plus - v_partial_dq_ref)/alpha;
    BOOST_CHECK(a_partial_dq.isApprox(rhs,sqrt(alpha)));
    
//    std::cout << "a_partial_dq\n" << a_partial_dq << std::endl;
//    std::cout << "rhs\n" << rhs << std::endl;
  }
  
  
}

BOOST_AUTO_TEST_CASE(test_kinematics_hessians)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model,true);
  
  Data data(model), data_ref(model), data_plus(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  const Model::JointIndex joint_id = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  
  computeJointJacobians(model,data,q);
  computeJointKinematicHessians(model,data);
  
  Data data2(model);
  computeJointKinematicHessians(model,data2,q);
  BOOST_CHECK(data2.J.isApprox(data.J));
  
  const Eigen::DenseIndex matrix_offset = 6 * model.nv;
  
  for(int k = 0; k < model.nv; ++k)
  {
    Eigen::Map<Data::Matrix6x> dJ(data.kinematic_hessians.data() + k*matrix_offset,6,model.nv);
    Eigen::Map<Data::Matrix6x> dJ2(data2.kinematic_hessians.data() + k*matrix_offset,6,model.nv);
    
    BOOST_CHECK(dJ2.isApprox(dJ));
  }
  
  for(int i = 0; i < model.nv; ++i)
  {
    for(int j = i; j < model.nv; ++j)
    {
      bool j_is_children_of_i = false;
      for(int parent = j; parent >= 0; parent = data.parents_fromRow[(size_t)parent])
      {
        if(parent == i)
        {
          j_is_children_of_i = true;
          break;
        }
      }
      
      if(j_is_children_of_i)
      {
        if(i==j)
        {
          Eigen::Map<Data::Motion::Vector6> SixSi(  data.kinematic_hessians.data()
                                                  + i * matrix_offset
                                                  + i * 6);
          BOOST_CHECK(SixSi.isZero());
        }
        else
        {
          Eigen::Map<Data::Motion::Vector6> SixSj(  data.kinematic_hessians.data()
                                                  + i * matrix_offset
                                                  + j * 6);
          
          Eigen::Map<Data::Motion::Vector6> SjxSi(  data.kinematic_hessians.data()
                                                  + j * matrix_offset
                                                  + i * 6);
          
          BOOST_CHECK(SixSj.isApprox(-SjxSi));
        }
      }
      else
      {
        Eigen::Map<Data::Motion::Vector6> SixSj(  data.kinematic_hessians.data()
                                                + i * matrix_offset
                                                + j * 6);
        
        Eigen::Map<Data::Motion::Vector6> SjxSi(  data.kinematic_hessians.data()
                                                + j * matrix_offset
                                                + i * 6);
        
        BOOST_CHECK(SixSj.isZero());
        BOOST_CHECK(SjxSi.isZero());
      }
    }
  }
  
  const double eps = 1e-8;
  Data::Matrix6x J_ref(6,model.nv), J_plus(6,model.nv);
  J_ref.setZero(); J_plus.setZero();
  
  computeJointJacobians(model,data_ref,q);
  VectorXd v_plus(VectorXd::Zero(model.nv));
  
  const Eigen::DenseIndex outer_offset = model.nv * 6;
  
  // WORLD
  getJointJacobian(model,data_ref,joint_id,WORLD,J_ref);
  Data::Tensor3x kinematic_hessian_world = getJointKinematicHessian(model,data,joint_id,WORLD);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_plus[k] = eps;
    const VectorXd q_plus = integrate(model,q,v_plus);
    computeJointJacobians(model,data_plus,q_plus);
    J_plus.setZero();
    getJointJacobian(model,data_plus,joint_id,WORLD,J_plus);
    
    Data::Matrix6x dJ_dq_ref = (J_plus - J_ref)/eps;
    Eigen::Map<Data::Matrix6x> dJ_dq(kinematic_hessian_world.data() + k * outer_offset,6,model.nv);
    
//    std::cout << "k: " << k << std::endl;
//    std::cout << "dJ_dq:\n" << dJ_dq << std::endl;
//    std::cout << "dJ_dq_ref:\n" << dJ_dq_ref << std::endl;
    BOOST_CHECK((dJ_dq_ref-dJ_dq).isZero(sqrt(eps)));
    v_plus[k] = 0.;
  }
    
  // LOCAL_WORLD_ALIGNED
  computeJointJacobians(model,data_ref,q);
  getJointJacobian(model,data_ref,joint_id,LOCAL_WORLD_ALIGNED,J_ref);
  Data::Tensor3x kinematic_hessian_local_world_aligned = getJointKinematicHessian(model,data,joint_id,LOCAL_WORLD_ALIGNED);
  Data::Matrix3x dt_last_fd(3,model.nv);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_plus[k] = eps;
    const VectorXd q_plus = integrate(model,q,v_plus);
    computeJointJacobians(model,data_plus,q_plus);
    J_plus.setZero();
    getJointJacobian(model,data_plus,joint_id,LOCAL_WORLD_ALIGNED,J_plus);
    
    SE3 tMt_plus = data_ref.oMi[joint_id].inverse() * data_plus.oMi[joint_id];
    tMt_plus.rotation().setIdentity();
    
    dt_last_fd.col(k) = (data_plus.oMi[joint_id].translation() - data_ref.oMi[joint_id].translation())/eps;
    
    Data::Matrix6x dJ_dq_ref = (J_plus - J_ref)/eps;
    Eigen::Map<Data::Matrix6x> dJ_dq(kinematic_hessian_local_world_aligned.data() + k * outer_offset,6,model.nv);

//    std::cout << "k: " << k << std::endl;
//    std::cout << "dJ_dq:\n" << dJ_dq << std::endl;
//    std::cout << "dJ_dq_ref:\n" << dJ_dq_ref << std::endl;
    BOOST_CHECK((dJ_dq_ref-dJ_dq).isZero(sqrt(eps)));
    v_plus[k] = 0.;
  }
  
  Data::Matrix6x J_world(6,model.nv); J_world.setZero();
  getJointJacobian(model,data_ref,joint_id,LOCAL_WORLD_ALIGNED,J_world);
  
  BOOST_CHECK(dt_last_fd.isApprox(J_world.topRows<3>(),sqrt(eps)));
  
  // LOCAL
  computeJointJacobians(model,data_ref,q);
  getJointJacobian(model,data_ref,joint_id,LOCAL,J_ref);
  Data::Tensor3x kinematic_hessian_local = getJointKinematicHessian(model,data,joint_id,LOCAL);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_plus[k] = eps;
    const VectorXd q_plus = integrate(model,q,v_plus);
    computeJointJacobians(model,data_plus,q_plus);
    J_plus.setZero();
    getJointJacobian(model,data_plus,joint_id,LOCAL,J_plus);
    
//    const SE3 tMt_plus = data_ref.oMi[joint_id].inverse() * data_plus.oMi[joint_id];
 
    Data::Matrix6x dJ_dq_ref = (J_plus - J_ref)/eps;
    Eigen::Map<Data::Matrix6x> dJ_dq(kinematic_hessian_local.data() + k * outer_offset,6,model.nv);
    
//    std::cout << "k: " << k << std::endl;
//    std::cout << "dJ_dq:\n" << dJ_dq << std::endl;
//    std::cout << "dJ_dq_ref:\n" << dJ_dq_ref << std::endl;
    BOOST_CHECK((dJ_dq_ref-dJ_dq).isZero(sqrt(eps)));
    v_plus[k] = 0.;
  }
}

BOOST_AUTO_TEST_SUITE_END()
