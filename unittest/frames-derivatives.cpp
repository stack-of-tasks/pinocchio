//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_frames_derivatives_velocity)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  const Model::JointIndex jointId = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  Frame frame("rand",jointId,0,SE3::Random(),OP_FRAME);
  FrameIndex frameId = model.addFrame(frame);
  
  BOOST_CHECK(model.getFrameId("rand") == frameId);
  BOOST_CHECK(model.frames[frameId].parent == jointId);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  
  Data::Matrix6x partial_dq(6,model.nv); partial_dq.setZero();
  Data::Matrix6x partial_dq_local_world_aligned(6,model.nv); partial_dq_local_world_aligned.setZero();
  Data::Matrix6x partial_dq_local(6,model.nv); partial_dq_local.setZero();
  Data::Matrix6x partial_dv(6,model.nv); partial_dv.setZero();
  Data::Matrix6x partial_dv_local_world_aligned(6,model.nv); partial_dv_local_world_aligned.setZero();
  Data::Matrix6x partial_dv_local(6,model.nv); partial_dv_local.setZero();
  
  getFrameVelocityDerivatives(model,data,frameId,WORLD,
                              partial_dq,partial_dv);
  
  getFrameVelocityDerivatives(model,data,frameId,LOCAL_WORLD_ALIGNED,
                              partial_dq_local_world_aligned,partial_dv_local_world_aligned);
  
  getFrameVelocityDerivatives(model,data,frameId,LOCAL,
                              partial_dq_local,partial_dv_local);
  
  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local_world_aligned(6,model.nv); J_ref_local_world_aligned.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  computeJointJacobians(model,data_ref,q);
  getFrameJacobian(model,data_ref,frameId,WORLD,J_ref);
  getFrameJacobian(model,data_ref,frameId,LOCAL_WORLD_ALIGNED,J_ref_local_world_aligned);
  getFrameJacobian(model,data_ref,frameId,LOCAL,J_ref_local);
  
  BOOST_CHECK(data_ref.oMf[frameId].isApprox(data.oMf[frameId]));
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
  Motion v0 = getFrameVelocity(model,data,frameId,WORLD);
  Motion v0_local_world_aligned = getFrameVelocity(model,data,frameId,LOCAL_WORLD_ALIGNED);
  Motion v0_local = getFrameVelocity(model,data,frameId,LOCAL);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus);

    partial_dv_fd.col(k) = (getFrameVelocity(model,data_plus,frameId,WORLD) - v0).toVector()/alpha;
    partial_dv_fd_local_world_aligned.col(k) = (getFrameVelocity(model,data_plus,frameId,LOCAL_WORLD_ALIGNED) - v0_local_world_aligned).toVector()/alpha;
    partial_dv_fd_local.col(k) = (getFrameVelocity(model,data_plus,frameId,LOCAL) - v0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(partial_dv.isApprox(partial_dv_fd,sqrt(alpha)));
  BOOST_CHECK(partial_dv_local_world_aligned.isApprox(partial_dv_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(partial_dv_local.isApprox(partial_dv_fd_local,sqrt(alpha)));

  // dvel/dq
  Eigen::VectorXd q_plus(q), v_eps(Eigen::VectorXd::Zero(model.nv));
  forwardKinematics(model,data_ref,q,v);
  updateFramePlacements(model,data_ref);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_plus,q_plus,v);
    updateFramePlacements(model,data_plus);

    SE3 oMi_plus_rot = data_plus.oMi[jointId];
    oMi_plus_rot.translation().setZero();

    Motion v_plus_local_world_aligned = getFrameVelocity(model,data_plus,frameId,LOCAL_WORLD_ALIGNED);
    SE3::Vector3 trans = data_plus.oMf[frameId].translation() - data_ref.oMf[frameId].translation();
    v_plus_local_world_aligned.linear() -= v_plus_local_world_aligned.angular().cross(trans);
    partial_dq_fd.col(k) = (getFrameVelocity(model,data_plus,frameId,WORLD) - v0).toVector()/alpha;
    partial_dq_fd_local_world_aligned.col(k) = (v_plus_local_world_aligned - v0_local_world_aligned).toVector()/alpha;
    partial_dq_fd_local.col(k) = (getFrameVelocity(model,data_plus,frameId,LOCAL) - v0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }

  BOOST_CHECK(partial_dq.isApprox(partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(partial_dq_local_world_aligned.isApprox(partial_dq_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(partial_dq_local.isApprox(partial_dq_fd_local,sqrt(alpha)));
  
}

BOOST_AUTO_TEST_SUITE_END()
