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

BOOST_AUTO_TEST_CASE(test_kinematics_derivatives_acceleration)
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
  
  getFrameAccelerationDerivatives(model,data,frameId,WORLD,
                                  v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);
                                  
  getFrameAccelerationDerivatives(model,data,frameId,LOCAL_WORLD_ALIGNED,
                                  v_partial_dq_local_world_aligned,
                                  a_partial_dq_local_world_aligned,
                                  a_partial_dv_local_world_aligned,
                                  a_partial_da_local_world_aligned);
  
  getFrameAccelerationDerivatives(model,data,frameId,LOCAL,
                                  v_partial_dq_local,
                                  a_partial_dq_local,a_partial_dv_local,a_partial_da_local);

  // Check v_partial_dq against getFrameVelocityDerivatives
  {
    Data data_v(model);
    computeForwardKinematicsDerivatives(model,data_v,q,v,a);

    Data::Matrix6x v_partial_dq_ref(6,model.nv); v_partial_dq_ref.setZero();
    Data::Matrix6x v_partial_dq_ref_local_world_aligned(6,model.nv); v_partial_dq_ref_local_world_aligned.setZero();
    Data::Matrix6x v_partial_dq_ref_local(6,model.nv); v_partial_dq_ref_local.setZero();
    Data::Matrix6x v_partial_dv_ref(6,model.nv); v_partial_dv_ref.setZero();
    Data::Matrix6x v_partial_dv_ref_local_world_aligned(6,model.nv); v_partial_dv_ref_local_world_aligned.setZero();
    Data::Matrix6x v_partial_dv_ref_local(6,model.nv); v_partial_dv_ref_local.setZero();

    getFrameVelocityDerivatives(model,data_v,frameId,WORLD,
                                v_partial_dq_ref,v_partial_dv_ref);

    BOOST_CHECK(v_partial_dq.isApprox(v_partial_dq_ref));
    BOOST_CHECK(a_partial_da.isApprox(v_partial_dv_ref));

    getFrameVelocityDerivatives(model,data_v,frameId,LOCAL_WORLD_ALIGNED,
                                v_partial_dq_ref_local_world_aligned,v_partial_dv_ref_local_world_aligned);

    BOOST_CHECK(v_partial_dq_local_world_aligned.isApprox(v_partial_dq_ref_local_world_aligned));
    BOOST_CHECK(a_partial_da_local_world_aligned.isApprox(v_partial_dv_ref_local_world_aligned));

    getFrameVelocityDerivatives(model,data_v,frameId,LOCAL,
                                v_partial_dq_ref_local,v_partial_dv_ref_local);

    BOOST_CHECK(v_partial_dq_local.isApprox(v_partial_dq_ref_local));
    BOOST_CHECK(a_partial_da_local.isApprox(v_partial_dv_ref_local));
  }

  Data::Matrix6x J_ref(6,model.nv); J_ref.setZero();
  Data::Matrix6x J_ref_local(6,model.nv); J_ref_local.setZero();
  Data::Matrix6x J_ref_local_world_aligned(6,model.nv); J_ref_local_world_aligned.setZero();
  computeJointJacobians(model,data_ref,q);
  getFrameJacobian(model,data_ref,frameId,WORLD,J_ref);
  getFrameJacobian(model,data_ref,frameId,LOCAL_WORLD_ALIGNED,J_ref_local_world_aligned);
  getFrameJacobian(model,data_ref,frameId,LOCAL,J_ref_local);

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

  // dacc/da
  Motion a0 = getFrameAcceleration(model,data,frameId,WORLD);
  Motion a0_local_world_aligned = getFrameAcceleration(model,data,frameId,LOCAL_WORLD_ALIGNED);
  Motion a0_local = getFrameAcceleration(model,data,frameId,LOCAL);
  for(int k = 0; k < model.nv; ++k)
  {
    a_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v,a_plus);

    a_partial_da_fd.col(k) = (getFrameAcceleration(model,data_plus,frameId,WORLD) - a0).toVector()/alpha;
    a_partial_da_fd_local_world_aligned.col(k) = (getFrameAcceleration(model,data_plus,frameId,LOCAL_WORLD_ALIGNED) - a0_local_world_aligned).toVector()/alpha;
    a_partial_da_fd_local.col(k) = (getFrameAcceleration(model,data_plus,frameId,LOCAL) - a0_local).toVector()/alpha;
    a_plus[k] -= alpha;
  }
  BOOST_CHECK(a_partial_da.isApprox(a_partial_da_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_da_local_world_aligned.isApprox(a_partial_da_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(a_partial_da_local.isApprox(a_partial_da_fd_local,sqrt(alpha)));

  // dacc/dv
  Data::Matrix6x a_partial_dv_fd(6,model.nv); a_partial_dv_fd.setZero();
  Data::Matrix6x a_partial_dv_fd_local_world_aligned(6,model.nv); a_partial_dv_fd_local_world_aligned.setZero();
  Data::Matrix6x a_partial_dv_fd_local(6,model.nv); a_partial_dv_fd_local.setZero();
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    forwardKinematics(model,data_plus,q,v_plus,a);

    a_partial_dv_fd.col(k) = (getFrameAcceleration(model,data_plus,frameId,WORLD) - a0).toVector()/alpha;    a_partial_dv_fd_local_world_aligned.col(k) = (getFrameAcceleration(model,data_plus,frameId,LOCAL_WORLD_ALIGNED) - a0_local_world_aligned).toVector()/alpha;
    a_partial_dv_fd_local.col(k) = (getFrameAcceleration(model,data_plus,frameId,LOCAL) - a0_local).toVector()/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(a_partial_dv.isApprox(a_partial_dv_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_dv_local_world_aligned.isApprox(a_partial_dv_fd_local_world_aligned,sqrt(alpha)));
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
  getFrameAccelerationDerivatives(model,data,frameId,WORLD,
                                  v_partial_dq,
                                  a_partial_dq,a_partial_dv,a_partial_da);

  getFrameAccelerationDerivatives(model,data,frameId,LOCAL_WORLD_ALIGNED,
                                  v_partial_dq_local_world_aligned,
                                  a_partial_dq_local_world_aligned,
                                  a_partial_dv_local_world_aligned,
                                  a_partial_da_local_world_aligned);

  getFrameAccelerationDerivatives(model,data,frameId,LOCAL,
                                  v_partial_dq_local,
                                  a_partial_dq_local,a_partial_dv_local,a_partial_da_local);

  Eigen::VectorXd q_plus(q), v_eps(Eigen::VectorXd::Zero(model.nv));
  forwardKinematics(model,data_ref,q,v,a);
  updateFramePlacements(model,data_ref);
  a0 = getFrameAcceleration(model,data,frameId,WORLD);
  a0_local_world_aligned = getFrameAcceleration(model,data,frameId,LOCAL_WORLD_ALIGNED);
  a0_local = getFrameAcceleration(model,data,frameId,LOCAL);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_plus,q_plus,v,a);
    updateFramePlacements(model,data_plus);

    a_partial_dq_fd.col(k) = (getFrameAcceleration(model,data_plus,frameId,WORLD) - a0).toVector()/alpha;
    Motion a_plus_local_world_aligned = getFrameAcceleration(model,data_plus,frameId,LOCAL_WORLD_ALIGNED);
    const SE3::Vector3 trans = data_plus.oMf[frameId].translation() - data_ref.oMf[frameId].translation();
    a_plus_local_world_aligned.linear() -= a_plus_local_world_aligned.angular().cross(trans);
    a_partial_dq_fd_local_world_aligned.col(k) = (a_plus_local_world_aligned - a0_local_world_aligned).toVector()/alpha;
    a_partial_dq_fd_local.col(k) = (getFrameAcceleration(model,data_plus,frameId,LOCAL) - a0_local).toVector()/alpha;
    v_eps[k] -= alpha;
  }

  BOOST_CHECK(a_partial_dq.isApprox(a_partial_dq_fd,sqrt(alpha)));
  BOOST_CHECK(a_partial_dq_local_world_aligned.isApprox(a_partial_dq_fd_local_world_aligned,sqrt(alpha)));
  BOOST_CHECK(a_partial_dq_local.isApprox(a_partial_dq_fd_local,sqrt(alpha)));
  
  // Test other signatures
  Data::Matrix6x v_partial_dq_other(6,model.nv); v_partial_dq_other.setZero();
  Data::Matrix6x v_partial_dq_local_other(6,model.nv); v_partial_dq_local_other.setZero();
  Data::Matrix6x v_partial_dq_local_world_aligned_other(6,model.nv); v_partial_dq_local_world_aligned_other.setZero();
  Data::Matrix6x v_partial_dv_other(6,model.nv); v_partial_dv_other.setZero();
  Data::Matrix6x v_partial_dv_local_other(6,model.nv); v_partial_dv_local_other.setZero();
  Data::Matrix6x v_partial_dv_local_world_aligned_other(6,model.nv); v_partial_dv_local_world_aligned_other.setZero();
  Data::Matrix6x a_partial_dq_other(6,model.nv); a_partial_dq_other.setZero();
  Data::Matrix6x a_partial_dq_local_world_aligned_other(6,model.nv); a_partial_dq_local_world_aligned_other.setZero();
  Data::Matrix6x a_partial_dq_local_other(6,model.nv); a_partial_dq_local_other.setZero();
  Data::Matrix6x a_partial_dv_other(6,model.nv); a_partial_dv_other.setZero();
  Data::Matrix6x a_partial_dv_local_world_aligned_other(6,model.nv); a_partial_dv_local_world_aligned_other.setZero();
  Data::Matrix6x a_partial_dv_local_other(6,model.nv); a_partial_dv_local_other.setZero();
  Data::Matrix6x a_partial_da_other(6,model.nv); a_partial_da_other.setZero();
  Data::Matrix6x a_partial_da_local_world_aligned_other(6,model.nv); a_partial_da_local_world_aligned_other.setZero();
  Data::Matrix6x a_partial_da_local_other(6,model.nv); a_partial_da_local_other.setZero();
 
  getFrameAccelerationDerivatives(model,data,frameId,WORLD,
                                  v_partial_dq_other,
                                  v_partial_dv_other,
                                  a_partial_dq_other,
                                  a_partial_dv_other,
                                  a_partial_da_other);
  
  BOOST_CHECK(v_partial_dq_other.isApprox(v_partial_dq));
  BOOST_CHECK(v_partial_dv_other.isApprox(a_partial_da));
  BOOST_CHECK(a_partial_dq_other.isApprox(a_partial_dq));
  BOOST_CHECK(a_partial_dv_other.isApprox(a_partial_dv));
  BOOST_CHECK(a_partial_da_other.isApprox(a_partial_da));
                                  
  getFrameAccelerationDerivatives(model,data,frameId,LOCAL_WORLD_ALIGNED,
                                  v_partial_dq_local_world_aligned_other,
                                  v_partial_dv_local_world_aligned_other,
                                  a_partial_dq_local_world_aligned_other,
                                  a_partial_dv_local_world_aligned_other,
                                  a_partial_da_local_world_aligned_other);
                                  
  BOOST_CHECK(v_partial_dq_local_world_aligned_other.isApprox(v_partial_dq_local_world_aligned));
  BOOST_CHECK(v_partial_dv_local_world_aligned_other.isApprox(a_partial_da_local_world_aligned));
  BOOST_CHECK(a_partial_dq_local_world_aligned_other.isApprox(a_partial_dq_local_world_aligned));
  BOOST_CHECK(a_partial_dv_local_world_aligned_other.isApprox(a_partial_dv_local_world_aligned));
  BOOST_CHECK(a_partial_da_local_world_aligned_other.isApprox(a_partial_da_local_world_aligned));
                                  
  getFrameAccelerationDerivatives(model,data,frameId,LOCAL,
                                  v_partial_dq_local_other,
                                  v_partial_dv_local_other,
                                  a_partial_dq_local_other,
                                  a_partial_dv_local_other,
                                  a_partial_da_local_other);
                                  
  BOOST_CHECK(v_partial_dq_local_other.isApprox(v_partial_dq_local));
  BOOST_CHECK(v_partial_dv_local_other.isApprox(a_partial_da_local));
  BOOST_CHECK(a_partial_dq_local_other.isApprox(a_partial_dq_local));
  BOOST_CHECK(a_partial_dv_local_other.isApprox(a_partial_dv_local));
  BOOST_CHECK(a_partial_da_local_other.isApprox(a_partial_da_local));
}

BOOST_AUTO_TEST_SUITE_END()
