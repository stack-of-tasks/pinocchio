//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
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

BOOST_AUTO_TEST_CASE(frame_basic)
{
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model);
  
  BOOST_CHECK(model.frames.size() >= size_t(model.njoints));
  for(Model::FrameVector::const_iterator it = model.frames.begin();
      it != model.frames.end(); ++it)
  {
    const Frame & frame = *it;
    BOOST_CHECK(frame == frame);
    Frame frame_copy(frame);
    BOOST_CHECK(frame_copy == frame);
  }
  
  std::ostringstream os;
  os << Frame("toto",0,0,SE3::Random(),OP_FRAME) << std::endl;
  BOOST_CHECK(!os.str().empty());
}

BOOST_AUTO_TEST_CASE(cast)
{
  using namespace pinocchio;
  Frame frame("toto",0,0,SE3::Random(),OP_FRAME);
  
  BOOST_CHECK(frame.cast<double>() == frame);
  BOOST_CHECK(frame.cast<double>().cast<long double>() == frame.cast<long double>());
}

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  framesForwardKinematics(model, data, q);

  BOOST_CHECK(data.oMf[model.getFrameId(frame_name)].isApprox(data.oMi[parent_idx]*framePlacement));

}

BOOST_AUTO_TEST_CASE ( test_update_placements )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  Model::FrameIndex frame_idx = model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();

  forwardKinematics(model, data, q);
  updateFramePlacements(model, data);

  framesForwardKinematics(model, data_ref, q);

  BOOST_CHECK(data.oMf[frame_idx].isApprox(data_ref.oMf[frame_idx]));
}

BOOST_AUTO_TEST_CASE ( test_update_single_placement )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  Model::FrameIndex frame_idx = model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();

  forwardKinematics(model, data, q);
  updateFramePlacement(model, data, frame_idx);

  framesForwardKinematics(model, data_ref, q);

  BOOST_CHECK(data.oMf[frame_idx].isApprox(data_ref.oMf[frame_idx]));
}

BOOST_AUTO_TEST_CASE ( test_velocity )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  Model::FrameIndex frame_idx = model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  forwardKinematics(model, data, q, v);

  Motion vf = getFrameVelocity(model, data, frame_idx);

  BOOST_CHECK(vf.isApprox(framePlacement.actInv(data.v[parent_idx])));

  pinocchio::Data data_ref(model);
  forwardKinematics(model, data_ref, q, v);
  updateFramePlacements(model, data_ref);
  Motion v_ref = getFrameVelocity(model, data_ref, frame_idx);

  BOOST_CHECK(v_ref.isApprox(getFrameVelocity(model,data,frame_idx)));
  BOOST_CHECK(v_ref.isApprox(getFrameVelocity(model,data,frame_idx,LOCAL)));
  BOOST_CHECK(data_ref.oMf[frame_idx].act(v_ref).isApprox(getFrameVelocity(model,data,frame_idx,WORLD)));
  BOOST_CHECK(SE3(data_ref.oMf[frame_idx].rotation(), Eigen::Vector3d::Zero()).act(v_ref).isApprox(getFrameVelocity(model,data,frame_idx,LOCAL_WORLD_ALIGNED)));
}

BOOST_AUTO_TEST_CASE ( test_acceleration )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  Model::FrameIndex frame_idx = model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  forwardKinematics(model, data, q, v, a);

  Motion af = getFrameAcceleration(model, data, frame_idx);

  BOOST_CHECK(af.isApprox(framePlacement.actInv(data.a[parent_idx])));

  pinocchio::Data data_ref(model);
  forwardKinematics(model, data_ref, q, v, a);
  updateFramePlacements(model, data_ref);
  Motion a_ref = getFrameAcceleration(model, data_ref, frame_idx);

  BOOST_CHECK(a_ref.isApprox(getFrameAcceleration(model,data,frame_idx)));
  BOOST_CHECK(a_ref.isApprox(getFrameAcceleration(model,data,frame_idx,LOCAL)));
  BOOST_CHECK(data_ref.oMf[frame_idx].act(a_ref).isApprox(getFrameAcceleration(model,data,frame_idx,WORLD)));
  BOOST_CHECK(SE3(data_ref.oMf[frame_idx].rotation(), Eigen::Vector3d::Zero()).act(a_ref).isApprox(getFrameAcceleration(model,data,frame_idx,LOCAL_WORLD_ALIGNED)));
}

BOOST_AUTO_TEST_CASE ( test_classic_acceleration )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  Model::FrameIndex frame_idx = model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  pinocchio::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  forwardKinematics(model, data, q, v, a);

  Motion vel = framePlacement.actInv(data.v[parent_idx]);
  Motion acc = framePlacement.actInv(data.a[parent_idx]);
  Vector3d linear;

  Motion acc_classical_local = acc;
  linear = acc.linear() + vel.angular().cross(vel.linear());
  acc_classical_local.linear() = linear;

  Motion af = getFrameClassicalAcceleration(model, data, frame_idx);

  BOOST_CHECK(af.isApprox(acc_classical_local));

  pinocchio::Data data_ref(model);
  forwardKinematics(model, data_ref, q, v, a);
  updateFramePlacements(model, data_ref);

  SE3 T_ref = data_ref.oMf[frame_idx];
  Motion v_ref = getFrameVelocity(model, data_ref, frame_idx);
  Motion a_ref = getFrameAcceleration(model, data_ref, frame_idx);

  Motion acc_classical_local_ref = a_ref;
  linear = a_ref.linear() + v_ref.angular().cross(v_ref.linear());
  acc_classical_local_ref.linear() = linear;

  BOOST_CHECK(acc_classical_local_ref.isApprox(getFrameClassicalAcceleration(model,data,frame_idx)));
  BOOST_CHECK(acc_classical_local_ref.isApprox(getFrameClassicalAcceleration(model,data,frame_idx,LOCAL)));

  Motion vel_world_ref = T_ref.act(v_ref);
  Motion acc_classical_world_ref = T_ref.act(a_ref);
  linear = acc_classical_world_ref.linear() + vel_world_ref.angular().cross(vel_world_ref.linear());
  acc_classical_world_ref.linear() = linear;

  BOOST_CHECK(acc_classical_world_ref.isApprox(getFrameClassicalAcceleration(model,data,frame_idx,WORLD)));

  Motion vel_aligned_ref = SE3(T_ref.rotation(), Eigen::Vector3d::Zero()).act(v_ref);
  Motion acc_classical_aligned_ref = SE3(T_ref.rotation(), Eigen::Vector3d::Zero()).act(a_ref);
  linear = acc_classical_aligned_ref.linear() + vel_aligned_ref.angular().cross(vel_aligned_ref.linear());
  acc_classical_aligned_ref.linear() = linear;

  BOOST_CHECK(acc_classical_aligned_ref.isApprox(getFrameClassicalAcceleration(model,data,frame_idx,LOCAL_WORLD_ALIGNED)));
}

BOOST_AUTO_TEST_CASE(test_frame_getters)
{
  using namespace Eigen;
  using namespace pinocchio;

  // Build a simple 1R planar model
  Model model;
  JointIndex parentId = model.addJoint(0, JointModelRZ(), SE3::Identity(), "Joint1");
  FrameIndex frameId = model.addFrame(Frame("Frame1", parentId, 0, SE3(Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)), OP_FRAME));

  Data data(model);

  // Predetermined configuration values
  VectorXd q(model.nq);
  q << M_PI/2;

  VectorXd v(model.nv);
  v << 1.0;

  VectorXd a(model.nv);
  a << 0.0;

  // Expected velocity
  Motion v_local;
  v_local.linear() = Vector3d(0.0, 1.0, 0.0);
  v_local.angular() = Vector3d(0.0, 0.0, 1.0);

  Motion v_world;
  v_world.linear() = Vector3d::Zero();
  v_world.angular() = Vector3d(0.0, 0.0, 1.0);

  Motion v_align;
  v_align.linear() = Vector3d(-1.0, 0.0, 0.0);
  v_align.angular() = Vector3d(0.0, 0.0, 1.0);

  // Expected classical acceleration
  Motion ac_local;
  ac_local.linear() = Vector3d(-1.0, 0.0, 0.0);
  ac_local.angular() = Vector3d::Zero();

  Motion ac_world = Motion::Zero();

  Motion ac_align;
  ac_align.linear() = Vector3d(0.0, -1.0, 0.0);
  ac_align.angular() = Vector3d::Zero();

  // Perform kinematics
  forwardKinematics(model,data,q,v,a);

  // Check output velocity
  BOOST_CHECK(v_local.isApprox(getFrameVelocity(model,data,frameId)));
  BOOST_CHECK(v_local.isApprox(getFrameVelocity(model,data,frameId,LOCAL)));
  BOOST_CHECK(v_world.isApprox(getFrameVelocity(model,data,frameId,WORLD)));
  BOOST_CHECK(v_align.isApprox(getFrameVelocity(model,data,frameId,LOCAL_WORLD_ALIGNED)));

  // Check output acceleration (all zero)
  BOOST_CHECK(getFrameAcceleration(model,data,frameId).isZero());
  BOOST_CHECK(getFrameAcceleration(model,data,frameId,LOCAL).isZero());
  BOOST_CHECK(getFrameAcceleration(model,data,frameId,WORLD).isZero());
  BOOST_CHECK(getFrameAcceleration(model,data,frameId,LOCAL_WORLD_ALIGNED).isZero());

  // Check output classical acceleration
  BOOST_CHECK(ac_local.isApprox(getFrameClassicalAcceleration(model,data,frameId)));
  BOOST_CHECK(ac_local.isApprox(getFrameClassicalAcceleration(model,data,frameId,LOCAL)));
  BOOST_CHECK(ac_world.isApprox(getFrameClassicalAcceleration(model,data,frameId,WORLD)));
  BOOST_CHECK(ac_align.isApprox(getFrameClassicalAcceleration(model,data,frameId,LOCAL_WORLD_ALIGNED)));
}

BOOST_AUTO_TEST_CASE ( test_get_frame_jacobian )
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  BOOST_CHECK(model.existFrame(frame_name));
  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Ones(model.nv);
  
  /// In local frame
  Model::Index idx = model.getFrameId(frame_name);
  const Frame & frame = model.frames[idx];
  BOOST_CHECK(frame.placement.isApprox_impl(framePlacement));
  Data::Matrix6x Jjj(6,model.nv); Jjj.fill(0);
  Data::Matrix6x Jff(6,model.nv); Jff.fill(0);
  Data::Matrix6x Jff2(6,model.nv); Jff2.fill(0);

  computeJointJacobians(model,data,q);
  updateFramePlacement(model, data, idx);
  getFrameJacobian(model,     data,        idx, LOCAL, Jff);
  computeJointJacobians(model,data_ref,q);
  getJointJacobian(model, data_ref, parent_idx, LOCAL, Jjj);

  Motion nu_frame = Motion(Jff*v);
  Motion nu_joint = Motion(Jjj*v);
  
  const SE3::ActionMatrixType jXf = frame.placement.toActionMatrix();
  Data::Matrix6x Jjj_from_frame(jXf * Jff);
  BOOST_CHECK(Jjj_from_frame.isApprox(Jjj));
  
  BOOST_CHECK(nu_frame.isApprox(frame.placement.actInv(nu_joint), 1e-12));
  
  // In world frame
  getFrameJacobian(model,data,idx,WORLD,Jff);
  getJointJacobian(model, data_ref, parent_idx,WORLD, Jjj);
  BOOST_CHECK(Jff.isApprox(Jjj));
  
  computeFrameJacobian(model,data,q,idx,WORLD,Jff2);
  
  BOOST_CHECK(Jff2.isApprox(Jjj));
}

BOOST_AUTO_TEST_CASE ( test_frame_jacobian )
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  BOOST_CHECK(model.existFrame(frame_name));

  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Ones(model.nv);

  Model::Index idx = model.getFrameId(frame_name);
  const Frame & frame = model.frames[idx];
  BOOST_CHECK(frame.placement.isApprox_impl(framePlacement));
  Data::Matrix6x Jf(6,model.nv); Jf.fill(0);
  Data::Matrix6x Jf2(6,model.nv); Jf2.fill(0);
  Data::Matrix6x Jf_ref(6,model.nv); Jf_ref.fill(0);

  computeFrameJacobian(model, data_ref, q, idx, Jf);

  computeJointJacobians(model, data_ref, q);
  updateFramePlacement(model,  data_ref, idx);
  getFrameJacobian(model,      data_ref, idx, LOCAL, Jf_ref);

  BOOST_CHECK(Jf.isApprox(Jf_ref));
  
  computeFrameJacobian(model,data,q,idx,LOCAL,Jf2);
  
  BOOST_CHECK(Jf2.isApprox(Jf_ref));
}

BOOST_AUTO_TEST_CASE ( test_frame_jacobian_local_world_oriented )
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  BOOST_CHECK(model.existFrame(frame_name));

  pinocchio::Data data(model);

  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Ones(model.nv);

  Model::Index idx = model.getFrameId(frame_name);
  Data::Matrix6x Jf(6,model.nv); Jf.fill(0);
  Data::Matrix6x Jf2(6,model.nv); Jf2.fill(0);
  Data::Matrix6x Jf_ref(6,model.nv); Jf_ref.fill(0);

  computeJointJacobians(model, data, q);
  updateFramePlacement(model,  data, idx);
  getFrameJacobian(model,      data, idx, LOCAL, Jf_ref);

  // Compute the jacobians.
  Jf_ref = SE3(data.oMf[idx].rotation(), Eigen::Vector3d::Zero()).toActionMatrix() * Jf_ref;
  getFrameJacobian(model,      data, idx, LOCAL_WORLD_ALIGNED, Jf);
  
  BOOST_CHECK(Jf.isApprox(Jf_ref));
  
  computeFrameJacobian(model,data,q,idx,LOCAL_WORLD_ALIGNED,Jf2);
  
  BOOST_CHECK(Jf2.isApprox(Jf_ref));
}

BOOST_AUTO_TEST_CASE ( test_frame_jacobian_time_variation )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));  
  pinocchio::Data data(model);
  pinocchio::Data data_ref(model);
  
  VectorXd q = randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) );
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);
  
  computeJointJacobiansTimeVariation(model,data,q,v);
  updateFramePlacements(model,data);

  forwardKinematics(model,data_ref,q,v,a);
  updateFramePlacements(model,data_ref);  

  BOOST_CHECK(isFinite(data.dJ));

  Model::Index idx = model.getFrameId(frame_name);
  const Frame & frame = model.frames[idx];
  BOOST_CHECK(frame.placement.isApprox_impl(framePlacement));
    
  Data::Matrix6x J(6,model.nv); J.fill(0.);
  Data::Matrix6x dJ(6,model.nv); dJ.fill(0.);
  
  // Regarding to the WORLD origin
  getFrameJacobian(model,data,idx,WORLD,J);
  getFrameJacobianTimeVariation(model,data,idx,WORLD,dJ);
  
  Motion v_idx(J*v);
  const Motion & v_ref_local = frame.placement.actInv(data_ref.v[parent_idx]);
  const Motion & v_ref = data_ref.oMf[idx].act(v_ref_local);
  
  const SE3 & wMf = SE3(data_ref.oMf[idx].rotation(),SE3::Vector3::Zero());
  const Motion & v_ref_local_world_aligned = wMf.act(v_ref_local);
  BOOST_CHECK(v_idx.isApprox(v_ref));
  
  Motion a_idx(J*a + dJ*v);
  const Motion & a_ref_local = frame.placement.actInv(data_ref.a[parent_idx]);
  const Motion & a_ref = data_ref.oMf[idx].act(a_ref_local);
  const Motion & a_ref_local_world_aligned = wMf.act(a_ref_local);
  BOOST_CHECK(a_idx.isApprox(a_ref));
  
  J.fill(0.);  dJ.fill(0.);
  // Regarding to the LOCAL frame
  getFrameJacobian(model,data,idx,LOCAL,J);
  getFrameJacobianTimeVariation(model,data,idx,LOCAL,dJ);
  
  v_idx = (Motion::Vector6)(J*v);
  BOOST_CHECK(v_idx.isApprox(v_ref_local));
              
  a_idx = (Motion::Vector6)(J*a + dJ*v);
  BOOST_CHECK(a_idx.isApprox(a_ref_local));
  
  // Regarding to the LOCAL_WORLD_ALIGNED frame
  getFrameJacobian(model,data,idx,LOCAL_WORLD_ALIGNED,J);
  getFrameJacobianTimeVariation(model,data,idx,LOCAL_WORLD_ALIGNED,dJ);
  
  v_idx = (Motion::Vector6)(J*v);
  BOOST_CHECK(v_idx.isApprox(v_ref_local_world_aligned));
              
  a_idx = (Motion::Vector6)(J*a + dJ*v);
  BOOST_CHECK(a_idx.isApprox(a_ref_local_world_aligned));
  
  // compare to finite differencies
  {
    Data data_ref(model), data_ref_plus(model);
    
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model,q,alpha*v);

    //data_ref
    Data::Matrix6x J_ref_world(6,model.nv), J_ref_local(6,model.nv), J_ref_local_world_aligned(6,model.nv);
    J_ref_world.fill(0.); J_ref_local.fill(0.); J_ref_local_world_aligned.fill(0.);
    computeJointJacobians(model,data_ref,q);
    updateFramePlacements(model,data_ref);
    const SE3 & oMf_q = data_ref.oMf[idx];
    getFrameJacobian(model,data_ref,idx,WORLD,J_ref_world);
    getFrameJacobian(model,data_ref,idx,LOCAL,J_ref_local);
    getFrameJacobian(model,data_ref,idx,LOCAL_WORLD_ALIGNED,J_ref_local_world_aligned);
    
    //data_ref_plus
    Data::Matrix6x J_ref_plus_world(6,model.nv), J_ref_plus_local(6,model.nv), J_ref_plus_local_world_aligned(6,model.nv);
    J_ref_plus_world.fill(0.); J_ref_plus_local.fill(0.); J_ref_plus_local_world_aligned.fill(0.);
    computeJointJacobians(model,data_ref_plus,q_plus);
    updateFramePlacements(model,data_ref_plus);
    const SE3 & oMf_q_plus = data_ref_plus.oMf[idx];
    getFrameJacobian(model,data_ref_plus,idx,WORLD,J_ref_plus_world);
    getFrameJacobian(model,data_ref_plus,idx,LOCAL,J_ref_plus_local);
    getFrameJacobian(model,data_ref_plus,idx,LOCAL_WORLD_ALIGNED,J_ref_plus_local_world_aligned);

    //Move J_ref_plus_local to reference frame
    J_ref_plus_local = (oMf_q.inverse()*oMf_q_plus).toActionMatrix()*(J_ref_plus_local);

    //Move J_ref_plus_local_world_aligned to reference frame
    SE3 oMf_translation = SE3::Identity();
    oMf_translation.translation() = oMf_q_plus.translation() - oMf_q.translation();
    J_ref_plus_local_world_aligned = oMf_translation.toActionMatrix()*(J_ref_plus_local_world_aligned);
    
    Data::Matrix6x dJ_ref_world(6,model.nv), dJ_ref_local(6,model.nv), dJ_ref_local_world_aligned(6,model.nv);
    dJ_ref_world.fill(0.); dJ_ref_local.fill(0.); dJ_ref_local_world_aligned.fill(0.);
    dJ_ref_world = (J_ref_plus_world - J_ref_world)/alpha;
    dJ_ref_local = (J_ref_plus_local - J_ref_local)/alpha;
    dJ_ref_local_world_aligned = (J_ref_plus_local_world_aligned - J_ref_local_world_aligned)/alpha;

    //data
    computeJointJacobiansTimeVariation(model,data,q,v);
    forwardKinematics(model,data,q,v);
    updateFramePlacements(model,data);
    Data::Matrix6x dJ_world(6,model.nv), dJ_local(6,model.nv), dJ_local_world_aligned(6,model.nv);
    dJ_world.fill(0.); dJ_local.fill(0.); dJ_local_world_aligned.fill(0.);
    getFrameJacobianTimeVariation(model,data,idx,WORLD,dJ_world);
    getFrameJacobianTimeVariation(model,data,idx,LOCAL,dJ_local);
    getFrameJacobianTimeVariation(model,data,idx,LOCAL_WORLD_ALIGNED,dJ_local_world_aligned);

    BOOST_CHECK(dJ_world.isApprox(dJ_ref_world,sqrt(alpha)));
    BOOST_CHECK(dJ_local.isApprox(dJ_ref_local,sqrt(alpha)));
    BOOST_CHECK(dJ_local_world_aligned.isApprox(dJ_ref_local_world_aligned,sqrt(alpha)));
  }
}
             
BOOST_AUTO_TEST_SUITE_END ()

