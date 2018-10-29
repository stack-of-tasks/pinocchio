//
// Copyright (c) 2016-2018 CNRS
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

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  framesForwardKinematics(model, data, q);

  BOOST_CHECK(data.oMf[model.getFrameId(frame_name)].isApprox(data.oMi[parent_idx]*framePlacement));

}

BOOST_AUTO_TEST_CASE ( test_update_placements )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  Model::FrameIndex frame_idx = model.getFrameId(frame_name);
  se3::Data data(model);
  se3::Data data_ref(model);

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
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  Model::FrameIndex frame_idx = model.getFrameId(frame_name);
  se3::Data data(model);
  se3::Data data_ref(model);

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
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  Model::FrameIndex frame_idx = model.getFrameId(frame_name);
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  forwardKinematics(model, data, q, v);

  Motion vf;
  getFrameVelocity(model, data, frame_idx, vf);

  BOOST_CHECK(vf.isApprox(framePlacement.actInv(data.v[parent_idx])));
}

BOOST_AUTO_TEST_CASE ( test_acceleration )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  Model::FrameIndex frame_idx = model.getFrameId(frame_name);
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);
  forwardKinematics(model, data, q, v, a);

  Motion af;
  getFrameAcceleration(model, data, frame_idx, af);

  BOOST_CHECK(af.isApprox(framePlacement.actInv(data.a[parent_idx])));
}

BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  BOOST_CHECK(model.existFrame(frame_name));
  
  se3::Data data(model);
  se3::Data data_ref(model);
  
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
  computeJointJacobians(model,data,q);
  updateFramePlacement(model, data, idx);
  getFrameJacobian<LOCAL>(model, data,    idx,         Jff);
  computeJointJacobians(model,data_ref,q);
  getJointJacobian<LOCAL>(model, data_ref, parent_idx, Jjj);

  Motion nu_frame = Motion(Jff*v);
  Motion nu_joint = Motion(Jjj*v);
  
  const SE3::ActionMatrix_t jXf = frame.placement.toActionMatrix();
  Data::Matrix6x Jjj_from_frame(jXf * Jff);
  BOOST_CHECK(Jjj_from_frame.isApprox(Jjj));
  
  BOOST_CHECK(nu_frame.isApprox(frame.placement.actInv(nu_joint), 1e-12));
  
  // In world frame
  getFrameJacobian<WORLD>(model,data,idx,Jff);
  getJointJacobian<WORLD>(model, data_ref, parent_idx, Jjj);
  BOOST_CHECK(Jff.isApprox(Jjj));
}

BOOST_AUTO_TEST_CASE ( test_frame_jacobian_time_variation )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));  
  se3::Data data(model);
  se3::Data data_ref(model);
  
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
  
  // Regarding to the world origin
  getFrameJacobian<WORLD>(model,data,idx,J);
  getFrameJacobianTimeVariation<WORLD>(model,data,idx,dJ);
  
  Motion v_idx(J*v);
  const Motion & v_ref_local = frame.placement.actInv(data_ref.v[parent_idx]);
  const Motion & v_ref = data_ref.oMf[idx].act(v_ref_local);
  BOOST_CHECK(v_idx.isApprox(v_ref));
  
  Motion a_idx(J*a + dJ*v);
  const Motion & a_ref_local = frame.placement.actInv(data_ref.a[parent_idx]);
  const Motion & a_ref = data_ref.oMf[idx].act(a_ref_local);
  BOOST_CHECK(a_idx.isApprox(a_ref));
  
  J.fill(0.);  dJ.fill(0.);
  // Regarding to the local frame
  getFrameJacobian<LOCAL>(model,data,idx,J);
  getFrameJacobianTimeVariation<LOCAL>(model,data,idx,dJ);
  
  v_idx = (Motion::Vector6)(J*v);
  BOOST_CHECK(v_idx.isApprox(v_ref_local));
              
  a_idx = (Motion::Vector6)(J*a + dJ*v);
  BOOST_CHECK(a_idx.isApprox(a_ref_local));
  
  // compare to finite differencies
  {
    Data data_ref(model), data_ref_plus(model);
    
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model,q,alpha*v);

    //data_ref
    Data::Matrix6x J_ref_world(6,model.nv), J_ref_local(6,model.nv);
    J_ref_world.fill(0.);     J_ref_local.fill(0.);
    computeJointJacobians(model,data_ref,q);
    updateFramePlacements(model,data_ref);
    const SE3 & oMf_q = data_ref.oMf[idx];
    getFrameJacobian<WORLD>(model,data_ref,idx,J_ref_world);
    getFrameJacobian<LOCAL>(model,data_ref,idx,J_ref_local);
    
    //data_ref_plus
    Data::Matrix6x J_ref_plus_world(6,model.nv), J_ref_plus_local(6,model.nv);
    J_ref_plus_world.fill(0.);    J_ref_plus_local.fill(0.);
    computeJointJacobians(model,data_ref_plus,q_plus);
    updateFramePlacements(model,data_ref_plus);
    const SE3 & oMf_qplus = data_ref_plus.oMf[idx];
    getFrameJacobian<WORLD>(model,data_ref_plus,idx,J_ref_plus_world);
    getFrameJacobian<LOCAL>(model,data_ref_plus,idx,J_ref_plus_local);

    //Move J_ref_plus_local to reference frame
    J_ref_plus_local = (oMf_q.inverse()*oMf_qplus).toActionMatrix()*(J_ref_plus_local);
    
    Data::Matrix6x dJ_ref_world(6,model.nv), dJ_ref_local(6,model.nv);
    dJ_ref_world.fill(0.);        dJ_ref_local.fill(0.);
    dJ_ref_world = (J_ref_plus_world - J_ref_world)/alpha;
    dJ_ref_local = (J_ref_plus_local - J_ref_local)/alpha;

    //data
    computeJointJacobiansTimeVariation(model,data,q,v);
    forwardKinematics(model,data,q,v);
    updateFramePlacements(model,data);
    Data::Matrix6x dJ_world(6,model.nv), dJ_local(6,model.nv);
    dJ_world.fill(0.);    dJ_local.fill(0.);
    getFrameJacobianTimeVariation<WORLD>(model,data,idx,dJ_world);
    getFrameJacobianTimeVariation<LOCAL>(model,data,idx,dJ_local);

    BOOST_CHECK(dJ_world.isApprox(dJ_ref_world,sqrt(alpha)));
    BOOST_CHECK(dJ_local.isApprox(dJ_ref_local,sqrt(alpha)));   
  }
}
             
BOOST_AUTO_TEST_SUITE_END ()

