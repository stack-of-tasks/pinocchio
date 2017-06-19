//
// Copyright (c) 2016 CNRS
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
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
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


BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidSimple(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.njoints-1);
  const std::string & frame_name = std::string( model.names[parent_idx]+ "_frame");
  const SE3 & framePlacement = SE3::Random();
  model.addFrame(Frame (frame_name, parent_idx, 0, framePlacement, OP_FRAME));
  se3::Data data(model);
  se3::Data data_ref(model);
  

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd q_dot = VectorXd::Ones(model.nv);


  /// In global frame
  Data::Matrix6x Joj(6,model.nv); Joj.fill(0);
  Data::Matrix6x Jof(6,model.nv); Jof.fill(0);
  Model::Index idx = model.getFrameId(frame_name);
  const Frame & frame = model.frames[idx];
  BOOST_CHECK(frame.placement.isApprox_impl(framePlacement));

  computeJacobians(model,data,q);
  computeJacobians(model,data_ref,q);
  framesForwardKinematics(model,data);
  getFrameJacobian<false>(model,data,idx,Jof);
  getJacobian<false>(model, data_ref, parent_idx, Joj);

  Motion nu_frame(Jof*q_dot);
  Motion nu_joint(Joj*q_dot);
  
  SE3 translation(SE3::Identity());
  translation.translation(data.oMi[parent_idx].rotation()*frame.placement.translation());
  
  Motion nu_frame_from_nu_joint(translation.actInv(nu_joint));

  BOOST_CHECK(nu_frame.isApprox(nu_frame_from_nu_joint, 1e-12));

  /// In local frame

  Data::Matrix6x Jjj(6,model.nv); Jjj.fill(0);
  Data::Matrix6x Jff(6,model.nv); Jff.fill(0);
  getFrameJacobian<true>(model,data,idx,Jff);
  getJacobian<true>(model, data_ref, parent_idx, Jjj);

  nu_frame = Motion(Jff*q_dot);
  nu_joint = Motion(Jjj*q_dot);
  
  const SE3::ActionMatrix_t jXf = frame.placement.toActionMatrix();
  Data::Matrix6x Jjj_from_frame(jXf * Jff);
  BOOST_CHECK(Jjj_from_frame.isApprox(Jjj));
  
  nu_frame_from_nu_joint = frame.placement.act(nu_frame);
  BOOST_CHECK(nu_frame.isApprox(frame.placement.actInv(nu_joint), 1e-12));
}

BOOST_AUTO_TEST_SUITE_END ()

