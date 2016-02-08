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
#include "pinocchio/algorithm/operational-frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE OperationalFramesTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( OperationalFramesTest)

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.nbody-1);
  const std::string & frame_name = std::string( model.getJointName(parent_idx)+ "_frame");
  const SE3 & frame_placement = SE3::Random();
  model.addFrame(frame_name, parent_idx, frame_placement);
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  framesForwardKinematic(model, data, q);

  BOOST_CHECK(data.oMof[model.getFrameId(frame_name)].isApprox(data.oMi[parent_idx]*frame_placement));

}


BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;
  typedef Data::Matrix6x Matrix6x;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.nbody-1);
  const std::string & frame_name = std::string( model.getJointName(parent_idx)+ "_frame");
  const SE3 & frame_placement = SE3::Random();
  model.addFrame(frame_name, parent_idx, frame_placement);
  se3::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  framesForwardKinematic(model, data, q);


  computeJacobians(model,data,q);


  MatrixXd expected(6,model.nv); expected.fill(0);
  Matrix6x Jof(6,model.nv); Jof.fill(0);
  Model::Index idx = model.getFrameId(frame_name);

  getFrameJacobian<false>(model,data,idx,Jof);
  getJacobian<false>(model, data, parent_idx, expected);
  expected = frame_placement.inverse().toActionMatrix() * expected;

  BOOST_CHECK(Jof.isApprox(expected, 1e-12));


  expected.fill(0); Jof.fill(0);
  getFrameJacobian<true>(model,data,idx,Jof);
  getJacobian<true>(model, data, parent_idx, expected);

  expected = frame_placement.inverse().toActionMatrix() * expected;

  // std::cout << Jof << std::endl;
  // std::cout << "----" << std::endl;
  // std::cout << frame_placement << std::endl;
  // std::cout << expected << std::endl;
  BOOST_CHECK(Jof.isApprox(expected, 1e-12));
}

BOOST_AUTO_TEST_SUITE_END ()

