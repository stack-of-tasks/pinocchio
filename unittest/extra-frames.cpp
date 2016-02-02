//
// Copyright (c) 2015 CNRS
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
#include "pinocchio/algorithm/extra-frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ExtraFramesTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

BOOST_AUTO_TEST_SUITE ( ExtraFramesTest)

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.nbody-1);
  const std::string & frame_name = std::string( model.getJointName(parent_idx)+ "_frame");
  model.addFrame(frame_name, parent_idx, SE3());
  se3::Data data(model);

  VectorXd q = VectorXd::Random(model.nq);
  extraFramesForwardKinematic(model, data, q);

}


BOOST_AUTO_TEST_CASE ( test_jacobian )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  Model::Index parent_idx = model.existJointName("rarm2_joint")?model.getJointId("rarm2_joint"):(Model::Index)(model.nbody-1);
  const std::string & frame_name = std::string( model.getJointName(parent_idx)+ "_frame");
  model.addFrame(frame_name, parent_idx, SE3(1));
  se3::Data data(model);

  VectorXd q = VectorXd::Random(model.nq);
  extraFramesForwardKinematic(model, data, q);

  computeJacobians(model,data,q);

  MatrixXd expected(6,model.nv); expected.fill(0);
  MatrixXd Jef(6,model.nv); Jef.fill(0);
  Model::Index idx = model.getFrameId(frame_name);

  getExtraFrameJacobian<false>(model,data,idx,Jef);

  getJacobian<false>(model, data, parent_idx, expected);
  
  is_matrix_absolutely_closed(Jef, expected, 1e-12);
}

BOOST_AUTO_TEST_SUITE_END ()

