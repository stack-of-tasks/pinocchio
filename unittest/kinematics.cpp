//
// Copyright (c) 2018 CNRS
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
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematics_zero)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidSimple(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  forwardKinematics(model,data_ref,q);
  crba(model,data,q);
  updateGlobalPlacements(model,data);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.liMi[i] == data_ref.liMi[i]);
  }
}

BOOST_AUTO_TEST_CASE(test_kinematics_first)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidSimple(model);
  
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  
  forwardKinematics(model,data,q,v);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.v[i] == Motion::Zero());
  }
}

BOOST_AUTO_TEST_CASE(test_kinematics_second)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidSimple(model);
  
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));
  
  forwardKinematics(model,data,q,v,a);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.v[i] == Motion::Zero());
    BOOST_CHECK(data.a[i] == Motion::Zero());
  }
}

BOOST_AUTO_TEST_SUITE_END()
