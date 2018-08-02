//
// Copyright(c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or(at your option) any later version.
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
#include "pinocchio/algorithm/copy.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_data_copy)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidSimple(model);
  
  model.upperPositionLimit.head<3>().fill(100);
  model.upperPositionLimit.segment<4>(3).setOnes();
  model.lowerPositionLimit.head<7>() = - model.upperPositionLimit.head<7>();
  
  VectorXd q(model.nq);
  q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));
  
  Data data_ref(model), data(model);
  forwardKinematics(model,data_ref,q,v,a);
  rnea(model,data_ref,q,v,a); // for a_gf to be initialized
  
  // Check zero order kinematic quantities
  copy(model,data_ref,data,0);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
  }
  
  // Check first order kinematic quantities
  copy(model,data_ref,data,1);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.v[i] == data_ref.v[i]);
  }
  
  // Check second order kinematic quantities
  copy(model,data_ref,data,2);
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.v[i] == data_ref.v[i]);
    BOOST_CHECK(data.a[i] == data_ref.a[i]);
    BOOST_CHECK(data.a_gf[i] == data_ref.a_gf[i]);
  }
  

}

BOOST_AUTO_TEST_SUITE_END()
