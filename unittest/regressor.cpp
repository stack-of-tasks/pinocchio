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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_static_regressor)
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model; buildModels::humanoidRandom(model);
  
  se3::Data data(model);
  se3::Data data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);
  
  VectorXd q = randomConfiguration(model);
  regressor::computeStaticRegressor(model,data,q);
  
  VectorXd phi(4*(model.njoints-1));
  for(int k = 1; k < model.njoints; ++k)
  {
    const Inertia & Y = model.inertias[(size_t)k];
    phi.segment<4>(4*(k-1)) << Y.mass(), Y.mass() * Y.lever();
  }
  
  Vector3d com = centerOfMass(model,data_ref,q);
  Vector3d static_com_ref;
  static_com_ref <<  com;
  
  Vector3d static_com = data.staticRegressor * phi;
  
  BOOST_CHECK(static_com.isApprox(static_com_ref)); 
}

BOOST_AUTO_TEST_SUITE_END()
