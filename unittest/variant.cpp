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
#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_variant )
{
	using namespace Eigen;
  using namespace se3;;


  JointModelVariant jmodel = JointModelRX();
  const JointDataVariant & jdata = CreateJointData::run(jmodel);

  JointDataGeneric jdatagen(jdata);
  JointModelGeneric jmodelgen(jmodel);

  se3::Model model;
}

BOOST_AUTO_TEST_SUITE_END ()
