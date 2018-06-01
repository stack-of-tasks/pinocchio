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

#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/python.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( buildModel )
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_model.py";

  #ifndef NDEBUG
   std::cout << "Parse filename \"" << filename << "\"" << std::endl;
  #endif
  se3::Model model = se3::python::buildModel(filename,"model",false);
  #ifndef NDEBUG
   std::cout << "This model has \"" << model.nq << "\" DoF" << std::endl;
  #endif

  BOOST_CHECK(model.nq==9);
  BOOST_CHECK(model.nv==8);
}

BOOST_AUTO_TEST_SUITE_END()
