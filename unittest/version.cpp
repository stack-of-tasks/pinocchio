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

#include "utils/macros.hpp"

#include <pinocchio/macros.hpp>
#include <pinocchio/utils/version.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_version)
{
  using namespace std;
  using namespace se3;
  
  const string delimiter = ".";
  ostringstream version_ref;
  version_ref
  << PINOCCHIO_MAJOR_VERSION << delimiter
  << PINOCCHIO_MINOR_VERSION << delimiter
  << PINOCCHIO_PATCH_VERSION;
  
  BOOST_CHECK_EQUAL(version_ref.str().c_str(),printVersion());
  
  BOOST_CHECK(checkVersionAtLeast(0,0,0));
  BOOST_CHECK(checkVersionAtLeast(PINOCCHIO_MAJOR_VERSION,PINOCCHIO_MINOR_VERSION,PINOCCHIO_PATCH_VERSION));
  BOOST_CHECK(not checkVersionAtLeast(PINOCCHIO_MAJOR_VERSION,PINOCCHIO_MINOR_VERSION,PINOCCHIO_PATCH_VERSION+1));
  BOOST_CHECK(not checkVersionAtLeast(99,0,0));
}

BOOST_AUTO_TEST_SUITE_END()
