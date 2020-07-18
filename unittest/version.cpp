//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include <pinocchio/fwd.hpp>
#include <pinocchio/utils/version.hpp>

#include "utils/macros.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_version)
{
  using namespace std;
  using namespace pinocchio;
  
  const string delimiter = ".";
  ostringstream version_ref;
  version_ref
  << PINOCCHIO_MAJOR_VERSION << delimiter
  << PINOCCHIO_MINOR_VERSION << delimiter
  << PINOCCHIO_PATCH_VERSION;
  
  BOOST_CHECK_EQUAL(version_ref.str().c_str(),printVersion());
  
  BOOST_CHECK(checkVersionAtLeast(0,0,0));
  BOOST_CHECK(checkVersionAtLeast(PINOCCHIO_MAJOR_VERSION,PINOCCHIO_MINOR_VERSION,PINOCCHIO_PATCH_VERSION));
  BOOST_CHECK(!checkVersionAtLeast(PINOCCHIO_MAJOR_VERSION,PINOCCHIO_MINOR_VERSION,PINOCCHIO_PATCH_VERSION+1));
  BOOST_CHECK(!checkVersionAtLeast(99,0,0));
}

BOOST_AUTO_TEST_SUITE_END()
