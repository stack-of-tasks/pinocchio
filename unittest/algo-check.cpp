//
// Copyright (c) 2016-2017 CNRS
//

#include <boost/fusion/container/generation/make_list.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/check.hpp>
#include <pinocchio/algorithm/default-check.hpp>
#include <iostream>

using namespace pinocchio;

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

// Dummy checker.
struct Check1 : public AlgorithmCheckerBase<Check1>
{
  bool checkModel_impl( const Model& ) const { return true; }
};

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_check )
{
  using namespace boost::fusion;

  pinocchio::Model model; buildModels::humanoidRandom(model);
  
  BOOST_CHECK(model.check (Check1()));
  BOOST_CHECK(model.check (CRBAChecker()));
  BOOST_CHECK(! model.check (ABAChecker())); // some inertias are negative ... check fail.
  BOOST_FOREACH(Inertia& Y,model.inertias) 
      Y.mass() = Y.inertia().data()[0] = Y.inertia().data()[3] = Y.inertia().data()[5] = 1.0 ;
  BOOST_CHECK(model.check (ABAChecker())); // some inertias are negative ... check fail.

  BOOST_CHECK(model.check(makeAlgoCheckerList(Check1(),ParentChecker(),CRBAChecker()) ));
  BOOST_CHECK(model.check(DEFAULT_CHECKERS));

  pinocchio::Data data(model);
  BOOST_CHECK(checkData(model,data));
  BOOST_CHECK(model.check(data));
  
}

BOOST_AUTO_TEST_SUITE_END ()

