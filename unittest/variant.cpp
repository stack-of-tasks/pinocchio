
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE VariantTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( VariantTest)

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
