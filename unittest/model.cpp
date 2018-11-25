//
// Copyright (c) 2016,2018 CNRS
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

  BOOST_AUTO_TEST_CASE(test_model_subtree)
  {
    Model model;
    buildModels::humanoidRandom(model);
    
    Model::JointIndex idx_larm1 = model.getJointId("larm1_joint");
    BOOST_CHECK(idx_larm1<(Model::JointIndex)model.njoints);
    Model::IndexVector subtree = model.subtrees[idx_larm1];
    BOOST_CHECK(subtree.size()==6);
    
    for(size_t i=1; i<subtree.size();++i)
      BOOST_CHECK(model.parents[subtree[i]]==subtree[i-1]);
  }

  BOOST_AUTO_TEST_CASE(comparison)
  {
    Model model;
    buildModels::humanoidRandom(model);
    
    BOOST_CHECK(model == model);
  }
  
  BOOST_AUTO_TEST_CASE(cast)
  {
    Model model;
    buildModels::humanoidRandom(model);
    
    BOOST_CHECK(model.cast<double>() == model);
    BOOST_CHECK(model.cast<long double>().cast<double>() == model);
  }

BOOST_AUTO_TEST_SUITE_END()
