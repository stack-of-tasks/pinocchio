//
// Copyright (c) 2016 CNRS
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
#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_model_subtree)
{
  Model model;
  std::cout << "build model" << std::endl;
  buildModels::humanoidRandom(model);
  
  Model::JointIndex idx_larm1 = model.getJointId("larm1_joint");
  BOOST_CHECK(idx_larm1<(Model::JointIndex)model.njoints);
  Model::IndexVector subtree = model.subtrees[idx_larm1];
  BOOST_CHECK(subtree.size()==6);
  
  for(size_t i=1; i<subtree.size();++i)
    BOOST_CHECK(model.parents[subtree[i]]==subtree[i-1]);
}

BOOST_AUTO_TEST_SUITE_END()
