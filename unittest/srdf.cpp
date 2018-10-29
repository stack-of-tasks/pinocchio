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
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <boost/test/unit_test.hpp>

using namespace se3;
using namespace std;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_removeCollisionPairs)
{
  using namespace se3::urdf;
  using namespace se3::srdf;
  const string model_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  const string model_dir = PINOCCHIO_SOURCE_DIR"/models/romeo";
  const string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/srdf/romeo.srdf";
  
  Model model;
  buildModel(model_filename, model);
  
  GeometryModel geom_model;
  vector<string> paths; paths.push_back(model_dir);
  buildGeom(model,model_filename,COLLISION,geom_model,paths);
  
  geom_model.addAllCollisionPairs();
  
  const size_t num_init_col_pairs = geom_model.collisionPairs.size();
  
  removeCollisionPairs(model,geom_model,srdf_filename,false);
  const size_t num_col_pairs = geom_model.collisionPairs.size();
  
  BOOST_CHECK(num_init_col_pairs > num_col_pairs);
}
  
BOOST_AUTO_TEST_CASE(readNeutralConfig)
{
  using namespace se3::urdf;
  using namespace se3::srdf;
  const string model_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  const string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/srdf/romeo.srdf";
  
  Model model;
  buildModel(model_filename, model);

  Eigen::VectorXd q = getNeutralConfiguration(model,srdf_filename,false);
  
  BOOST_CHECK(q.size() == model.nq);
  BOOST_CHECK(!q.isZero());
}

BOOST_AUTO_TEST_CASE(readRotorParams)
{
  using namespace se3::urdf;
  using namespace se3::srdf;
  const string model_filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  const string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.srdf";
  
  Model model;
  buildModel(model_filename, model);

  loadRotorParameters(model,srdf_filename,false);
  
  BOOST_CHECK(model.rotorInertia(model.joints[model.getJointId("WAIST_P")].idx_v())==1.0);
  BOOST_CHECK(model.rotorGearRatio(model.joints[model.getJointId("WAIST_R")].idx_v())==1.0);
}

BOOST_AUTO_TEST_SUITE_END()
