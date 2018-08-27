//
// Copyright (c) 2015-2018 CNRS
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
#include <fstream>
#include <streambuf>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <boost/test/unit_test.hpp>

#include <urdf_parser/urdf_parser.h>


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( build_model )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  const std::string dir = PINOCCHIO_SOURCE_DIR"/models/romeo";
  
  se3::Model model;
  se3::urdf::buildModel(filename, model);
  se3::GeometryModel geomModel;
  se3::urdf::buildGeom(model, filename, se3::COLLISION, geomModel, dir);
  
  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE ( build_model_simple_humanoid )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";

  se3::Model model;
  se3::urdf::buildModel(filename, model);

  BOOST_CHECK(model.nq == 29);
  
  se3::Model model_ff;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(), model_ff);
  
  BOOST_CHECK(model_ff.nq == 36);
}
  
BOOST_AUTO_TEST_CASE ( build_model_from_XML )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  
  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  
  se3::Model model;
  se3::urdf::buildModelFromXML(filestr, model);
  
  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE ( build_model_from_UDRFTree )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
  
  se3::Model model;
  se3::urdf::buildModel(urdfTree, model);
  
  BOOST_CHECK(model.nq == 31);
}
  
BOOST_AUTO_TEST_CASE ( build_model_with_joint )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  const std::string dir = PINOCCHIO_SOURCE_DIR"/models/romeo";
  
  se3::Model model;
  se3::urdf::buildModel(filename, se3::JointModelFreeFlyer(), model);
  se3::GeometryModel geomModel;
  se3::urdf::buildGeom(model, filename, se3::COLLISION, geomModel, dir);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE ( build_model_with_joint_from_XML )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  
  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  
  se3::Model model;
  se3::urdf::buildModelFromXML(filestr, se3::JointModelFreeFlyer(), model);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE ( build_model_with_joint_from_UDRFTree )
{
  const std::string filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/urdf/romeo_small.urdf";
  
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
  
  se3::Model model;
  se3::urdf::buildModel(urdfTree, se3::JointModelFreeFlyer(), model);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_SUITE_END()
