//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include <iostream>
#include <fstream>
#include <streambuf>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sdf.hpp"

#include <hpp/fcl/collision_object.h>

#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( build_model )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/cassie_description/robots/cassie_v2.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, model, contact_models);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);

  BOOST_CHECK(model.nq == 22);
}
  
BOOST_AUTO_TEST_CASE ( build_model_with_joint )
{

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/cassie_description/robots/cassie_v2.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model, contact_models);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);

  BOOST_CHECK(model.nq == 29);
}

BOOST_AUTO_TEST_SUITE_END()
