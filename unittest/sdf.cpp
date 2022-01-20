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
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  
  pinocchio::Model model;
  const std::string rootLinkName = "pelvis";
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, model, contact_models,rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel,rootLinkName, dir);

  BOOST_CHECK(model.nq == 62);
}
  
BOOST_AUTO_TEST_CASE ( build_model_with_joint )
{

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  const std::string rootLinkName = "pelvis";
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model, contact_models,rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel,rootLinkName, dir);

  BOOST_CHECK(model.nq == 69);
}

BOOST_AUTO_TEST_CASE ( build_model_without_rootLink )
{

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  const std::string rootLinkName = "";
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model, contact_models,rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel,rootLinkName, dir);
  
  BOOST_CHECK(model.nq == 69);
}

BOOST_AUTO_TEST_CASE (compare_model_with_urdf)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model_sdf;
  const std::string rootLinkName = "WAIST_LINK0";
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_sdf, contact_models, rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model_sdf, filename, pinocchio::COLLISION, geomModel, rootLinkName, dir);

  const std::string filename_urdf = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir_urdf = PINOCCHIO_MODEL_DIR;
  pinocchio::Model model_urdf;
  pinocchio::urdf::buildModel(filename_urdf, pinocchio::JointModelFreeFlyer(), model_urdf);

  typedef typename pinocchio::Model::ConfigVectorMap ConfigVectorMap;
  //Compare models
  //BOOST_CHECK(model_urdf==model);

  BOOST_CHECK(model_urdf.nq == model_sdf.nq);
  BOOST_CHECK(model_urdf.nv == model_sdf.nv);
  BOOST_CHECK(model_urdf.njoints == model_sdf.njoints);
  BOOST_CHECK(model_urdf.nbodies == model_sdf.nbodies);
  BOOST_CHECK(model_urdf.nframes == model_sdf.nframes);
  BOOST_CHECK(model_urdf.parents == model_sdf.parents);
  BOOST_CHECK(model_urdf.children == model_sdf.children);
  BOOST_CHECK(model_urdf.names == model_sdf.names);
  BOOST_CHECK(model_urdf.subtrees == model_sdf.subtrees);
  BOOST_CHECK(model_urdf.gravity == model_sdf.gravity);
  BOOST_CHECK(model_urdf.name == model_sdf.name);
  BOOST_CHECK(model_urdf.idx_qs == model_sdf.idx_qs);
  BOOST_CHECK(model_urdf.nqs == model_sdf.nqs);
  BOOST_CHECK(model_urdf.idx_vs == model_sdf.idx_vs);
  BOOST_CHECK(model_urdf.nvs == model_sdf.nvs);
  BOOST_CHECK(model_urdf.referenceConfigurations.size() == model_sdf.referenceConfigurations.size());
  
  typename ConfigVectorMap::const_iterator it = model_sdf.referenceConfigurations.begin();
  typename ConfigVectorMap::const_iterator it_model_urdf = model_urdf.referenceConfigurations.begin();
  for(long k = 0; k < (long)model_sdf.referenceConfigurations.size(); ++k)
  {
    std::advance(it,k); std::advance(it_model_urdf,k);
    BOOST_CHECK(it->second.size() == it_model_urdf->second.size());
    BOOST_CHECK(it->second == it_model_urdf->second);    
  }
  BOOST_CHECK(model_urdf.armature.size() == model_sdf.armature.size());
  
  BOOST_CHECK(model_urdf.armature == model_sdf.armature);
  BOOST_CHECK(model_urdf.friction.size() == model_sdf.friction.size());
  BOOST_CHECK(model_urdf.friction == model_sdf.friction);

  BOOST_CHECK(model_urdf.damping.size() == model_sdf.damping.size());

  BOOST_CHECK(model_urdf.damping == model_sdf.damping);

  BOOST_CHECK(model_urdf.rotorInertia.size() == model_sdf.rotorInertia.size());

  BOOST_CHECK(model_urdf.rotorInertia == model_sdf.rotorInertia);

  BOOST_CHECK(model_urdf.rotorGearRatio.size() == model_sdf.rotorGearRatio.size());
  
  BOOST_CHECK(model_urdf.rotorGearRatio == model_sdf.rotorGearRatio);

  BOOST_CHECK(model_urdf.effortLimit.size() == model_sdf.effortLimit.size());
  BOOST_CHECK(model_urdf.effortLimit == model_sdf.effortLimit);

  BOOST_CHECK(model_urdf.velocityLimit.size() == model_sdf.velocityLimit.size());

  BOOST_CHECK(model_urdf.velocityLimit == model_sdf.velocityLimit);
  BOOST_CHECK(model_urdf.lowerPositionLimit.size() == model_sdf.lowerPositionLimit.size());
  BOOST_CHECK(model_urdf.lowerPositionLimit == model_sdf.lowerPositionLimit);

  BOOST_CHECK(model_urdf.upperPositionLimit.size() == model_sdf.upperPositionLimit.size());
  BOOST_CHECK(model_urdf.upperPositionLimit == model_sdf.upperPositionLimit);

  for(size_t k = 1; k < model_sdf.inertias.size(); ++k)
  {
    BOOST_CHECK(model_urdf.inertias[k].isApprox(model_sdf.inertias[k]));
  }

  for(size_t k = 1; k < model_urdf.jointPlacements.size(); ++k)
  {
    BOOST_CHECK(model_urdf.jointPlacements[k] == model_sdf.jointPlacements[k]);
  }

  BOOST_CHECK(model_urdf.joints == model_sdf.joints);

  BOOST_CHECK(model_urdf.frames.size() == model_sdf.frames.size());
  for(size_t k = 1; k < model_urdf.frames.size(); ++k)
  {
    BOOST_CHECK(model_urdf.frames[k] == model_sdf.frames[k]);
  }
}


BOOST_AUTO_TEST_SUITE_END()
