//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include <iostream>
#include <fstream>
#include <streambuf>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
#include <hpp/fcl/collision_object.h>
#endif // PINOCCHIO_WITH_HPP_FCL

#include <boost/test/unit_test.hpp>

#include <urdf_parser/urdf_parser.h>


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( build_model )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR + std::string("/others/robots");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);
  
  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE ( build_model_simple_humanoid )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);

  BOOST_CHECK_EQUAL(model.nq, 29);

  // Check that parsing collision_checking works.
  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);
  BOOST_CHECK_EQUAL(geomModel.ngeoms, 2);

#ifdef PINOCCHIO_WITH_HPP_FCL
  // Check that cylinder is converted into capsule.
#ifdef PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), hpp::fcl::GEOM_CYLINDER);
#else // PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), hpp::fcl::GEOM_CAPSULE);
#endif

#if ( HPP_FCL_MAJOR_VERSION>1 || ( HPP_FCL_MAJOR_VERSION==1 && \
      ( HPP_FCL_MINOR_VERSION>1 || ( HPP_FCL_MINOR_VERSION==1 && \
                                     HPP_FCL_PATCH_VERSION>3))))
#  define PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3
#endif

#if defined(PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3) && !defined(PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME)
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[1].geometry->getNodeType(), hpp::fcl::GEOM_CONVEX);
#undef PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3
#else // PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3 && !PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[1].geometry->getObjectType(), hpp::fcl::OT_BVH);
#endif // PINOCCHIO_HPP_FCL_SUPERIOR_TO_1_1_3 && !PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
#endif // PINOCCHIO_WITH_HPP_FCL
  
  pinocchio::Model model_ff;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_ff);
  
  BOOST_CHECK(model_ff.nq == 36);
}
  
BOOST_AUTO_TEST_CASE ( build_model_from_XML )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  
  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);
  
  BOOST_CHECK(model.nq == 31);
}
  
BOOST_AUTO_TEST_CASE ( check_tree_from_XML )
{
  // Read file as XML
  std::string filestr(
      "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
      "<robot name=\"test\">"
      "  <link name=\"base_link\"/>"
      "  <link name=\"link_1\"/>"
      "  <link name=\"link_2\"/>"
      "  <joint name=\"joint_1\" type=\"fixed\">"
      "    <origin xyz=\"1 0 0\"/>"
      "    <axis xyz=\"0 0 1\"/>"
      "    <parent link=\"base_link\"/>"
      "    <child link=\"link_1\"/>"
      "  </joint>"
      "  <joint name=\"joint_2\" type=\"fixed\">"
      "    <origin xyz=\"0 1 0\"/>"
      "    <axis xyz=\"0 0 1\"/>"
      "    <parent link=\"link_1\"/>"
      "    <child link=\"link_2\"/>"
      "  </joint>"
      "</robot>"
      );
  
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  pinocchio::JointIndex
    base_link_id = model.getFrameId("base_link"),
    link1_id     = model.getFrameId("link_1"),
    link2_id     = model.getFrameId("link_2"),
    joint1_id    = model.getFrameId("joint_1"),
    joint2_id    = model.getFrameId("joint_2");

  BOOST_CHECK_EQUAL(base_link_id, model.frames[joint1_id].previousFrame);
  BOOST_CHECK_EQUAL(joint1_id   , model.frames[link1_id ].previousFrame);
  BOOST_CHECK_EQUAL(link1_id    , model.frames[joint2_id].previousFrame);
  BOOST_CHECK_EQUAL(joint2_id   , model.frames[link2_id ].previousFrame);
}

BOOST_AUTO_TEST_CASE ( build_model_from_UDRFTree )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfTree, model);
  
  BOOST_CHECK(model.nq == 31);
}
  
BOOST_AUTO_TEST_CASE ( build_model_with_joint )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR + std::string("/others/robots");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::GeometryModel geomModel_collision, geomModel_visual;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel_collision, dir);
  pinocchio::urdf::buildGeom(model, filename, pinocchio::VISUAL, geomModel_visual, dir);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE ( build_model_with_joint_from_XML )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  
  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, pinocchio::JointModelFreeFlyer(), model);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE ( build_model_with_joint_from_UDRFTree )
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/others/robots/romeo_description/urdf/romeo_small.urdf");
  
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfTree, pinocchio::JointModelFreeFlyer(), model);
  
  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE(append_two_URDF_models)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
  
  BOOST_CHECK(model.njoints == 30);
  const int nframes = model.nframes;
  const std::string filestr(
                            "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
                            "<robot name=\"test\">"
                            "  <link name=\"box\"/>"
                            "</robot>"
                            );
  
  pinocchio::urdf::buildModelFromXML(filestr, model);
  BOOST_CHECK(model.njoints == 30);
  BOOST_CHECK(nframes + 1 == model.nframes);
}

BOOST_AUTO_TEST_CASE(append_two_URDF_models_with_root_joint)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  
  BOOST_CHECK(model.njoints == 31);
  const std::string filestr(
                            "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
                            "<robot name=\"test\">"
                            "  <link name=\"box\"/>"
                            "</robot>"
                            );
  
  
  BOOST_CHECK_THROW(pinocchio::urdf::buildModelFromXML(filestr, pinocchio::JointModelFreeFlyer(), model),
                    std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(check_specific_models)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/baxter_simple.urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
}

BOOST_AUTO_TEST_SUITE_END()
