//
// Copyright (c) 2015-2022 CNRS INRIA
//

#include <iostream>
#include <fstream>
#include <streambuf>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <coal/collision_object.h>
#endif // PINOCCHIO_WITH_HPP_FCL

#include <boost/test/unit_test.hpp>

#include <urdf_parser/urdf_parser.h>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(build_model)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);

  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE(build_model_with_root_joint_name)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  BOOST_CHECK(model.names[1] == "root_joint");

  pinocchio::Model model_name;
  const std::string name_ = "freeFlyer_joint";
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), name_, model_name);
  BOOST_CHECK(model_name.names[1] == name_);
}

BOOST_AUTO_TEST_CASE(build_model_simple_humanoid)
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
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), coal::GEOM_CYLINDER);
  #else // PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), coal::GEOM_CAPSULE);
  #endif

  #ifndef PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[1].geometry->getNodeType(), coal::GEOM_CONVEX);
  #else // PINOCCHIO_URDFDOM_COLLISION_WITH_GROUP_NAME
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[1].geometry->getObjectType(), coal::OT_BVH);
  #endif
#endif // PINOCCHIO_WITH_HPP_FCL

  pinocchio::Model model_ff;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_ff);

  BOOST_CHECK(model_ff.nq == 36);
}

BOOST_AUTO_TEST_CASE(check_mesh_relative_path)
{
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model0;
  pinocchio::urdf::buildModel(filename, model0);
  pinocchio::GeometryModel geomModel0;
  pinocchio::urdf::buildGeom(model0, filename, pinocchio::COLLISION, geomModel0, dir);
  BOOST_CHECK_EQUAL(geomModel0.ngeoms, 2);

  // check if urdf with relative mesh path without //package can be loaded
  filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid_rel_mesh.urdf");
  pinocchio::Model model1;
  pinocchio::urdf::buildModel(filename, model1);
  pinocchio::GeometryModel geomModel1;
  pinocchio::urdf::buildGeom(model1, filename, pinocchio::COLLISION, geomModel1, dir);
  BOOST_CHECK_EQUAL(geomModel1.ngeoms, 2);

  BOOST_CHECK_EQUAL(
    geomModel0.geometryObjects[1].name.compare(geomModel1.geometryObjects[1].name), 0);
}

BOOST_AUTO_TEST_CASE(build_model_from_XML)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");

  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE(check_tree_from_XML)
{
  // Read file as XML
  std::string filestr("<?xml version=\"1.0\" encoding=\"utf-8\"?>"
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
                      "</robot>");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  pinocchio::JointIndex base_link_id = model.getFrameId("base_link"),
                        link1_id = model.getFrameId("link_1"),
                        link2_id = model.getFrameId("link_2"),
                        joint1_id = model.getFrameId("joint_1"),
                        joint2_id = model.getFrameId("joint_2");

  BOOST_CHECK_EQUAL(base_link_id, model.frames[joint1_id].parentFrame);
  BOOST_CHECK_EQUAL(joint1_id, model.frames[link1_id].parentFrame);
  BOOST_CHECK_EQUAL(link1_id, model.frames[joint2_id].parentFrame);
  BOOST_CHECK_EQUAL(joint2_id, model.frames[link2_id].parentFrame);
}

BOOST_AUTO_TEST_CASE(build_model_from_UDRFTree)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfTree, model);

  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE(build_model_with_joint)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::GeometryModel geomModel_collision, geomModel_visual;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel_collision, dir);
  pinocchio::urdf::buildGeom(model, filename, pinocchio::VISUAL, geomModel_visual, dir);

  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE(build_model_with_joint_from_XML)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");

  // Read file as XML
  std::ifstream file;
  file.open(filename.c_str());
  std::string filestr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, pinocchio::JointModelFreeFlyer(), model);

  BOOST_CHECK(model.nq == 38);
}

BOOST_AUTO_TEST_CASE(build_model_with_joint_from_UDRFTree)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");

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
  const std::string filestr("<?xml version=\"1.0\" encoding=\"utf-8\"?>"
                            "<robot name=\"test\">"
                            "  <link name=\"box\"/>"
                            "</robot>");

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
  const std::string filestr("<?xml version=\"1.0\" encoding=\"utf-8\"?>"
                            "<robot name=\"test\">"
                            "  <link name=\"box\"/>"
                            "</robot>");

  BOOST_CHECK_THROW(
    pinocchio::urdf::buildModelFromXML(filestr, pinocchio::JointModelFreeFlyer(), model),
    std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(check_specific_models)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/baxter_simple.urdf");

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
}

#if defined(PINOCCHIO_WITH_HPP_FCL)
BOOST_AUTO_TEST_CASE(test_geometry_parsing)
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;

  std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/romeo_description/urdf/romeo_small.urdf");
  std::vector<std::string> packageDirs;
  std::string meshDir = PINOCCHIO_MODEL_DIR;
  packageDirs.push_back(meshDir);

  Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs);
  geomModel.addAllCollisionPairs();

  GeometryModel geomModelOther =
    pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs);
  BOOST_CHECK(geomModelOther == geomModel);
}
#endif // if defined(PINOCCHIO_WITH_HPP_FCL)

BOOST_AUTO_TEST_CASE(test_getFrameId_identical_link_and_joint_name)
{
  // This test checks whether the input argument of getFrameId raises an exception when multiple
  // frames with identical names are found. Note, a model that contains a link and a joint with the
  // same name is valid, but the look-up for e.g. getFrameId requires the explicit specification of
  // the FrameType in order to be valid. It resolved
  // https://github.com/stack-of-tasks/pinocchio/issues/1771
  pinocchio::Model model;
  const std::string filename =
    PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/link_and_joint_identical_name.urdf");
  pinocchio::urdf::buildModel(filename, model);

  BOOST_CHECK_THROW(model.getFrameId("base"), std::invalid_argument);
  BOOST_CHECK(model.getFrameId("base", pinocchio::FrameType::BODY) == 1);
  BOOST_CHECK(model.getFrameId("base", pinocchio::FrameType::FIXED_JOINT) == 2);
}

BOOST_AUTO_TEST_SUITE_END()
