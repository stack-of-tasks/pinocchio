//
// Copyright (c) 2015-2022 CNRS INRIA
//

#define BOOST_TEST_MODULE urdf

#include <cstddef>
#include <fstream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/parsers/urdf.hpp"

#ifdef PINOCCHIO_WITH_COLLISION
  #include <coal/collision_object.h>
#endif // PINOCCHIO_WITH_COLLISION

#include <boost/filesystem.hpp>

#include <urdf_parser/urdf_parser.h>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(build_model)
{
  const std::string filename =
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");
  const std::string dir =
    boost::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR).parent_path().parent_path().string();

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, model);
  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, dir);

  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE(build_model_with_root_joint_name)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir =
    boost::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR).parent_path().parent_path().string();

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

#ifdef PINOCCHIO_WITH_COLLISION
  // Check that cylinder is converted into capsule.
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), coal::GEOM_CAPSULE);
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[1].geometry->getNodeType(), coal::GEOM_CONVEX);
#endif // PINOCCHIO_WITH_COLLISION

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
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");

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
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfTree, model);

  BOOST_CHECK(model.nq == 31);
}

BOOST_AUTO_TEST_CASE(build_model_with_joint)
{
  const std::string filename =
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");
  const std::string dir =
    boost::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR).parent_path().parent_path().string();

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
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");

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
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");

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
  const std::string filestr(
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
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

#if defined(PINOCCHIO_WITH_COLLISION)
BOOST_AUTO_TEST_CASE(test_geometry_parsing)
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;

  std::string filename =
    EXAMPLE_ROBOT_DATA_MODEL_DIR + std::string("/romeo_description/urdf/romeo_small.urdf");
  std::vector<std::string> packageDirs;
  const std::string meshDir =
    boost::filesystem::path(EXAMPLE_ROBOT_DATA_MODEL_DIR).parent_path().parent_path().string();
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
#endif // if defined(PINOCCHIO_WITH_COLLISION)

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

BOOST_AUTO_TEST_CASE(test_mimic_parsing)
{
  // Read file as XML
  std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
                      <robot name="test">
                        <link name="base_link"/>
                        <link name="link_1"/>
                        <link name="link_2"/>
                        <link name="link_3"/>
                        <link name="link_4"/>
                        <joint name="joint_1" type="revolute">
                          <origin xyz="1 0 0"/>
                          <axis xyz="0 0 1"/>
                          <parent link="base_link"/>
                          <child link="link_1"/>
                          <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        </joint>
                        <joint name="joint_2" type="revolute">
                          <origin xyz="0 1 0"/>
                          <axis xyz="0 0 1"/>
                          <parent link="link_1"/>
                          <child link="link_2"/>
                          <mimic joint="joint_1" multiplier="-1"/>
                          <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        </joint>
                        <joint name="joint_3" type="continuous">
                          <origin xyz="1 0 0"/>
                          <axis xyz="0 0 1"/>
                          <parent link="link_2"/>
                          <child link="link_3"/>
                        </joint>
                        <joint name="joint_4" type="continuous">
                          <origin xyz="0 1 0"/>
                          <axis xyz="0 0 1"/>
                          <parent link="link_3"/>
                          <child link="link_4"/>
                          <mimic joint="joint_3" multiplier="-2"/>
                        </joint>
                      </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model, false, true);

  BOOST_CHECK(model.nq == 3);
  BOOST_CHECK(model.nv == 2);
  BOOST_CHECK(model.nvExtended == 4);

  BOOST_CHECK(
    model.joints[model.getJointId("joint_2")].idx_q()
    == model.joints[model.getJointId("joint_1")].idx_q());
  BOOST_CHECK(
    model.joints[model.getJointId("joint_4")].idx_q()
    == model.joints[model.getJointId("joint_3")].idx_q());

  // Check non possible mimic pair
  std::string filestr1(R"(<?xml version="1.0" encoding="utf-8"?>
                    <robot name="test">
                      <link name="base_link"/>
                      <link name="link_1"/>
                      <link name="link_2"/>
                      <joint name="joint_1" type="revolute">
                        <origin xyz="1 0 0"/>
                        <axis xyz="0 0 1"/>
                        <parent link="base_link"/>
                        <child link="link_1"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                      </joint>
                      <joint name="joint_4" type="continuous">
                        <origin xyz="0 1 0"/>
                        <axis xyz="0 0 1"/>
                        <parent link="link_1"/>
                        <child link="link_2"/>
                        <mimic joint="joint_1" multiplier="-2"/>
                      </joint>
                    </robot>)");
  pinocchio::Model model1;
  BOOST_CHECK_THROW(
    pinocchio::urdf::buildModelFromXML(filestr1, model1, false, true), std::invalid_argument);

  // unaligned joints
  std::string filestr2(R"(<?xml version="1.0" encoding="utf-8"?>
                    <robot name="test">
                      <link name="base_link"/>
                      <link name="link_1"/>
                      <link name="link_2"/>
                      <link name="link_3"/>
                      <link name="link_4"/>
                      <link name="link_5"/>
                      <joint name="joint_1" type="revolute">
                        <origin xyz="1 0 0"/>
                        <axis xyz="0 0 1"/>
                        <parent link="base_link"/>
                        <child link="link_1"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                      </joint>
                      <joint name="joint_2" type="revolute">
                        <origin xyz="0 1 0"/>
                        <axis xyz="1 0 0"/>
                        <parent link="link_1"/>
                        <child link="link_2"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        <mimic joint="joint_1" multiplier="-3"/>
                      </joint>
                      <joint name="joint_3" type="revolute">
                        <origin xyz="0 1 0"/>
                        <axis xyz="0 1 0"/>
                        <parent link="link_2"/>
                        <child link="link_3"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        <mimic joint="joint_1" multiplier="-3"/>
                      </joint>
                      <joint name="joint_4" type="revolute">
                        <origin xyz="0 1 0"/>
                        <axis xyz="0 0 1"/>
                        <parent link="link_3"/>
                        <child link="link_4"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        <mimic joint="joint_1" multiplier="-3"/>
                      </joint>
                      <joint name="joint_5" type="revolute">
                        <origin xyz="0 1 0"/>
                        <axis xyz="0 0 -1"/>
                        <parent link="link_4"/>
                        <child link="link_5"/>
                        <limit effort="50" lower="0.0" upper="0.8" velocity="0.5"/>
                        <mimic joint="joint_1" multiplier="-2"/>
                      </joint>
                    </robot>)");

  pinocchio::Model model_mimic;

  pinocchio::urdf::buildModelFromXML(filestr2, model_mimic, false, true);

  // RX
  auto j1 = boost::get<pinocchio::JointModelMimic>(model_mimic.joints[2]);
  BOOST_CHECK(boost::get<pinocchio::JointModelRX>(&j1.jmodel()));

  // RY
  auto j2 = boost::get<pinocchio::JointModelMimic>(model_mimic.joints[3]);
  BOOST_CHECK(boost::get<pinocchio::JointModelRY>(&j2.jmodel()));

  // RZ
  auto j3 = boost::get<pinocchio::JointModelMimic>(model_mimic.joints[4]);
  BOOST_CHECK(boost::get<pinocchio::JointModelRZ>(&j3.jmodel()));

  // RU
  auto j4 = boost::get<pinocchio::JointModelMimic>(model_mimic.joints[5]);
  BOOST_CHECK(boost::get<pinocchio::JointModelRevoluteUnaligned>(&j4.jmodel()));
  BOOST_CHECK(
    boost::get<pinocchio::JointModelRevoluteUnaligned>(j4.jmodel())
      .axis.isApprox(-1 * Eigen::Vector3d::UnitZ()));
}

#if PINOCCHIO_URDFDOM_HEADERS_VERSION_AT_LEAST(2, 1, 0) && defined(PINOCCHIO_WITH_COLLISION)

/*
 * This test creates a robot with a capsule joint
 * Tests are performed to check if capusle is well parsed
 */
BOOST_AUTO_TEST_CASE(test_urdf_v11_capsule)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
  <robot name="capsule_robot" version="1.1">
    <link name="base_link"/>
    <link name="arm_link">
      <collision>
        <geometry>
          <capsule radius="0.05" length="0.3"/>
        </geometry>
      </collision>
    </link>
    <joint name="arm_joint" type="revolute">
      <parent link="base_link"/>
      <child link="arm_link"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
    </joint>
  </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  const std::stringstream filestr_stream(filestr);

  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(
    model, filestr_stream, pinocchio::COLLISION, geomModel, PINOCCHIO_MODEL_DIR);

  BOOST_CHECK_EQUAL(geomModel.ngeoms, 1);
  BOOST_CHECK_EQUAL(geomModel.geometryObjects[0].geometry->getNodeType(), coal::GEOM_CAPSULE);

  // check capsule  half length and radius
  const auto * capsule =
    dynamic_cast<const coal::Capsule *>(geomModel.geometryObjects[0].geometry.get());

  BOOST_REQUIRE(capsule != nullptr);
  BOOST_CHECK_EQUAL(capsule->radius, 0.05);
  BOOST_CHECK_SMALL(capsule->halfLength - 0.15, 1e-6);
}
#endif // PINOCCHIO_URDFDOM_HEADERS_VERSION_AT_LEAST(2, 1, 0) && defined(PINOCCHIO_WITH_COLLISION)

#if PINOCCHIO_URDFDOM_HEADERS_VERSION_AT_LEAST(3, 0, 0)

/*
 * This test creates a robot with a revolute joint that has acceleration, deceleration and jerk
 * limits
 * Tests are performed to check Accel and Jerk limits size and values
 */
BOOST_AUTO_TEST_CASE(test_urdf_v12_accel_jerk_revolute)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.2">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="revolute">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"
               acceleration="5.0" deceleration="3.0" jerk="10.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit[0], 5.0);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[0], -3.0);
  BOOST_CHECK_EQUAL(model.upperJerkLimit[0], 10.0);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit[0], -10.0);
}

/*
 * This test creates a robot with a primsatic joint that has acceleration and jerk
 * limits
 * Test are performed to check Accel and Jerk limits size and value
 * Since deceleration is not given, lowerAccelerationLimit = upperAccelerationLimit
 */
BOOST_AUTO_TEST_CASE(test_urdf_v12_accel_jerk_prismatic)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.2">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="prismatic">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-0.785" upper="0.785"
               acceleration="1.0" jerk="5.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit[0], 1.0);
  // no deceleration is given, deceleration = -acceleration
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[0], -1.0);
  BOOST_CHECK_EQUAL(model.upperJerkLimit[0], 5.0);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit[0], -5.0);
}

/*
 * This test creates a robot with a continuous joint that has acceleration, deceleration and jerk
 * limits
 * Tests are performed to check Accel and Jerk limits size and values
 */
BOOST_AUTO_TEST_CASE(test_urdf_v12_accel_jerk_continuous)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.2">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="continuous">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"
               acceleration="5.0" deceleration="3.0" jerk="10.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit[0], 5.0);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[0], -3.0);
  BOOST_CHECK_EQUAL(model.upperJerkLimit[0], 10.0);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit[0], -10.0);
}

/*
 * This test creates a robot with a continuous joint with no accel and jerk limits
 * Tests are performed to check Accel and Jerk limits size and value
 * Since limits are not given, they are initialized equal to infinity
 */
BOOST_AUTO_TEST_CASE(test_urdf_v12_accel_jerk_continuous_no_init)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.2">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="continuous">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  // no limits are given, limits are initialized to infinity
  const double infty = std::numeric_limits<double>::infinity();

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit[0], infty);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[0], -infty);
  BOOST_CHECK_EQUAL(model.upperJerkLimit[0], infty);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit[0], -infty);
}

/*
 * This test creates a robot with a revolute joint that has acceleration, deceleration and jerk
 * limits
 * Test are performed to check Accel and Jerk limits size and value
 * Since urdf file version is 1.0, limits are not read from file and are initialized to infinity
 */
BOOST_AUTO_TEST_CASE(test_urdf_v10_accel_jerk_revolute)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.0">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="revolute">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"
          acceleration="1.0" deceleration="2.0" jerk="5.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  // accel and jerk limits are ignored for version inferior to 1.2
  // limits are initialized to infinity
  const double infty = std::numeric_limits<double>::infinity();

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit[0], infty);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[0], -infty);
  BOOST_CHECK_EQUAL(model.upperJerkLimit[0], infty);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit[0], -infty);
}

/*
 * This test creates a robot with a planar joint that has acceleration, deceleration and jerk
 * limits
 * Test are performed to check Accel and Jerk limits size and value
 * Since the joint type is planar, limits are not read, no matter the version
 */
BOOST_AUTO_TEST_CASE(test_urdf_v12_accel_jerk_planar)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.2">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="planar">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"
          acceleration="1.0" deceleration="2.0" jerk="5.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  // joint limits are not read for planar joint no matter the version
  // limits are initialized to infinity
  const double infty = std::numeric_limits<double>::infinity();

  for (int i = 0; i < model.nv; ++i)
  {
    BOOST_CHECK_EQUAL(model.upperAccelerationLimit[i], infty);
    BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[i], -infty);
    BOOST_CHECK_EQUAL(model.upperJerkLimit[i], infty);
    BOOST_CHECK_EQUAL(model.lowerJerkLimit[i], -infty);
  }
}

/*
 * This test creates a robot with a floating joint that has acceleration, deceleration and jerk
 * limits
 * Test are performed to check Accel and Jerk limits size and value
 * Since the joint type is floating, limits are not read, no matter the version
 */
BOOST_AUTO_TEST_CASE(test_urdf_v10_accel_jerk_floating)
{
  const std::string filestr(R"(<?xml version="1.0" encoding="utf-8"?>
    <robot name="test" version="1.0">
      <link name="base_link"/>
      <link name="link_1"/>
      <joint name="joint_1" type="floating">
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"
          acceleration="1.0" deceleration="2.0" jerk="5.0"/>
      </joint>
    </robot>)");

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(filestr, model);

  BOOST_CHECK_EQUAL(model.upperAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerAccelerationLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.upperJerkLimit.size(), model.nv);
  BOOST_CHECK_EQUAL(model.lowerJerkLimit.size(), model.nv);

  // joint limits are not read for floating joint no matter the version
  // limits are initialized to infinity
  const double infty = std::numeric_limits<double>::infinity();

  for (int i = 0; i < model.nv; ++i)
  {
    BOOST_CHECK_EQUAL(model.upperAccelerationLimit[i], infty);
    BOOST_CHECK_EQUAL(model.lowerAccelerationLimit[i], -infty);
    BOOST_CHECK_EQUAL(model.upperJerkLimit[i], infty);
    BOOST_CHECK_EQUAL(model.lowerJerkLimit[i], -infty);
  }
}
#endif // PINOCCHIO_URDFDOM_HEADERS_VERSION_AT_LEAST(3, 0, 0)
