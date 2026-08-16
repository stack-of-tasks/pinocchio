//
// Copyright (c) 2015-2024 CNRS INRIA
//

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "pinocchio/multibody.hpp"

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/multibody/joint.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/filesystem/fstream.hpp>

typedef ::pinocchio::mjcf::details::MjcfVisitor MjcfVisitor;

namespace
{

  struct TemporaryFileHolder
  {
    TemporaryFileHolder()
    : path(boost::filesystem::temp_directory_path() / boost::filesystem::unique_path())
    {
    }

    ~TemporaryFileHolder()
    {
      boost::filesystem::remove(path);
    }

    TemporaryFileHolder(const TemporaryFileHolder &) = delete;
    TemporaryFileHolder & operator=(const TemporaryFileHolder &) = delete;

    TemporaryFileHolder(TemporaryFileHolder &&) = default;
    TemporaryFileHolder & operator=(TemporaryFileHolder &&) = default;

    std::string name() const
    {
      return path.string();
    }

    boost::filesystem::path path;
  };

} // namespace

static TemporaryFileHolder createTempFile(const std::istringstream & data)
{
  TemporaryFileHolder temp_file;

  // Write the XML data to the temporary file
  boost::filesystem::ofstream file_stream(temp_file.path);
  if (!file_stream)
  {
    std::cerr << "Error opening file for writing" << std::endl;
    exit(EXIT_FAILURE);
  }
  file_stream << data.rdbuf();
  file_stream.close();

  return temp_file;
}

static bool comparePropertyTrees(
  const boost::property_tree::ptree & pt1, const boost::property_tree::ptree & pt2)
{
  // Check if the number of children is the same
  if (pt1.size() != pt2.size())
    return false;

  // Iterate over all children
  for (auto it1 = pt1.begin(), it2 = pt2.begin(); it1 != pt1.end(); ++it1, ++it2)
  {
    // Compare keys
    if (it1->first != it2->first)
      return false;

    // Compare values
    if (it1->second.data() != it2->second.data())
      return false;

    // Recursively compare child trees
    if (!comparePropertyTrees(it1->second, it2->second))
      return false;
  }

  return true;
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

// /// @brief Test for the inertia conversion from mjcf to pinocchio inertia
// /// @param
BOOST_AUTO_TEST_CASE(convert_inertia_fullinertia)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;
  // Parse the XML
  std::istringstream xmlData(R"(
        <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
    )");

  // Create a Boost Property Tree
  boost::property_tree::ptree pt;
  boost::property_tree::read_xml(xmlData, pt);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model;
  MjcfVisitor visitor(model);

  MjcfGraph graph(visitor, "fakeMjcf");

  //  // Try to get the "name" element using get_optional
  pinocchio::Inertia inertia = graph.convertInertiaFromMjcf(pt.get_child("inertial"));

  Matrix3 inertia_matrix;
  // M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)
  inertia_matrix << 0.00315, 8.2904e-7, 0.00015, 8.2904e-7, 0.00388, 8.2299e-6, 0.00015, 8.2299e-6,
    0.004285;

  pinocchio::Inertia real_inertia(0.629769, Vector3(-0.041018, -0.00014, 0.049974), inertia_matrix);
  BOOST_CHECK(inertia.isEqual(real_inertia));
}

// /// @brief Test for the inertia conversion from mjcf to pinocchio inertia
// /// @param
BOOST_AUTO_TEST_CASE(convert_inertia_diaginertia)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;
  // Parse the XML
  std::istringstream xmlData(R"(
        <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        diaginertia="0.00315 0.00388 0.004285"/>
    )");

  // Create a Boost Property Tree
  boost::property_tree::ptree pt;
  boost::property_tree::read_xml(xmlData, pt);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model;
  MjcfVisitor visitor(model);

  MjcfGraph graph(visitor, "fakeMjcf");

  //  // Try to get the "name" element using get_optional
  pinocchio::Inertia inertia = graph.convertInertiaFromMjcf(pt.get_child("inertial"));

  Matrix3 inertia_matrix = Eigen::Matrix3d::Zero();
  inertia_matrix(0, 0) = 0.00315;
  inertia_matrix(1, 1) = 0.00388;
  inertia_matrix(2, 2) = 0.004285;
  pinocchio::Inertia real_inertia(0.629769, Vector3(-0.041018, -0.00014, 0.049974), inertia_matrix);

  BOOST_CHECK(inertia.isApprox(real_inertia, 1e-7));
}

/// @brief Test for geometry computing
BOOST_AUTO_TEST_CASE(geoms_construction)
{
  double pi = boost::math::constants::pi<double>();
  // Parse the XML
  std::istringstream xmlData(R"(<mujoco model="inertiaFromGeom">
                                    <compiler inertiafromgeom="true" />
                                    <worldbody>
                                        <body pos="0 0 0" name="bodyCylinder">
                                            <geom type="cylinder" size=".01" fromto="0 0 0 0 0 .5"/>
                                            <geom type="cylinder" size=".01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            <body pos="0 0 0" name="bodyBox">
                                                <geom type="box" size=".01" fromto="0 0 0 0 0 .5"/>
                                                <geom type="box" size=".01 0.01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodyCapsule">
                                                <geom type="capsule" size=".01" fromto="0 0 0 0 0 .5"/>
                                                <geom type="capsule" size=".01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodySphere">
                                                <geom type="sphere" size=".01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodyEllip">
                                                <geom type="ellipsoid" size=".01" fromto="0 0 0 0 0 .5"/>
                                                <geom type="ellipsoid" size=".01 0.01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                        </body>
                                    </worldbody>
                                  </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());

  // Test Cylinder
  pinocchio::mjcf::details::MjcfBody bodyTest = graph.mapOfBodies.at("bodyCylinder");
  double mass = 1000 * std::pow(0.01, 2) * 0.25 * pi * 2;
  pinocchio::Inertia inertTest = pinocchio::Inertia::FromCylinder(mass, 0.01, 0.25 * 2);
  Eigen::Vector2d sizeCy;
  sizeCy << 0.01, 0.25;
  for (const auto & geom : bodyTest.geomChildren)
  {
    // Check size
    BOOST_CHECK(geom.size == sizeCy);
    // Check inertia
    BOOST_CHECK(geom.geomInertia.isApprox(inertTest));
  }
  // Test Box
  bodyTest = graph.mapOfBodies.at("bodyBox");
  mass = 1000 * 0.02 * 0.02 * 0.5;
  inertTest = pinocchio::Inertia::FromBox(mass, 0.02, 0.02, 0.5);
  Eigen::Vector3d sizeB;
  sizeB << 0.02, 0.02, .5;
  for (const auto & geom : bodyTest.geomChildren)
  {
    // Check size
    BOOST_CHECK(geom.size == sizeB);
    // Check inertia
    BOOST_CHECK(geom.geomInertia.isApprox(inertTest));
  }
  // Test Capsule
  bodyTest = graph.mapOfBodies.at("bodyCapsule");
  mass = 1000 * (4.0 / 3 * pi * std::pow(0.01, 3) + 2 * pi * std::pow(0.01, 2) * 0.25);
  inertTest = pinocchio::Inertia::FromCapsule(mass, 0.01, 0.25 * 2);
  Eigen::Vector2d sizeCa;
  sizeCa << 0.01, 0.25;
  for (const auto & geom : bodyTest.geomChildren)
  {
    // Check size
    BOOST_CHECK(geom.size == sizeCa);
    // Check inertia
    BOOST_CHECK(geom.geomInertia.isApprox(inertTest));
  }
  // Test Sphere
  bodyTest = graph.mapOfBodies.at("bodySphere");
  mass = 1000 * 4.0 / 3 * pi * std::pow(0.01, 3);
  inertTest = pinocchio::Inertia::FromSphere(mass, 0.01);
  Eigen::VectorXd sizeS(1);
  sizeS << 0.01;
  for (const auto & geom : bodyTest.geomChildren)
  {
    // Check size
    BOOST_CHECK(geom.size == sizeS);
    // Check inertia
    BOOST_CHECK(geom.geomInertia.isApprox(inertTest));
  }
  // Test Ellipsoid
  bodyTest = graph.mapOfBodies.at("bodyEllip");
  mass = 1000 * 4.0 / 3 * pi * 0.01 * 0.01 * 0.25;
  inertTest = pinocchio::Inertia::FromEllipsoid(mass, 0.01, 0.01, 0.25);
  Eigen::Vector3d sizeEl;
  sizeEl << 0.01, 0.01, 0.25;
  for (const auto & geom : bodyTest.geomChildren)
  {
    // Check size
    BOOST_CHECK(geom.size == sizeEl);
    // Check inertia
    BOOST_CHECK(geom.geomInertia.isApprox(inertTest));
  }
}

/// @brief Test for computing inertia from geoms
/// @param
BOOST_AUTO_TEST_CASE(inertia_from_geom)
{
  // Parse the XML
  std::istringstream xmlData(R"(<mujoco model="inertiaFromGeom">
                                    <compiler inertiafromgeom="true" />
                                    <worldbody>
                                        <body pos="0 0 0" name="body0">
                                            <geom type="box" size=".01 .01 .01" pos="0 0 0.01"/>
                                            <geom type="box" size=".01 .01 .01" pos="0 0 0.03"/>
                                        </body>
                                    </worldbody>
                                  </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  // only one inertias since only one body
  pinocchio::Inertia inertBody = model_m.inertias[0];

  double massBigBox = 1000 * 0.02 * 0.02 * 0.04; // density * volume
  pinocchio::Inertia inertBigBox = pinocchio::Inertia::FromBox(massBigBox, 0.02, 0.02, 0.04);
  // Create a frame on the bottom face of BigBox and express inertia
  Eigen::Vector3d pl;
  pl << 0, 0, 0.02;
  pinocchio::SE3 placementBigBox(Eigen::Matrix3d::Identity(), pl);
  inertBigBox = placementBigBox.act(inertBigBox);

  BOOST_CHECK(inertBigBox.isApprox(inertBody));
}

// /// @brief Test for the pose conversion from mjcf model to pinocchio
// /// @param
BOOST_AUTO_TEST_CASE(convert_orientation)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
        <compiler angle="radian" eulerseq="xyz"/>
        <quaternion pos="0.3 0.2 0.5" quat="1 -1 0 0"/>
        <axis pos="0.3 0.2 0.5" axisangle="-1 0 0 1.5707963"/>
        <euler pos="0.3 0.2 0.5" euler="-1.57079633 0 0"/>
        <xyaxes pos="0.3 0.2 0.5" xyaxes="1 0 0 0 0 -1"/>
        <zaxis pos="0.3 0.2 0.5" zaxis="0 1 0"/>
    )");

  // Create a Boost Property Tree
  boost::property_tree::ptree pt;
  boost::property_tree::read_xml(xmlData, pt);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model;
  MjcfVisitor visitor(model);

  MjcfGraph graph(visitor, "fakeMjcf");

  graph.parseCompiler(pt.get_child("compiler"));

  pinocchio::SE3 placement_q = graph.convertPosition(pt.get_child("quaternion"));
  pinocchio::SE3 placement_a = graph.convertPosition(pt.get_child("axis"));
  pinocchio::SE3 placement_e = graph.convertPosition(pt.get_child("euler"));
  pinocchio::SE3 placement_xy = graph.convertPosition(pt.get_child("xyaxes"));
  pinocchio::SE3 placement_z = graph.convertPosition(pt.get_child("zaxis"));

  Matrix3 rotation_matrix;
  rotation_matrix << 1., 0., 0., 0., 0., 1., 0., -1., 0.;

  pinocchio::SE3 real_placement(rotation_matrix, Vector3(0.3, 0.2, 0.5));

  BOOST_CHECK(placement_q.isApprox(real_placement, 1e-7));
  BOOST_CHECK(placement_e.isApprox(real_placement, 1e-7));
  BOOST_CHECK(placement_a.isApprox(real_placement, 1e-7));
  BOOST_CHECK(placement_xy.isApprox(real_placement, 1e-7));
  BOOST_CHECK(placement_z.isApprox(real_placement, 1e-7));
}

/// @brief Test if merging default classes works
/// @param
BOOST_AUTO_TEST_CASE(merge_default)
{
  namespace pt = boost::property_tree;
  std::istringstream xmlIn(R"(
            <default>
                <default class="mother">
                    <joint A="a0" B="b0" />
                    <geom A="a0" />
                    <default class="layer1">
                        <joint A="a1" C="c1" />
                        <default class="layer2">
                            <joint B="b2" />
                            <geom C="c2" />
                        </default>
                    </default>
                    <default class="layerP">
                        <joint K="b2" />
                        <site  H="f1"/>
                    </default>
                </default>
            </default>)");

  // Create a Boost Property Tree
  pt::ptree ptr;
  pt::read_xml(xmlIn, ptr, pt::xml_parser::trim_whitespace);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model;
  MjcfVisitor visitor(model);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseDefault(ptr.get_child("default"), ptr, "default");

  std::unordered_map<std::string, pt::ptree> TrueMap;

  std::istringstream xml1(R"(<default class="mother">
                                    <joint A="a0" B="b0" />
                                    <geom A="a0" />
                                    <default class="layer1">
                                        <joint A="a1" C="c1" />
                                        <default class="layer2">
                                            <joint B="b2" />
                                            <geom C="c2" />
                                        </default>
                                    </default>
                                    <default class="layerP">
                                        <joint K="b2" />
                                        <site  H="f1"/>
                                    </default>
                                </default>)");
  pt::ptree p1;
  pt::read_xml(xml1, p1, pt::xml_parser::trim_whitespace);
  BOOST_CHECK(
    comparePropertyTrees(graph.mapOfClasses.at("mother").classElement, p1.get_child("default")));

  std::istringstream xml2(R"(<default class="layer1">
                                    <joint A="a1" B="b0" C="c1" />
                                    <default class="layer2">
                                        <joint B="b2" />
                                        <geom C="c2" />
                                    </default>
                                    <geom A="a0" />
                                </default>)");
  pt::ptree p2;
  pt::read_xml(xml2, p2, pt::xml_parser::trim_whitespace);
  BOOST_CHECK(
    comparePropertyTrees(graph.mapOfClasses.at("layer1").classElement, p2.get_child("default")));
  std::string name = "layer1";
  TrueMap.insert(std::make_pair(name, p2.get_child("default")));

  std::istringstream xml3(R"(<default class="layer2">
                                    <joint A="a1" B="b2" C="c1"/>
                                    <geom A="a0" C="c2"/>
                                </default>)");
  pt::ptree p3;
  pt::read_xml(xml3, p3, pt::xml_parser::trim_whitespace);
  BOOST_CHECK(
    comparePropertyTrees(graph.mapOfClasses.at("layer2").classElement, p3.get_child("default")));

  std::istringstream xml4(R"(<default class="layerP">
                                    <joint A="a0" B="b0" K="b2"/>
                                    <site H="f1"/>
                                    <geom A="a0"/>
                                </default>)");
  pt::ptree p4;
  pt::read_xml(xml4, p4, pt::xml_parser::trim_whitespace);
  BOOST_CHECK(
    comparePropertyTrees(graph.mapOfClasses.at("layerP").classElement, p4.get_child("default")));
}

#ifdef PINOCCHIO_WITH_URDFDOM
// @brief Test to check that default classes and child classes are taken into account
BOOST_AUTO_TEST_CASE(parse_default_class)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::vector<pinocchio::FrameAnchorConstraintModel> frame_anchor_constraint_models;
  std::vector<pinocchio::PointAnchorConstraintModel> point_anchor_constraint_models;
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_mjcf.xml");

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(
    filename, model_m, point_anchor_constraint_models, frame_anchor_constraint_models);

  std::string file_u = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_mjcf.urdf");
  pinocchio::Model model_u;
  pinocchio::urdf::buildModel(file_u, model_u);

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());
  idx = model_u.addJoint(
    size_t(model_u.njoints - 1), pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(),
    "joint3");
  model_u.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], model_u.joints[i]);
}
#endif // PINOCCHIO_WITH_URDFDOM

/// @brief Test to see if path options work
BOOST_AUTO_TEST_CASE(parse_dirs_no_strippath)
{
#ifdef _WIN32
  std::istringstream xmlDataNoStrip(R"(<mujoco model="parseDirs">
                                    <compiler meshdir="meshes" texturedir="textures"/>
                                    <asset>
                                        <texture name="testTexture" file="texture.png" type="2d"/>
                                        <material name="matTest" texture="testTexture"/>
                                        <mesh file="C:/auto/mesh.obj"/>
                                    </asset>
                                  </mujoco>)");
#else
  std::istringstream xmlDataNoStrip(R"(<mujoco model="parseDirs">
                                    <compiler meshdir="meshes" texturedir="textures"/>
                                    <asset>
                                        <texture name="testTexture" file="texture.png" type="2d"/>
                                        <material name="matTest" texture="testTexture"/>
                                        <mesh file="/auto/mesh.obj"/>
                                    </asset>
                                  </mujoco>)");
#endif

  auto namefile = createTempFile(xmlDataNoStrip);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "/fakeMjcf/fake.xml");
  graph.parseGraphFromXML(namefile.name());

  // Test texture
  pinocchio::mjcf::details::MjcfTexture text = graph.mapOfTextures.at("testTexture");
  BOOST_CHECK_EQUAL(text.textType, "2d");
  BOOST_CHECK_EQUAL(
    boost::filesystem::path(text.filePath).generic_string(), "/fakeMjcf/textures/texture.png");
  // Test Material
  pinocchio::mjcf::details::MjcfMaterial mat = graph.mapOfMaterials.at("matTest");
  BOOST_CHECK_EQUAL(mat.texture, "testTexture");
  // Test Meshes
  pinocchio::mjcf::details::MjcfMesh mesh = graph.mapOfMeshes.at("mesh");
#ifdef _WIN32
  BOOST_CHECK_EQUAL(boost::filesystem::path(mesh.filePath).generic_string(), "C:/auto/mesh.obj");
#else
  BOOST_CHECK_EQUAL(boost::filesystem::path(mesh.filePath).generic_string(), "/auto/mesh.obj");
#endif
}

/// @brief Test strippath option
/// @param
BOOST_AUTO_TEST_CASE(parse_dirs_strippath)
{
  std::istringstream xmlDataNoStrip(R"(<mujoco model="parseDirs">
                                    <compiler meshdir="meshes" texturedir="textures" strippath="true"/>
                                    <asset>
                                        <mesh file="/auto/mesh.obj"/>
                                    </asset>
                                  </mujoco>)");

  auto namefile = createTempFile(xmlDataNoStrip);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "/fakeMjcf/fake.xml");
  graph.parseGraphFromXML(namefile.name());

  // Test Meshes
  pinocchio::mjcf::details::MjcfMesh mesh = graph.mapOfMeshes.at("mesh");
  BOOST_CHECK_EQUAL(
    boost::filesystem::path(mesh.filePath).generic_string(), "/fakeMjcf/meshes/mesh.obj");
}

// Test for parsing Revolute
BOOST_AUTO_TEST_CASE(parse_RX)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="model_RX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelRX;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  idx = modelRX.addJoint(0, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "rx");
  modelRX.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelRX.joints[i]);
}

// Test for parsing prismatic
BOOST_AUTO_TEST_CASE(parse_PX)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="model_PX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelPX;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  idx = modelPX.addJoint(0, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "px");
  modelPX.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelPX.joints[i]);
}

// Test parsing sphere
BOOST_AUTO_TEST_CASE(parse_Sphere)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="model_Sphere">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="ball"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelS;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  idx = modelS.addJoint(0, pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(), "s");
  modelS.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelS.joints[i]);
}

// Test parsing free flyer
BOOST_AUTO_TEST_CASE(parse_Free)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="model_Free">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <freejoint name="joint1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelF;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  idx = modelF.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "f");
  modelF.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelF.joints[i]);
}

// Test for Composite RXRY
BOOST_AUTO_TEST_CASE(parse_composite_RXRY)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="composite_RXRY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0"/>
                            <joint name="joint2" type="hinge" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelRXRY;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  pinocchio::JointModelComposite joint_model_RXRY;
  joint_model_RXRY.addJoint(pinocchio::JointModelRX());
  joint_model_RXRY.addJoint(pinocchio::JointModelRY());

  idx = modelRXRY.addJoint(0, joint_model_RXRY, pinocchio::SE3::Identity(), "rxry");
  modelRXRY.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelRXRY.joints[i]);
}

// Test for Composite PXPY
BOOST_AUTO_TEST_CASE(parse_composite_PXPY)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="composite_PXPY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="slide" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelPXPY;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  pinocchio::JointModelComposite joint_model_PXPY;
  joint_model_PXPY.addJoint(pinocchio::JointModelPX());
  joint_model_PXPY.addJoint(pinocchio::JointModelPY());

  idx = modelPXPY.addJoint(0, joint_model_PXPY, pinocchio::SE3::Identity(), "pxpy");
  modelPXPY.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelPXPY.joints[i]);
}

// Test for Composite PXRY
BOOST_AUTO_TEST_CASE(parse_composite_PXRY)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="composite_PXRY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="hinge" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelPXRY;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  pinocchio::JointModelComposite joint_model_PXRY;
  joint_model_PXRY.addJoint(pinocchio::JointModelPX());
  joint_model_PXRY.addJoint(pinocchio::JointModelRY());

  idx = modelPXRY.addJoint(0, joint_model_PXRY, pinocchio::SE3::Identity(), "pxry");
  modelPXRY.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelPXRY.joints[i]);
}

// Test for Composite PXSphere
BOOST_AUTO_TEST_CASE(parse_composite_PXSphere)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="composite_PXSphere">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="ball"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m, modelPXSphere;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::Model::JointIndex idx;
  pinocchio::Inertia inertia(0, Vector3(0.0, 0., 0.0), Matrix3::Identity());

  pinocchio::JointModelComposite joint_model_PXSphere;
  joint_model_PXSphere.addJoint(pinocchio::JointModelPX());
  joint_model_PXSphere.addJoint(pinocchio::JointModelSpherical());

  idx = modelPXSphere.addJoint(0, joint_model_PXSphere, pinocchio::SE3::Identity(), "pxsphere");
  modelPXSphere.appendBodyToJoint(idx, inertia);

  for (size_t i = 0; i < size_t(model_m.njoints); i++)
    BOOST_CHECK_EQUAL(model_m.joints[i], modelPXSphere.joints[i]);
}

// Test loading a mujoco composite and compare body position with mujoco results
BOOST_AUTO_TEST_CASE(parse_composite_Mujoco_comparison)
{
  using namespace pinocchio;
  std::string filename =
    PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_composite.xml");

  pinocchio::Model model;
  pinocchio::mjcf::buildModel(filename, model);

  Data data(model);
  Eigen::Vector3d q;
  q << 1.57079633, 0.3, 0.5;
  framesForwardKinematics(model, data, q);

  FrameIndex f_id = model.getFrameId("body1");
  SE3 pinPos = data.oMf[f_id];

  Eigen::Matrix3d refOrient;
  refOrient << 0, -.9553, .2955, 1, 0, 0, 0, .2955, .9553;
  Eigen::Vector3d pos;
  pos << .8522, 2, 0.5223;
  BOOST_CHECK(pinPos.isApprox(SE3(refOrient, pos), 1e-4));

  f_id = model.getFrameId("body2");
  pinPos = data.oMf[f_id];

  refOrient << 0, -.9553, .2955, 1, 0, 0, 0, .2955, .9553;

  pos << .8522, 4, 0.5223;
  BOOST_CHECK(pinPos.isApprox(SE3(refOrient, pos), 1e-4));
}

// Test laoding a model with a spherical joint and verify that keyframe is valid
BOOST_AUTO_TEST_CASE(adding_keyframes)
{
  std::istringstream xmlData(R"(
            <mujoco model="testKeyFrame">
                <default>
                    <position ctrllimited="true" ctrlrange="-.1 .1" kp="30"/>
                    <default class="joint">
                    <geom type="cylinder" size=".006" fromto="0 0 0 0 0 .05" rgba=".9 .6 1 1"/>
                    </default>
                </default>
                <worldbody>
                    <body name="body1">
                        <freejoint/>
                        <geom type="capsule" size=".01" fromto="0 0 0 .2 0 0"/>
                        <body pos=".2 0 0" name="body2">
                            <joint type="ball" damping=".1"/>
                            <geom type="capsule" size=".01" fromto="0 -.15 0 0 0 0"/>
                        </body>
                    </body>
                </worldbody>
                <keyframe>
                    <key name="test"
                    qpos="0 0 0.596
                        0.988015 0 0.154359 0
                        0.988015 0 0.154359 0
                        "/>
                </keyframe>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd vect_model = model_m.referenceConfigurations.at("test");

  Eigen::VectorXd vect_ref(model_m.nq);
  vect_ref << 0, 0, 0.596, 0, 0.154359, 0, 0.988015, 0, 0.154359, 0, 0.988015;
  ::pinocchio::normalize(model_m, vect_ref);

  BOOST_CHECK(vect_model.size() == vect_ref.size());
  BOOST_CHECK(vect_model == vect_ref);
}

// Test laoding a model with a spherical joint and verify that keyframe is valid
BOOST_AUTO_TEST_CASE(adding_keyframes_with_ref_and_freejoint)
{
  std::istringstream xmlData(R"(
            <mujoco model="testKeyFrame">
                <default>
                    <position ctrllimited="true" ctrlrange="-.1 .1" kp="30"/>
                    <default class="joint">
                    <geom type="cylinder" size=".006" fromto="0 0 0 0 0 .05" rgba=".9 .6 1 1"/>
                    </default>
                </default>
                <worldbody>
                    <body name="body1" pos="0 0 1.1">
                        <freejoint/>
                        <geom type="capsule" size=".01" fromto="0 0 0 .2 0 0"/>
                        <body pos=".2 0 0" name="body2">
                            <joint type="ball" damping=".1"/>
                            <geom type="capsule" size=".01" fromto="0 -.15 0 0 0 0"/>
                        </body>
                    </body>
                </worldbody>
                <keyframe>
                    <key name="test"
                    qpos="0 0 0.596
                        0.988015 0 0.154359 0
                        0.988015 0 0.154359 0"/>
                </keyframe>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::Vector3d freejoint_trans_test = Eigen::Vector3d::Zero();
  Eigen::Vector3d freejoint_trans = model_m.jointPlacements[1].translation();
  BOOST_CHECK(freejoint_trans_test == freejoint_trans);

  Eigen::VectorXd vect_model = model_m.referenceConfigurations.at("test");

  Eigen::VectorXd vect_ref(model_m.nq);
  vect_ref << 0, 0, 0.596, 0, 0.154359, 0, 0.988015, 0, 0.154359, 0, 0.988015;
  ::pinocchio::normalize(model_m, vect_ref);

  BOOST_CHECK(vect_model.size() == vect_ref.size());
  BOOST_CHECK(vect_model == vect_ref);
}

// Test on which joints inertias are append
BOOST_AUTO_TEST_CASE(joint_and_inertias)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="testJointInertia">
                <worldbody>
                    <body name="body1">
                        <freejoint/>
                        <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
                        diaginertia="0.00315 0.00388 0.004285"/>
                    </body>
                </worldbody>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  BOOST_CHECK(model_m.inertias[0].isApprox(pinocchio::Inertia::Zero()));

  Matrix3 inertia_matrix = Eigen::Matrix3d::Zero();
  inertia_matrix(0, 0) = 0.00315;
  inertia_matrix(1, 1) = 0.00388;
  inertia_matrix(2, 2) = 0.004285;
  pinocchio::Inertia real_inertia(0.629769, Vector3(-0.041018, -0.00014, 0.049974), inertia_matrix);

  BOOST_CHECK(model_m.inertias[1].isApprox(real_inertia));
}

BOOST_AUTO_TEST_CASE(armature)
{
  std::istringstream xmlData(R"(
            <mujoco model="model_RX">
                <default>
                  <joint armature="1" damping="1" limited="true"/>
                </default>
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0" armature="1.3"/>
                            <joint name="joint2" type="hinge" axis="0 1 0" armature="2.4"/>
                            <joint name="joint3" type="hinge" axis="0 0 1" armature="0.4"/>
                            <body pos=".2 0 0" name="body2">
                              <joint type="ball"/>
                        </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  Eigen::VectorXd armature_real(model_m.nv);
  armature_real << 1.3, 2.4, 0.4, 1, 1, 1;

  for (Eigen::Index i = 0; i < model_m.nv; i++)
    BOOST_CHECK_EQUAL(model_m.armature[i], armature_real[i]);
}

// Test reference positions and how it's included in keyframe
BOOST_AUTO_TEST_CASE(reference_positions)
{
  std::istringstream xmlData(R"(
            <mujoco model="testRefPose">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="body1">
                        <body pos=".2 0 0" name="body2">
                            <joint type="slide" ref="0.14"/>
                            <body pos=".2 0 0" name="body3">
                                <joint type="hinge" ref="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
                <keyframe>
                    <key name="test" qpos=".8 .5"/>
                </keyframe>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd vect_model = model_m.referenceConfigurations.at("test");
  Eigen::VectorXd vect_ref(model_m.nq);
  vect_ref << 0.8, 0.5;

  BOOST_CHECK(vect_model.size() == vect_ref.size());
  BOOST_CHECK(vect_model == vect_ref);
}

// Test keyframe and composite joint
BOOST_AUTO_TEST_CASE(keyframe_and_composite)
{
  std::istringstream xmlData(R"(
            <mujoco model="keyframeComposite">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="body1">
                        <body pos=".2 0 0" name="body2">
                            <joint type="slide"/>
                            <joint type="hinge"/>
                            <body pos=".2 0 0" name="body3">
                                <joint type="hinge"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
                <keyframe>
                    <key name="test" qpos=".8 .5 .5"/>
                </keyframe>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd vect_model = model_m.referenceConfigurations.at("test");
  Eigen::VectorXd vect_ref(model_m.nq);
  vect_ref << 0.8, 0.5, 0.5;

  BOOST_CHECK(vect_model.size() == vect_ref.size());
  BOOST_CHECK(vect_model == vect_ref);
}

// Test site of mjcf model
BOOST_AUTO_TEST_CASE(adding_site)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="site">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="body1">
                        <body pos=".2 0 0" name="body2">
                            <joint type="hinge"/>
                            <body pos=".2 0 0" name="body3">
                                <joint type="hinge"/>
                                <site name="testSite" pos="0.03 0 -0.05"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  pinocchio::Data data(model_m);

  Matrix3 rotation_matrix;
  rotation_matrix << 1., 0., 0., 0., 1., 0., 0., 0., 1.;
  pinocchio::SE3 real_placement(rotation_matrix, Vector3(0.03, 0, -0.05));

  BOOST_CHECK(model_m.frames[model_m.getFrameId("testSite")].placement.isApprox(real_placement));
  BOOST_CHECK(
    model_m.frames[model_m.getFrameId("testSite")].parentJoint
    == model_m.frames[model_m.getFrameId("body3")].parentJoint);
}

/// @brief test that a fixed model is well parsed
/// @param
BOOST_AUTO_TEST_CASE(build_model_no_root_joint)
{
  using namespace pinocchio;
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.xml");

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(filename, model_m);

  BOOST_CHECK_EQUAL(model_m.nq, 29);
}

/// @brief test that a fixed-base root body's own pos/quat is not dropped,
/// and that it is correctly propagated to its children (regression test for #2782)
/// @param
BOOST_AUTO_TEST_CASE(build_model_fixed_base_root_body_placement)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;

  std::istringstream xmlData(R"(
            <mujoco model="fixed_base_test">
                <worldbody>
                    <body name="base" pos="1 2 3" quat="0.7071068 0 0 0.7071068">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="link1" pos="0 0 0.5">
                            <joint name="j1" type="hinge" axis="0 1 0"/>
                            <geom type="box" size="0.05 0.05 0.2"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  pinocchio::Data data(model_m);
  pinocchio::framesForwardKinematics(model_m, data, pinocchio::neutral(model_m));

  Matrix3 rotation_matrix;
  rotation_matrix << 0., -1., 0., 1., 0., 0., 0., 0., 1.;

  const pinocchio::SE3 expected_base(rotation_matrix, Vector3(1., 2., 3.));
  const pinocchio::SE3 expected_link1(rotation_matrix, Vector3(1., 2., 3.5));

  const pinocchio::FrameIndex baseFrameId = model_m.getFrameId("base", pinocchio::BODY);
  const pinocchio::FrameIndex link1FrameId = model_m.getFrameId("link1", pinocchio::BODY);

  BOOST_CHECK(data.oMf[baseFrameId].isApprox(expected_base, 1e-6));
  BOOST_CHECK(data.oMf[link1FrameId].isApprox(expected_link1, 1e-6));

  // The base geom has no offset of its own, so its inertia must be carried by
  // the exact same placement as the base frame.
  const double massBase = 1000 * 0.2 * 0.2 * 0.2; // density * volume
  const pinocchio::Inertia expected_universe_inertia =
    expected_base.act(pinocchio::Inertia::FromBox(massBase, 0.2, 0.2, 0.2));

  BOOST_CHECK(model_m.inertias[0].isApprox(expected_universe_inertia, 1e-6));
}

double degreesToRadian(double degrees)
{
  return degrees * (M_PI / 180.0);
}

BOOST_AUTO_TEST_CASE(slide_joint_limits)
{
  std::istringstream xmlData(R"(
            <mujoco model="model_PX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0" range="-16.34 17.2" margin="1.2345" actuatorfrcrange="1.5 4.8" frictionloss="11.6"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd lower_position_limit(model_m.lowerPositionLimit);
  lower_position_limit << -degreesToRadian(16.34);
  Eigen::VectorXd upper_position_limit(model_m.upperPositionLimit);
  upper_position_limit << degreesToRadian(17.2);
  Eigen::VectorXd position_limit_margin(model_m.positionLimitMargin);
  position_limit_margin << degreesToRadian(1.2345);
  Eigen::VectorXd min_dry_friction(model_m.lowerDryFrictionLimit);
  min_dry_friction << -11.6;
  Eigen::VectorXd max_dry_friction(model_m.upperDryFrictionLimit);
  max_dry_friction << 11.6;
  Eigen::VectorXd min_effort(model_m.lowerEffortLimit);
  min_effort << 1.5;
  Eigen::VectorXd max_effort(model_m.upperEffortLimit);
  max_effort << 4.8;

  BOOST_CHECK(lower_position_limit == model_m.lowerPositionLimit);
  BOOST_CHECK(upper_position_limit == model_m.upperPositionLimit);
  BOOST_CHECK(position_limit_margin == model_m.positionLimitMargin);
  BOOST_CHECK(min_dry_friction == model_m.lowerDryFrictionLimit);
  BOOST_CHECK(max_dry_friction == model_m.upperDryFrictionLimit);
  BOOST_CHECK(min_effort == model_m.lowerEffortLimit);
  BOOST_CHECK(max_effort == model_m.upperEffortLimit);
}

BOOST_AUTO_TEST_CASE(hinge_joint_limits)
{
  std::istringstream xmlData(R"(
            <mujoco model="model_PX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0" range="-16.34 17.2" margin="1.2345" actuatorfrcrange="1.5 4.8" frictionloss="11.6"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd lower_position_limit(model_m.lowerPositionLimit);
  lower_position_limit << -degreesToRadian(16.34);
  Eigen::VectorXd upper_position_limit(model_m.upperPositionLimit);
  upper_position_limit << degreesToRadian(17.2);
  Eigen::VectorXd position_limit_margin(model_m.positionLimitMargin);
  position_limit_margin << degreesToRadian(1.2345);
  Eigen::VectorXd min_dry_friction(model_m.lowerDryFrictionLimit);
  min_dry_friction << -11.6;
  Eigen::VectorXd max_dry_friction(model_m.upperDryFrictionLimit);
  max_dry_friction << 11.6;
  Eigen::VectorXd min_effort(model_m.lowerEffortLimit);
  min_effort << 1.5;
  Eigen::VectorXd max_effort(model_m.upperEffortLimit);
  max_effort << 4.8;

  BOOST_CHECK(lower_position_limit == model_m.lowerPositionLimit);
  BOOST_CHECK(upper_position_limit == model_m.upperPositionLimit);
  BOOST_CHECK(position_limit_margin == model_m.positionLimitMargin);
  BOOST_CHECK(min_dry_friction == model_m.lowerDryFrictionLimit);
  BOOST_CHECK(max_dry_friction == model_m.upperDryFrictionLimit);
  BOOST_CHECK(min_effort == model_m.lowerEffortLimit);
  BOOST_CHECK(max_effort == model_m.upperEffortLimit);
}

BOOST_AUTO_TEST_CASE(hinge_and_slide_joints_limits)
{
  std::istringstream xmlData(R"(
            <mujoco model="model_PX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0" range="-16.34 17.2" margin="1.2345" actuatorfrcrange="1.5 4.8" frictionloss="11.6"/>
                            <body name="link2" pos="0 0 0">
                              <joint name="joint2" type="slide" axis="1 0 0" range="-18.32 19.1" margin="2.3366" actuatorfrcrange="-6.87 -4.8" frictionloss="0.17"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  Eigen::VectorXd lower_position_limit(model_m.lowerPositionLimit);
  lower_position_limit << -degreesToRadian(16.34), -degreesToRadian(18.32);
  Eigen::VectorXd upper_position_limit(model_m.upperPositionLimit);
  upper_position_limit << degreesToRadian(17.2), degreesToRadian(19.1);
  Eigen::VectorXd position_limit_margin(model_m.positionLimitMargin);
  position_limit_margin << degreesToRadian(1.2345), degreesToRadian(2.3366);
  Eigen::VectorXd min_dry_friction(model_m.lowerDryFrictionLimit);
  min_dry_friction << -11.6, -0.17;
  Eigen::VectorXd max_dry_friction(model_m.upperDryFrictionLimit);
  max_dry_friction << 11.6, 0.17;
  Eigen::VectorXd min_effort(model_m.lowerEffortLimit);
  min_effort << 1.5, -6.87;
  Eigen::VectorXd max_effort(model_m.upperEffortLimit);
  max_effort << 4.8, -4.8;

  BOOST_CHECK(lower_position_limit == model_m.lowerPositionLimit);
  BOOST_CHECK(upper_position_limit == model_m.upperPositionLimit);
  BOOST_CHECK(position_limit_margin == model_m.positionLimitMargin);
  BOOST_CHECK(min_dry_friction == model_m.lowerDryFrictionLimit);
  BOOST_CHECK(max_dry_friction == model_m.upperDryFrictionLimit);
  BOOST_CHECK(min_effort == model_m.lowerEffortLimit);
  BOOST_CHECK(max_effort == model_m.upperEffortLimit);
}

BOOST_AUTO_TEST_CASE(build_model_with_root_joint_name)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.xml");

  pinocchio::Model model;
  pinocchio::mjcf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model);
  BOOST_CHECK(model.names[1] == "root_joint");

  pinocchio::Model model_name;
  const std::string name_ = "freeFlyer_joint";
  pinocchio::mjcf::buildModel(filename, pinocchio::JointModelFreeFlyer(), name_, model_name);
  BOOST_CHECK(model_name.names[1] == name_);
}

#ifdef PINOCCHIO_WITH_URDFDOM
/// @brief Test all the data of the humanoid model (Need to find the urdf yet)
/// @param
BOOST_AUTO_TEST_CASE(compare_to_urdf)
{
  using namespace pinocchio;

  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.xml");

  Model model_m;
  pinocchio::mjcf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_m);

  const std::string filename_urdf = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  pinocchio::Model model_urdf;
  pinocchio::urdf::buildModel(filename_urdf, pinocchio::JointModelFreeFlyer(), model_urdf);

  BOOST_CHECK(model_urdf.nq == model_m.nq);
  BOOST_CHECK(model_urdf.nv == model_m.nv);
  BOOST_CHECK(model_urdf.njoints == model_m.njoints);
  BOOST_CHECK(model_urdf.nbodies == model_m.nbodies);
  BOOST_CHECK(model_urdf.nframes == model_m.nframes);
  BOOST_CHECK(model_urdf.parents == model_m.parents);
  BOOST_CHECK(model_urdf.children == model_m.children);
  BOOST_CHECK(model_urdf.names == model_m.names);
  BOOST_CHECK(model_urdf.subtrees == model_m.subtrees);
  BOOST_CHECK(model_urdf.gravity == model_m.gravity);
  BOOST_CHECK(model_urdf.name == model_m.name);
  BOOST_CHECK(model_urdf.idx_qs == model_m.idx_qs);
  BOOST_CHECK(model_urdf.nqs == model_m.nqs);
  BOOST_CHECK(model_urdf.idx_vs == model_m.idx_vs);
  BOOST_CHECK(model_urdf.nvs == model_m.nvs);

  BOOST_CHECK(model_urdf.armature.size() == model_m.armature.size());

  BOOST_CHECK(model_urdf.armature == model_m.armature);
  BOOST_CHECK(model_urdf.upperDryFrictionLimit.size() == model_m.upperDryFrictionLimit.size());
  BOOST_CHECK(model_urdf.upperDryFrictionLimit == model_m.upperDryFrictionLimit);

  BOOST_CHECK(model_urdf.damping.size() == model_m.damping.size());

  BOOST_CHECK(model_urdf.damping == model_m.damping);

  BOOST_CHECK(model_urdf.rotorInertia.size() == model_m.rotorInertia.size());

  BOOST_CHECK(model_urdf.rotorInertia == model_m.rotorInertia);

  BOOST_CHECK(model_urdf.rotorGearRatio.size() == model_m.rotorGearRatio.size());

  BOOST_CHECK(model_urdf.rotorGearRatio == model_m.rotorGearRatio);

  BOOST_CHECK(model_urdf.upperEffortLimit.size() == model_m.upperEffortLimit.size());
  BOOST_CHECK(model_urdf.upperEffortLimit == model_m.upperEffortLimit);
  // Cannot test velocity limit since it does not exist in mjcf

  BOOST_CHECK(model_urdf.lowerPositionLimit.size() == model_m.lowerPositionLimit.size());
  BOOST_CHECK(model_urdf.lowerPositionLimit == model_m.lowerPositionLimit);

  BOOST_CHECK(model_urdf.upperPositionLimit.size() == model_m.upperPositionLimit.size());
  BOOST_CHECK(model_urdf.upperPositionLimit == model_m.upperPositionLimit);

  for (size_t k = 1; k < model_m.inertias.size(); ++k)
  {
    BOOST_CHECK(model_urdf.inertias[k].isApprox(model_m.inertias[k]));
  }

  for (size_t k = 1; k < model_urdf.jointPlacements.size(); ++k)
  {
    BOOST_CHECK(model_urdf.jointPlacements[k] == model_m.jointPlacements[k]);
  }

  BOOST_CHECK(model_urdf.joints == model_m.joints);

  BOOST_CHECK(model_urdf.frames.size() == model_m.frames.size());
  for (size_t k = 1; k < model_urdf.frames.size(); ++k)
  {
    BOOST_CHECK(model_urdf.frames[k] == model_m.frames[k]);
  }
}
#endif // PINOCCHIO_WITH_URDFDOM

#if defined(PINOCCHIO_WITH_COLLISION)
BOOST_AUTO_TEST_CASE(test_geometry_parsing)
{
  typedef pinocchio::Model Model;
  typedef pinocchio::GeometryModel GeometryModel;

  // Parse the XML
  std::istringstream xmlData(R"(<mujoco model="inertiaFromGeom">
                                    <compiler inertiafromgeom="true" />
                                    <worldbody>
                                        <body pos="0 0 0" name="bodyCylinder">
                                            <geom type="cylinder" size=".01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            <body pos="0 0 0" name="bodyBox">
                                                <geom type="box" size=".01 0.01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodyCapsule">
                                                <geom type="capsule" size=".01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodySphere">
                                                <geom type="sphere" size=".01" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                            <body pos="0 0 0" name="bodyEllip">
                                                <geom type="ellipsoid" size=".01 0.01 0.25" pos="0 0 0" quat="1 0 0 0"/>
                                            </body>
                                        </body>
                                    </worldbody>
                                  </mujoco>)");

  auto namefile = createTempFile(xmlData);

  Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  GeometryModel geomModel_m;
  pinocchio::mjcf::buildGeom(model_m, namefile.name(), pinocchio::COLLISION, geomModel_m);

  BOOST_CHECK(geomModel_m.ngeoms == 5);

  auto * cyl = dynamic_cast<coal::Cylinder *>(geomModel_m.geometryObjects.at(0).geometry.get());
  BOOST_REQUIRE(cyl);
  BOOST_CHECK(cyl->halfLength == 0.25);
  BOOST_CHECK(cyl->radius == 0.01);

  auto * cap = dynamic_cast<coal::Capsule *>(geomModel_m.geometryObjects.at(2).geometry.get());
  BOOST_REQUIRE(cap);
  BOOST_CHECK(cap->halfLength == 0.25);
  BOOST_CHECK(cap->radius == 0.01);

  auto * s = dynamic_cast<coal::Sphere *>(geomModel_m.geometryObjects.at(3).geometry.get());
  BOOST_REQUIRE(s);
  BOOST_CHECK(s->radius == 0.01);

  auto * b = dynamic_cast<coal::Box *>(geomModel_m.geometryObjects.at(1).geometry.get());
  BOOST_REQUIRE(b);
  Eigen::Vector3d sides;
  sides << 0.01, 0.01, 0.25;
  BOOST_CHECK(b->halfSide == sides);

  auto * e = dynamic_cast<coal::Ellipsoid *>(geomModel_m.geometryObjects.at(4).geometry.get());
  BOOST_REQUIRE(e);
  BOOST_CHECK(e->radii == sides);
}
#endif // if defined(PINOCCHIO_WITH_COLLISION)

BOOST_AUTO_TEST_CASE(test_contact_parsing)
{
  std::vector<pinocchio::FrameAnchorConstraintModel> frame_anchor_constraint_models;
  std::vector<pinocchio::PointAnchorConstraintModel> point_anchor_constraint_models;
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/closed_chain.xml");

  pinocchio::Model model;
  pinocchio::mjcf::buildModel(
    filename, model, point_anchor_constraint_models, frame_anchor_constraint_models);

  BOOST_CHECK_EQUAL(point_anchor_constraint_models.size(), 4);

  // We check that we have correctly parsed the values contained in the XML file
  BOOST_CHECK_EQUAL(
    point_anchor_constraint_models[0].joint1_placement.translation(),
    pinocchio::SE3::Vector3(0.50120, 0, 0));
  BOOST_CHECK_EQUAL(
    point_anchor_constraint_models[1].joint1_placement.translation(),
    pinocchio::SE3::Vector3(0.35012, 0, 0));
  BOOST_CHECK_EQUAL(
    point_anchor_constraint_models[2].joint1_placement.translation(),
    pinocchio::SE3::Vector3(0.50120, 0, 0));
  BOOST_CHECK_EQUAL(
    point_anchor_constraint_models[3].joint1_placement.translation(),
    pinocchio::SE3::Vector3(0.35012, 0, 0));

  // Next, we check if the other constraint placement has been computed correctly.
  // If a point anchor constraint has been constructed well, then the origin of the constraint
  // placements, expressed in the world frame, should match
  const Eigen::VectorXd q0 = model.referenceConfigurations["qpos0"];
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, q0);
  for (const auto & cm : point_anchor_constraint_models)
  {
    const pinocchio::SE3 oMc1 = data.oMi[cm.joint1_id] * cm.joint1_placement;
    const pinocchio::SE3 oMc2 = data.oMi[cm.joint2_id] * cm.joint2_placement;
    BOOST_CHECK(oMc1.translation().isApprox(oMc2.translation()));
  }
}

BOOST_AUTO_TEST_CASE(test_default_eulerseq)
{
  std::istringstream xmlData(R"(<mujoco model="arm">
                                  <compiler angle="radian" />
                                  <worldbody>
                                    <body name="base">
                                      <body name="body" pos="0 0 0.01" euler="1.57 0 0">
                                      </body>
                                    </body>
                                  </worldbody>
                                </mujoco>)");

  auto namefile = createTempFile(xmlData);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "fakeMjcf");
  graph.parseGraphFromXML(namefile.name());
  graph.parseRootTree();

  pinocchio::SE3 placement(
    Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX()).toRotationMatrix(),
    Eigen::Vector3d(0.0, 0.0, 0.01));

  BOOST_CHECK(graph.mapOfBodies["body"].bodyPlacement.isApprox(placement));
}

/// @brief Test parsing a mesh with vertices
/// @param
BOOST_AUTO_TEST_CASE(parse_mesh_with_vertices)
{
  std::istringstream xmlDataNoStrip(R"(<mujoco model="parseVertices">
                                    <asset>
                                      <mesh name="chasis" scale=".01 .006 .0015"
                                        vertex=" 9   2   0
                                                -10  10  10
                                                 9  -2   0
                                                 10  3  -10
                                                 10 -3  -10
                                                -8   10 -10
                                                -10 -10  10
                                                -8  -10 -10
                                                -5   0   20"/>
                                    </asset>
                                  </mujoco>)");

  auto namefile = createTempFile(xmlDataNoStrip);

  typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
  pinocchio::Model model_m;
  MjcfVisitor visitor(model_m);

  MjcfGraph graph(visitor, "/fakeMjcf/fake.xml");
  graph.parseGraphFromXML(namefile.name());

  // Test Meshes
  pinocchio::mjcf::details::MjcfMesh mesh = graph.mapOfMeshes.at("chasis");
  BOOST_CHECK_EQUAL(mesh.scale, Eigen::Vector3d(0.01, 0.006, 0.0015));
  Eigen::MatrixX3d vertices(9, 3);
  vertices << 9, 2, 0, -10, 10, 10, 9, -2, 0, 10, 3, -10, 10, -3, -10, -8, 10, -10, -10, -10, 10,
    -8, -10, -10, -5, 0, 20;
  BOOST_CHECK_EQUAL(mesh.vertices.rows(), 9);
  for (auto i = 0; i < mesh.vertices.rows(); ++i)
  {
    BOOST_CHECK(mesh.vertices.row(i) == vertices.row(i));
  }
}

BOOST_AUTO_TEST_CASE(test_get_unknown_size_vector_from_stream)
{
  const auto v = pinocchio::mjcf::details::internal::getUnknownSizeVectorFromStream("");
  BOOST_CHECK(v.size() == 0);

  const auto v1 = pinocchio::mjcf::details::internal::getUnknownSizeVectorFromStream("1 2 3");
  BOOST_CHECK(v1.size() == 3);
  Eigen::VectorXd expected(3);
  expected << 1, 2, 3;
  BOOST_CHECK(v1 == expected);

  const auto v2 = pinocchio::mjcf::details::internal::getUnknownSizeVectorFromStream(R"(1 2 3
                                                                                        4 5 6)");
  BOOST_CHECK(v2.size() == 6);
  Eigen::VectorXd expected2(6);
  expected2 << 1, 2, 3, 4, 5, 6;
  BOOST_CHECK(v2 == expected2);
}

BOOST_AUTO_TEST_CASE(load_hopper)
{
  std::istringstream xmlData(R"(
<mujoco model="hopper">
  <compiler angle="degree" inertiafromgeom="true"/>
  <default>
    <joint armature="1" damping="1" limited="true"/>
    <geom conaffinity="1" condim="1" contype="1" margin="0.001" material="geom" rgba="0.8 0.6 .4 1" solimp=".8 .8 .01" solref=".02 1"/>
    <motor ctrllimited="true" ctrlrange="-.4 .4"/>
  </default>
  <option integrator="RK4" timestep="0.002"/>
  <visual>
    <map znear="0.02"/>
  </visual>
  <worldbody>
    <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
    <geom conaffinity="1" condim="3" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="20 20 .125" type="plane" material="MatPlane"/>
    <body name="torso" pos="0 0 1.25">
      <camera name="track" mode="trackcom" pos="0 -3 -0.25" xyaxes="1 0 0 0 0 1"/>
      <joint armature="0" axis="1 0 0" damping="0" limited="false" name="rootx" pos="0 0 -1.25" stiffness="0" type="slide"/>
      <joint armature="0" axis="0 0 1" damping="0" limited="false" name="rootz" pos="0 0 -1.25" ref="1.25" stiffness="0" type="slide"/>
      <joint armature="0" axis="0 1 0" damping="0" limited="false" name="rooty" pos="0 0 0" stiffness="0" type="hinge"/>
      <geom friction="0.9" name="torso_geom" size="0.05 0.19999999999999996" type="capsule"/>
      <body name="thigh" pos="0 0 -0.19999999999999996">
        <joint axis="0 -1 0" name="thigh_joint" pos="0 0 0" range="-150 0" type="hinge"/>
        <geom friction="0.9" pos="0 0 -0.22500000000000009" name="thigh_geom" size="0.05 0.22500000000000003" type="capsule"/>
        <body name="leg" pos="0 0 -0.70000000000000007">
          <joint axis="0 -1 0" name="leg_joint" pos="0 0 0.25" range="-150 0" type="hinge"/>
          <geom friction="0.9" name="leg_geom" size="0.04 0.25" type="capsule"/>
          <body name="foot" pos="0.13 0 -0.35">
            <joint axis="0 -1 0" name="foot_joint" pos="-0.13 0 0.1" range="-45 45" type="hinge"/>
            <geom friction="2.0" pos="-0.065 0 0.1" quat="0.70710678118654757 0 -0.70710678118654746 0" name="foot_geom" size="0.06 0.195" type="capsule"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor ctrllimited="true" ctrlrange="-1.0 1.0" gear="200.0" joint="thigh_joint"/>
    <motor ctrllimited="true" ctrlrange="-1.0 1.0" gear="200.0" joint="leg_joint"/>
    <motor ctrllimited="true" ctrlrange="-1.0 1.0" gear="200.0" joint="foot_joint"/>
  </actuator>
    <asset>
        <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0"
            width="100" height="100"/>
        <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
        <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
        <material name="MatPlane" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
        <material name="geom" texture="texgeom" texuniform="true"/>
    </asset>
</mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::GeometryModel geomModel_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);
  // std::cout << "model_m: " << model_m << std::endl;
  pinocchio::mjcf::buildGeom(model_m, namefile.name(), pinocchio::COLLISION, geomModel_m);
  // std::cout << "geomModel_m: " << geomModel_m << std::endl;

  BOOST_CHECK_EQUAL(model_m.nq, 6);
}

// This non regression test address issue: https://github.com/stack-of-tasks/pinocchio/issues/2747
// Site attached to a fixed body should be added to the model as OP_FRAME.
BOOST_AUTO_TEST_CASE(test_site_attached_to_fixed_body)
{
  std::istringstream xmlData(R"(<?xml version="1.0" ?>
<mujoco model="double_pendulum">
    <compiler angle="radian" />

    <worldbody>
        <body name="link1" pos="0 0 0">
            <joint name="j1" type="hinge" axis="1 0 0" pos="0 0 0" />
            <site name="site1" />
            <body name="fixed_body_1">
                <site name="fixed_site_1" pos="0 0 0" />
            </body>
            <body name="link2" pos="0 0 -1">
                <joint name="j2" type="hinge" axis="1 0 0" pos="0 0 0" />
                <site name="site2" />
                <body name="fixed_body_2" pos="0 0 -1">
                    <site name="fixed_site_2" />
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>)");

  auto namefile = createTempFile(xmlData);

  pinocchio::Model model_m;
  pinocchio::mjcf::buildModel(namefile.name(), model_m);

  BOOST_REQUIRE(model_m.existFrame("j1", pinocchio::FrameType::JOINT));
  BOOST_CHECK(model_m.existFrame("site1", pinocchio::FrameType::OP_FRAME));
  BOOST_CHECK(model_m.existFrame("fixed_body_1", pinocchio::FrameType::BODY));
  BOOST_REQUIRE(model_m.existFrame("fixed_site_1", pinocchio::FrameType::OP_FRAME));
  BOOST_REQUIRE(model_m.existFrame("j2", pinocchio::FrameType::JOINT));
  BOOST_CHECK(model_m.existFrame("site2", pinocchio::FrameType::OP_FRAME));
  BOOST_CHECK(model_m.existFrame("fixed_body_2", pinocchio::FrameType::BODY));
  BOOST_REQUIRE(model_m.existFrame("fixed_site_2", pinocchio::FrameType::OP_FRAME));

  auto j1_id = model_m.getJointId("j1");
  auto j2_id = model_m.getJointId("j2");

  auto site_1_id = model_m.getFrameId("fixed_site_1");
  auto site_2_id = model_m.getFrameId("fixed_site_2");

  const auto & site_1 = model_m.frames[site_1_id];
  const auto & site_2 = model_m.frames[site_2_id];

  BOOST_CHECK_EQUAL(site_1.parentJoint, j1_id);
  BOOST_CHECK_EQUAL(site_2.parentJoint, j2_id);

  BOOST_CHECK(site_1.placement.isIdentity());
  BOOST_CHECK(site_2.placement.rotation().isIdentity());
  BOOST_CHECK(site_2.placement.translation().isApprox(Eigen::Vector3d(0, 0, -1.)));
}

BOOST_AUTO_TEST_SUITE_END()
