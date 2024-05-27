//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include <iostream>
#include <fstream>
#include <streambuf>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sdf.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(build_model)
{
  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model;
  const std::string rootLinkName = "pelvis";
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(filename, model, contact_models, rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, rootLinkName, dir);

  BOOST_CHECK(model.nq == 62);
}

BOOST_AUTO_TEST_CASE(build_model_with_joint)
{

  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  const std::string rootLinkName = "pelvis";
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(
    filename, pinocchio::JointModelFreeFlyer(), model, contact_models, rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, rootLinkName, dir);

  BOOST_CHECK(model.nq == 69);
}

BOOST_AUTO_TEST_CASE(build_model_without_rootLink)
{

  const std::string filename =
    PINOCCHIO_MODEL_DIR
    + std::string("/example-robot-data/robots/cassie_description/robots/cassie.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;
  const std::string rootLinkName = "";
  pinocchio::Model model;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(
    filename, pinocchio::JointModelFreeFlyer(), model, contact_models, rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, rootLinkName, dir);

  BOOST_CHECK(model.nq == 69);
}

BOOST_AUTO_TEST_CASE(compare_model_with_urdf)
{
  const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.sdf");
  const std::string dir = PINOCCHIO_MODEL_DIR;

  pinocchio::Model model_sdf;
  const std::string rootLinkName = "WAIST_LINK0";
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModel(
    filename, pinocchio::JointModelFreeFlyer(), model_sdf, contact_models, rootLinkName);
  pinocchio::GeometryModel geomModel;
  pinocchio::sdf::buildGeom(
    model_sdf, filename, pinocchio::COLLISION, geomModel, rootLinkName, dir);

  const std::string filename_urdf = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  const std::string dir_urdf = PINOCCHIO_MODEL_DIR;
  pinocchio::Model model_urdf;
  pinocchio::urdf::buildModel(filename_urdf, pinocchio::JointModelFreeFlyer(), model_urdf);

  typedef typename pinocchio::Model::ConfigVectorMap ConfigVectorMap;
  // Compare models
  // BOOST_CHECK(model_urdf==model);

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
  BOOST_CHECK(
    model_urdf.referenceConfigurations.size() == model_sdf.referenceConfigurations.size());

  typename ConfigVectorMap::const_iterator it = model_sdf.referenceConfigurations.begin();
  typename ConfigVectorMap::const_iterator it_model_urdf =
    model_urdf.referenceConfigurations.begin();
  for (long k = 0; k < (long)model_sdf.referenceConfigurations.size(); ++k)
  {
    std::advance(it, k);
    std::advance(it_model_urdf, k);
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

  for (size_t k = 1; k < model_sdf.inertias.size(); ++k)
  {
    BOOST_CHECK(model_urdf.inertias[k].isApprox(model_sdf.inertias[k]));
  }

  for (size_t k = 1; k < model_urdf.jointPlacements.size(); ++k)
  {
    BOOST_CHECK(model_urdf.jointPlacements[k] == model_sdf.jointPlacements[k]);
  }

  BOOST_CHECK(model_urdf.joints == model_sdf.joints);

  BOOST_CHECK(model_urdf.frames.size() == model_sdf.frames.size());
  for (size_t k = 1; k < model_urdf.frames.size(); ++k)
  {
    BOOST_CHECK(model_urdf.frames[k] == model_sdf.frames[k]);
  }
}

BOOST_AUTO_TEST_CASE(compare_model_in_version_1_6)
{
  // Read file as XML
  std::string filestr("<sdf version=\"1.6\">"
                      "  <model name=\"parallelogram\">"
                      "    <link name=\"link_A1\">"
                      "      <pose>0 0 0 0 0 0</pose>"
                      "      <inertial>"
                      "        <pose>0 0 0 0 0 0</pose>"
                      "        <mass>10</mass>"
                      "        <inertia>"
                      "          <ixx>0.008416666667</ixx>"
                      "          <iyy>0.841666666667</iyy>"
                      "          <izz>0.833416666667</izz>"
                      "          <ixy>0.</ixy>"
                      "          <ixz>0.</ixz>"
                      "          <iyz>0.</iyz>"
                      "        </inertia>"
                      "      </inertial>"
                      "      <visual name=\"link_A1_visual\">"
                      "        <geometry>"
                      "          <cylinder>"
                      "            <length>1.0</length>"
                      "            <radius>0.05</radius>"
                      "          </cylinder>"
                      "        </geometry>"
                      "      </visual>"
                      "    </link>"
                      "    <link name=\"link_B1\">"
                      "      <pose>-0.2 0 0 0 0 0</pose>"
                      "      <inertial>"
                      "        <pose>0 0 0 0 0 0</pose>"
                      "        <mass>5</mass>"
                      "        <inertia>"
                      "          <ixx>0.0042083333333</ixx>"
                      "          <iyy>0.1541666666667</iyy>"
                      "          <izz>0.1500416666667</izz>"
                      "          <ixy>0</ixy>"
                      "          <ixz>0</ixz>"
                      "          <iyz>0</iyz>"
                      "        </inertia>"
                      "      </inertial>"
                      "      <visual name=\"link_B1_visual\">"
                      "        <geometry>"
                      "          <cylinder>"
                      "            <length>0.6</length>"
                      "            <radius>0.05</radius>"
                      "          </cylinder>"
                      "        </geometry>"
                      "      </visual>"
                      "    </link>"
                      "    <link name=\"link_A2\">"
                      "      <pose>0.6 0 0 0 0 0</pose>"
                      "      <inertial>"
                      "        <pose>0 0 0 0 0 0</pose>"
                      "        <mass>10</mass>"
                      "        <inertia>"
                      "          <ixx>0.008416666667</ixx>"
                      "          <iyy>0.841666666667</iyy>"
                      "          <izz>0.833416666667</izz>"
                      "          <ixy>0.</ixy>"
                      "          <ixz>0.</ixz>"
                      "          <iyz>0.</iyz>"
                      "        </inertia>"
                      "      </inertial>"
                      "      <visual name=\"link_A2_visual\">"
                      "        <geometry>"
                      "          <cylinder>"
                      "            <length>1.0</length>"
                      "            <radius>0.05</radius>"
                      "          </cylinder>"
                      "        </geometry>"
                      "      </visual>"
                      "    </link>"
                      "    <link name=\"link_B2\">"
                      "      <pose>0.8 0 0 0 0 0</pose>"
                      "      <inertial>"
                      "        <pose>0 0 0 0 0 0</pose>"
                      "        <mass>5</mass>"
                      "        <inertia>"
                      "          <ixx>0.0042083333333</ixx>"
                      "          <iyy>0.1541666666667</iyy>"
                      "          <izz>0.1500416666667</izz>"
                      "          <ixy>0</ixy>"
                      "          <ixz>0</ixz>"
                      "          <iyz>0</iyz>"
                      "        </inertia>"
                      "      </inertial>"
                      "      <visual name=\"link_B2_visual\">"
                      "        <geometry>"
                      "          <cylinder>"
                      "            <length>0.6</length>"
                      "            <radius>0.05</radius>"
                      "          </cylinder>"
                      "        </geometry>"
                      "      </visual>"
                      "    </link>"
                      "    <joint name=\"joint_B1\" type=\"revolute\">"
                      "      <pose>-0.3 0 0 0 0 0</pose>"
                      "      <child>link_B1</child>"
                      "      <parent>link_A1</parent>"
                      "      <axis>"
                      "        <xyz>0 1 0</xyz>"
                      "        <use_parent_model_frame>1</use_parent_model_frame>"
                      "      </axis>"
                      "    </joint>"
                      "    <joint name=\"joint_A2\" type=\"revolute\">"
                      "      <pose>-0.5 0 0 0 0 0</pose>"
                      "      <child>link_A2</child>"
                      "      <parent>link_B1</parent>"
                      "      <axis>"
                      "        <xyz>0 1 0</xyz>"
                      "        <use_parent_model_frame>1</use_parent_model_frame>"
                      "      </axis>"
                      "    </joint>"
                      "    <joint name=\"joint_B2\" type=\"revolute\">"
                      "      <pose>-0.3 0 0 0 0 0</pose>"
                      "      <child>link_B2</child>"
                      "      <parent>link_A1</parent>"
                      "      <axis>"
                      "        <xyz>0 1 0</xyz>"
                      "        <use_parent_model_frame>1</use_parent_model_frame>"
                      "      </axis>"
                      "    </joint>"
                      "    <joint name=\"joint_B3\" type=\"revolute\">"
                      "      <pose>0.5 0 0 0 0 0</pose>"
                      "      <child>link_A2</child>"
                      "      <parent>link_B2</parent>"
                      "      <axis>"
                      "        <xyz>0 1 0</xyz>"
                      "        <use_parent_model_frame>1</use_parent_model_frame>"
                      "      </axis>"
                      "    </joint>"
                      "  </model>"
                      "</sdf>");

  double height = 0.1;
  double width = 0.01;

  double mass_link_A = 10.;
  double length_link_A = 1.;

  double mass_link_B = 5.;
  double length_link_B = .6;

  pinocchio::Inertia inertia_link_A_2 =
    pinocchio::Inertia::FromBox(mass_link_A / 2, length_link_A, width, height);

  pinocchio::SE3 placement_center_link_A = pinocchio::SE3::Identity();
  placement_center_link_A.translation() = Eigen::Vector3d::UnitX() * length_link_A / 2.;

  pinocchio::SE3 placement_center_link_A_minus = pinocchio::SE3::Identity();
  placement_center_link_A_minus.translation() = -Eigen::Vector3d::UnitX() * length_link_A / 2;

  pinocchio::SE3 placement_shape_A = placement_center_link_A;
  placement_shape_A.rotation() = Eigen::Quaterniond::Quaternion::FromTwoVectors(
                                   Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX())
                                   .matrix();

  pinocchio::Inertia inertia_link_B =
    pinocchio::Inertia::FromBox(mass_link_B, length_link_B, width, height);
  pinocchio::SE3 placement_center_link_B = pinocchio::SE3::Identity();
  placement_center_link_B.translation() = Eigen::Vector3d::UnitX() * length_link_B / 2.;
  pinocchio::SE3 placement_shape_B = placement_center_link_B;
  placement_shape_B.rotation() =
    Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()).matrix();

  pinocchio::Model model;
  pinocchio::JointIndex base_joint_id = 0;

  pinocchio::SE3 joint1_placement = pinocchio::SE3::Identity();
  joint1_placement.translation() = -Eigen::Vector3d::UnitX() * length_link_A / 2;
  pinocchio::JointIndex joint1_id =
    model.addJoint(base_joint_id, pinocchio::JointModelRY(), joint1_placement, "link_B1");
  model.appendBodyToJoint(joint1_id, inertia_link_B, placement_center_link_B);

  pinocchio::SE3 joint2_placement = pinocchio::SE3::Identity();
  joint2_placement.translation() = Eigen::Vector3d::UnitX() * length_link_B;
  pinocchio::JointIndex joint2_id =
    model.addJoint(joint1_id, pinocchio::JointModelRY(), joint2_placement, "link_A2");
  model.appendBodyToJoint(joint2_id, inertia_link_A_2, placement_center_link_A);

  pinocchio::SE3 joint3_placement = pinocchio::SE3::Identity();
  joint3_placement.translation() = Eigen::Vector3d::UnitX() * length_link_A / 2;
  pinocchio::JointIndex joint3_id =
    model.addJoint(base_joint_id, pinocchio::JointModelRY(), joint3_placement, "link_B2");
  model.appendBodyToJoint(joint3_id, inertia_link_B, placement_center_link_B);

  pinocchio::SE3 joint4_placement = pinocchio::SE3::Identity();
  joint4_placement.translation() = Eigen::Vector3d::UnitX() * length_link_B;
  pinocchio::JointIndex joint4_id =
    model.addJoint(joint3_id, pinocchio::JointModelRY(), joint4_placement, "link_B3");
  model.appendBodyToJoint(joint4_id, inertia_link_A_2, placement_center_link_A_minus);

  pinocchio::Model model_sdf;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) contact_models;
  pinocchio::sdf::buildModelFromXML(filestr, model_sdf, contact_models);

  BOOST_CHECK(model.nq == model_sdf.nq);
  BOOST_CHECK(model.nv == model_sdf.nv);
  BOOST_CHECK(model.njoints == model_sdf.njoints);
  BOOST_CHECK(model.nbodies == model_sdf.nbodies);

  BOOST_CHECK(model.parents == model_sdf.parents);

  BOOST_CHECK(model.children == model_sdf.children);
  BOOST_CHECK(model.subtrees == model_sdf.subtrees);

  BOOST_CHECK(model.gravity == model_sdf.gravity);
  BOOST_CHECK(model.idx_qs == model_sdf.idx_qs);

  BOOST_CHECK(model.nqs == model_sdf.nqs);
  BOOST_CHECK(model.idx_vs == model_sdf.idx_vs);
  BOOST_CHECK(model.nvs == model_sdf.nvs);

  for (std::size_t k = 1; k < model.jointPlacements.size(); k++)
  {
    BOOST_CHECK(model.jointPlacements[k].isApprox(model_sdf.jointPlacements[k]));
  }

  for (std::size_t k = 1; k < model.inertias.size(); k++)
  {
    BOOST_CHECK(model.inertias[k].isApprox(model_sdf.inertias[k]));
  }

  BOOST_CHECK(contact_models.size() == 1);
  BOOST_CHECK(contact_models[0].joint1_id == 4);
  BOOST_CHECK(contact_models[0].joint1_placement == placement_center_link_A_minus);
  BOOST_CHECK(contact_models[0].joint2_id == 2);
  BOOST_CHECK(contact_models[0].joint2_placement == placement_center_link_A);
  BOOST_CHECK(contact_models[0].type == pinocchio::CONTACT_6D);
  BOOST_CHECK(contact_models[0].reference_frame == pinocchio::LOCAL);
}

BOOST_AUTO_TEST_SUITE_END()
