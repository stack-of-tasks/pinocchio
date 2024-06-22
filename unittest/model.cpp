//
// Copyright (c) 2016-2022 CNRS INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/fwd.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_model_subtree)
{
  Model model;
  buildModels::humanoidRandom(model);

  Model::JointIndex idx_larm1 = model.getJointId("larm1_joint");
  BOOST_CHECK(idx_larm1 < (Model::JointIndex)model.njoints);
  Model::IndexVector & subtree = model.subtrees[idx_larm1];
  BOOST_CHECK(subtree.size() == 6);

  for (size_t joint_id = 0; joint_id < model.joints.size(); ++joint_id)
  {
    const Model::IndexVector & children = model.children[joint_id];
    for (size_t i = 0; i < children.size(); ++i)
    {
      BOOST_CHECK(model.parents[children[i]] == joint_id);
    }
  }

  for (size_t i = 1; i < subtree.size(); ++i)
  {
    BOOST_CHECK(model.parents[subtree[i]] == subtree[i - 1]);
  }

  // Check that i starts subtree[i]
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK(model.subtrees[joint_id][0] == joint_id);
  }

  // Check that subtree[0] contains all joint ids
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK(model.subtrees[0][joint_id - 1] == joint_id);
  }
}

BOOST_AUTO_TEST_CASE(test_model_get_frame_id)
{
  Model model;
  buildModels::humanoidRandom(model);

  for (FrameIndex i = 0; i < static_cast<FrameIndex>(model.nframes); i++)
  {
    BOOST_CHECK_EQUAL(i, model.getFrameId(model.frames[i].name));
  }
  BOOST_CHECK_EQUAL(model.nframes, model.getFrameId("NOT_A_FRAME"));
}

BOOST_AUTO_TEST_CASE(test_model_support)
{
  Model model;
  buildModels::humanoidRandom(model);
  const Model::IndexVector support0_ref(1, 0);
  BOOST_CHECK(model.supports[0] == support0_ref);

  // Check that i ends supports[i]
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK(model.supports[joint_id].back() == joint_id);
    Model::IndexVector & support = model.supports[joint_id];

    size_t current_id = support.size() - 2;
    for (JointIndex parent_id = model.parents[joint_id]; parent_id > 0;
         parent_id = model.parents[parent_id], current_id--)
    {
      BOOST_CHECK(parent_id == support[current_id]);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_model_subspace_dimensions)
{
  Model model;
  buildModels::humanoidRandom(model);

  // Check that i ends supports[i]
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    const Model::JointModel & jmodel = model.joints[joint_id];

    BOOST_CHECK(model.nqs[joint_id] == jmodel.nq());
    BOOST_CHECK(model.idx_qs[joint_id] == jmodel.idx_q());

    BOOST_CHECK(model.nvs[joint_id] == jmodel.nv());
    BOOST_CHECK(model.idx_vs[joint_id] == jmodel.idx_v());
  }
}

BOOST_AUTO_TEST_CASE(comparison)
{
  Model model;
  buildModels::humanoidRandom(model);

  BOOST_CHECK(model == model);
}

BOOST_AUTO_TEST_CASE(cast)
{
  Model model;
  buildModels::humanoidRandom(model);

  BOOST_CHECK(model.cast<double>() == model.cast<double>());
  BOOST_CHECK(model.cast<double>().cast<long double>() == model.cast<long double>());

  typedef ModelTpl<long double> Modelld;

  Modelld model2(model);
  BOOST_CHECK(model2 == model.cast<long double>());
}

BOOST_AUTO_TEST_CASE(test_std_vector_of_Model)
{
  Model model;
  buildModels::humanoid(model);

  PINOCCHIO_ALIGNED_STD_VECTOR(Model) models;
  for (size_t k = 0; k < 20; ++k)
  {
    models.push_back(Model());
    buildModels::humanoid(models.back());

    BOOST_CHECK(model == models.back());
  }
}

#ifdef PINOCCHIO_WITH_HPP_FCL
struct AddPrefix
{
  std::string p;
  std::string operator()(const std::string & n)
  {
    return p + n;
  }
  Frame operator()(const Frame & _f)
  {
    Frame f(_f);
    f.name = p + f.name;
    return f;
  }
  AddPrefix(const char * _p)
  : p(_p)
  {
  }
};

BOOST_AUTO_TEST_CASE(append)
{
  Model manipulator, humanoid;
  GeometryModel geomManipulator, geomHumanoid;

  buildModels::manipulator(manipulator);
  buildModels::manipulatorGeometries(manipulator, geomManipulator);
  geomManipulator.addAllCollisionPairs();
  // Add prefix to joint and frame names
  AddPrefix addManipulatorPrefix("manipulator/");
  std::transform(
    ++manipulator.names.begin(), manipulator.names.end(), ++manipulator.names.begin(),
    addManipulatorPrefix);
  std::transform(
    ++manipulator.frames.begin(), manipulator.frames.end(), ++manipulator.frames.begin(),
    addManipulatorPrefix);

  BOOST_TEST_MESSAGE(manipulator);

  buildModels::humanoid(humanoid);
  buildModels::humanoidGeometries(humanoid, geomHumanoid);
  geomHumanoid.addAllCollisionPairs();
  // Add prefix to joint and frame names
  AddPrefix addHumanoidPrefix("humanoid/");
  std::transform(
    ++humanoid.names.begin(), humanoid.names.end(), ++humanoid.names.begin(), addHumanoidPrefix);
  std::transform(
    ++humanoid.frames.begin(), humanoid.frames.end(), ++humanoid.frames.begin(), addHumanoidPrefix);

  BOOST_TEST_MESSAGE(humanoid);

  typename Model::ConfigVectorType humanoid_config_vector(humanoid.nq);
  typename Model::ConfigVectorType manipulator_config_vector(manipulator.nq);
  humanoid_config_vector = randomConfiguration(humanoid);
  manipulator_config_vector = randomConfiguration(manipulator);
  humanoid.referenceConfigurations.insert(std::make_pair("common_key", humanoid_config_vector));
  manipulator.referenceConfigurations.insert(
    std::make_pair("common_key", manipulator_config_vector));

  humanoid_config_vector = randomConfiguration(humanoid);
  manipulator_config_vector = randomConfiguration(manipulator);
  humanoid.referenceConfigurations.insert(std::make_pair("humanoid_key", humanoid_config_vector));
  manipulator.referenceConfigurations.insert(
    std::make_pair("manipulator_key", manipulator_config_vector));

  // TODO fix inertia of the base
  manipulator.inertias[0].setRandom();
  SE3 aMb = SE3::Random();

  // First append a model to the universe frame.
  Model model1;
  GeometryModel geomModel1;
  FrameIndex fid = 0;
  appendModel(
    humanoid, manipulator, geomHumanoid, geomManipulator, fid, SE3::Identity(), model1, geomModel1);
  typedef typename Model::ConfigVectorMap ConfigVectorMap;

  typename Model::ConfigVectorType neutral_config_vector(model1.nq);
  neutral(model1, neutral_config_vector);

  BOOST_CHECK(model1.referenceConfigurations.size() == 3);
  for (typename ConfigVectorMap::const_iterator config_it = model1.referenceConfigurations.begin();
       config_it != model1.referenceConfigurations.end(); ++config_it)
  {
    const std::string & config_name = config_it->first;
    const typename Model::ConfigVectorType & config_vector = config_it->second;

    typename ConfigVectorMap::const_iterator humanoid_config =
      humanoid.referenceConfigurations.find(config_name);
    typename ConfigVectorMap::const_iterator manipulator_config =
      manipulator.referenceConfigurations.find(config_name);
    for (JointIndex joint_id = 1; joint_id < model1.joints.size(); ++joint_id)
    {
      const JointModel & joint_model1 = model1.joints[joint_id];
      if (
        humanoid_config != humanoid.referenceConfigurations.end()
        && humanoid.existJointName(model1.names[joint_id]))
      { // key and joint exists in humanoid
        const JointModel & joint_model_humanoid =
          humanoid.joints[humanoid.getJointId(model1.names[joint_id])];
        BOOST_CHECK(
          joint_model_humanoid.jointConfigSelector(humanoid_config->second)
          == joint_model1.jointConfigSelector(config_vector));
        // std::cerr<<"humanoid "<<config_name<<" "<<model1.names[joint_id]<<std::endl;
      }
      else if (
        manipulator_config != manipulator.referenceConfigurations.end()
        && manipulator.existJointName(model1.names[joint_id]))
      { // key and joint exists in manipulator.
        const JointModel & joint_model_manipulator =
          manipulator.joints[manipulator.getJointId(model1.names[joint_id])];
        BOOST_CHECK(
          joint_model_manipulator.jointConfigSelector(manipulator_config->second)
          == joint_model1.jointConfigSelector(config_vector));
        // std::cerr<<"manipulator "<<config_name<<" "<<model1.names[joint_id]<<std::endl;
      }
      else
      { // joint and key combo not found, should with neutral
        BOOST_CHECK(
          joint_model1.jointConfigSelector(neutral_config_vector)
          == joint_model1.jointConfigSelector(config_vector));
        // std::cerr<<"neutral "<<config_name<<" "<<model1.names[joint_id]<<std::endl;
      }
    }
  }

  {
    Model model2 = appendModel(humanoid, manipulator, fid, SE3::Identity());
    Model model3;
    appendModel(humanoid, manipulator, fid, SE3::Identity(), model3);
    BOOST_CHECK(model1 == model2);
    BOOST_CHECK(model1 == model3);
    BOOST_CHECK(model2 == model3);
  }

  Data data1(model1);
  BOOST_CHECK(model1.check(data1));

  BOOST_TEST_MESSAGE(model1);

  // Second, append a model to a moving frame.
  Model model;

  GeometryModel geomModel;
  fid = humanoid.addFrame(Frame(
    "humanoid/add_manipulator", humanoid.getJointId("humanoid/chest2_joint"),
    humanoid.getFrameId("humanoid/chest2_joint"), aMb, OP_FRAME));

  // Append manipulator to chest2_joint of humanoid
  appendModel(
    humanoid, manipulator, geomHumanoid, geomManipulator, fid, SE3::Identity(), model, geomModel);

  neutral_config_vector.resize(model.nq);
  neutral(model, neutral_config_vector);

  BOOST_CHECK(model.referenceConfigurations.size() == 3);
  for (typename ConfigVectorMap::const_iterator config_it = model.referenceConfigurations.begin();
       config_it != model.referenceConfigurations.end(); ++config_it)
  {
    const std::string & config_name = config_it->first;
    const typename Model::ConfigVectorType & config_vector = config_it->second;

    typename ConfigVectorMap::const_iterator humanoid_config =
      humanoid.referenceConfigurations.find(config_name);
    typename ConfigVectorMap::const_iterator manipulator_config =
      manipulator.referenceConfigurations.find(config_name);
    for (JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
    {
      const JointModel & joint_model = model.joints[joint_id];
      if (
        humanoid_config != humanoid.referenceConfigurations.end()
        && humanoid.existJointName(model.names[joint_id]))
      { // key and joint exists in humanoid
        const JointModel & joint_model_humanoid =
          humanoid.joints[humanoid.getJointId(model.names[joint_id])];
        BOOST_CHECK(
          joint_model_humanoid.jointConfigSelector(humanoid_config->second)
          == joint_model.jointConfigSelector(config_vector));
        // std::cerr<<"humanoid "<<config_name<<" "<<model.names[joint_id]<<std::endl;
      }
      else if (
        manipulator_config != manipulator.referenceConfigurations.end()
        && manipulator.existJointName(model.names[joint_id]))
      { // key and joint exists in manipulator.
        const JointModel & joint_model_manipulator =
          manipulator.joints[manipulator.getJointId(model.names[joint_id])];
        BOOST_CHECK(
          joint_model_manipulator.jointConfigSelector(manipulator_config->second)
          == joint_model.jointConfigSelector(config_vector));
        // std::cerr<<"manipulator "<<config_name<<" "<<model.names[joint_id]<<std::endl;
      }
      else
      { // joint and key combo not found, should with neutral
        BOOST_CHECK(
          joint_model.jointConfigSelector(neutral_config_vector)
          == joint_model.jointConfigSelector(config_vector));
        // std::cerr<<"neutral "<<config_name<<" "<<model.names[joint_id]<<std::endl;
      }
    }
  }

  {
    Model model2 = appendModel(humanoid, manipulator, fid, SE3::Identity());
    Model model3;
    appendModel(humanoid, manipulator, fid, SE3::Identity(), model3);
    BOOST_CHECK(model == model2);
    BOOST_CHECK(model == model3);
    BOOST_CHECK(model2 == model3);
  }

  BOOST_TEST_MESSAGE(model);

  Data data(model);
  BOOST_CHECK(model.check(data));

  // Check the model
  BOOST_CHECK_EQUAL(
    model.getJointId("humanoid/chest2_joint"),
    model.parents[model.getJointId("manipulator/shoulder1_joint")]);

  // check the joint order and the inertias
  // All the joints of the manipulator should be at the end of the merged model
  JointIndex hnj = (JointIndex)humanoid.njoints;
  JointIndex chest2 = model.getJointId("humanoid/chest2_joint");
  for (JointIndex jid = 1; jid < hnj; ++jid)
  {
    BOOST_TEST_MESSAGE("Checking joint " << jid << " " << model.names[jid]);
    BOOST_CHECK_EQUAL(model.names[jid], humanoid.names[jid]);
    if (jid != chest2)
      BOOST_CHECK_EQUAL(model.inertias[jid], humanoid.inertias[jid]);
    else
      BOOST_CHECK_MESSAGE(
        model.inertias[jid].isApprox(
          manipulator.inertias[0].se3Action(aMb) + humanoid.inertias[jid]),
        model.inertias[jid] << " != "
                            << manipulator.inertias[0].se3Action(aMb) + humanoid.inertias[jid]);

    BOOST_CHECK_EQUAL(model.jointPlacements[jid], humanoid.jointPlacements[jid]);
  }
  for (JointIndex jid = 1; jid < manipulator.joints.size() - 1; ++jid)
  {
    BOOST_TEST_MESSAGE("Checking joint " << hnj - 1 + jid << " " << model.names[hnj + jid]);
    BOOST_CHECK_EQUAL(model.names[hnj - 1 + jid], manipulator.names[jid]);
    BOOST_CHECK_EQUAL(model.inertias[hnj - 1 + jid], manipulator.inertias[jid]);
    if (jid == 1)
      BOOST_CHECK_EQUAL(
        model.jointPlacements[hnj - 1 + jid], aMb * manipulator.jointPlacements[jid]);
    else
      BOOST_CHECK_EQUAL(model.jointPlacements[hnj - 1 + jid], manipulator.jointPlacements[jid]);
  }
  // Check the frames
  for (FrameIndex fid = 1; fid < humanoid.frames.size(); ++fid)
  {
    const Frame &frame = humanoid.frames[fid], parent = humanoid.frames[frame.parentFrame];
    BOOST_CHECK(model.existFrame(frame.name, frame.type));
    const Frame &nframe = model.frames[model.getFrameId(frame.name, frame.type)],
                nparent = model.frames[nframe.parentFrame];
    BOOST_CHECK_EQUAL(parent.name, nparent.name);
    BOOST_CHECK_EQUAL(frame.placement, nframe.placement);
  }
  for (FrameIndex fid = 1; fid < manipulator.frames.size(); ++fid)
  {
    const Frame &frame = manipulator.frames[fid], parent = manipulator.frames[frame.parentFrame];
    BOOST_CHECK(model.existFrame(frame.name, frame.type));
    const Frame &nframe = model.frames[model.getFrameId(frame.name, frame.type)],
                nparent = model.frames[nframe.parentFrame];
    if (frame.parentFrame > 0)
    {
      BOOST_CHECK_EQUAL(parent.name, nparent.name);
      BOOST_CHECK_EQUAL(frame.placement, nframe.placement);
    }
  }

  {
    Inertia inertia(2., Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Matrix3d::Identity());
    Frame additional_frame("inertial_frame", 2, SE3::Identity(), FrameType::JOINT, inertia);
    humanoid.addFrame(additional_frame);
    double mass_humanoid = computeTotalMass(humanoid);
    double mass_manipulator = computeTotalMass(manipulator);
    double total_mass = mass_manipulator + mass_humanoid;

    Model model4;
    GeometryModel geomModel4;
    appendModel(
      humanoid, manipulator, geomHumanoid, geomManipulator, 0, SE3::Identity(), model4, geomModel4);
    BOOST_CHECK_CLOSE(computeTotalMass(model4), total_mass, 1e-6);
  }
  {
    Model ff_model;
    auto ff_id = ff_model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "floating_base");
    ff_model.addJointFrame(ff_id);
    GeometryModel ff_geom_model = GeometryModel();
    FrameIndex frame_id = ff_model.getFrameId("floating_base");
    Model model4;
    GeometryModel geomModel4;
    appendModel(
      ff_model, manipulator, ff_geom_model, geomManipulator, frame_id, SE3::Identity(), model4,
      geomModel4);
    BOOST_CHECK(model4.inertias[1] == model4.inertias[1]);
  }

  {
    Model model5, gripperModel;

    Inertia inertia(1., SE3::Vector3(0.5, 0., 0.0), SE3::Matrix3::Identity());
    SE3 pos(1);

    pos.translation() = SE3::LinearType(0.1, 0., 0.);
    JointIndex idx = gripperModel.addJoint(0, JointModelPX(), pos, "left_finger");
    gripperModel.addJointFrame(idx);

    pos.translation() = SE3::LinearType(-0.1, 0., 0.);
    idx = gripperModel.addJoint(0, JointModelPX(), pos, "right_finger");
    gripperModel.addJointFrame(idx);

    SE3 transformManGr = SE3::Random();
    FrameIndex fid = (FrameIndex)(manipulator.frames.size() - 1);
    appendModel(manipulator, gripperModel, fid, transformManGr, model5);

    JointIndex jid5 = model5.getJointId("left_finger");
    JointIndex jidG = gripperModel.getJointId("left_finger");
    BOOST_CHECK(
      model5.jointPlacements[jid5].isApprox(transformManGr * gripperModel.jointPlacements[jidG]));

    jid5 = model5.getJointId("right_finger");
    jidG = gripperModel.getJointId("right_finger");
    BOOST_CHECK(
      model5.jointPlacements[jid5].isApprox(transformManGr * gripperModel.jointPlacements[jidG]));
  }
}
#endif

BOOST_AUTO_TEST_CASE(test_buildReducedModel_empty)
{
  Model humanoid_model;
  buildModels::humanoid(humanoid_model);

  static const std::vector<JointIndex> empty_index_vector;

  humanoid_model.lowerPositionLimit.head<3>().fill(-1.);
  humanoid_model.upperPositionLimit.head<3>().fill(1.);

  humanoid_model.referenceConfigurations.insert(
    std::pair<std::string, Eigen::VectorXd>("neutral", neutral(humanoid_model)));
  Eigen::VectorXd reference_config_humanoid = randomConfiguration(humanoid_model);
  Model humanoid_copy_model =
    buildReducedModel(humanoid_model, empty_index_vector, reference_config_humanoid);

  BOOST_CHECK(humanoid_copy_model.names == humanoid_model.names);
  BOOST_CHECK(humanoid_copy_model.joints == humanoid_model.joints);
  BOOST_CHECK(humanoid_copy_model == humanoid_model);
  BOOST_CHECK(
    humanoid_copy_model.referenceConfigurations["neutral"].isApprox(neutral(humanoid_copy_model)));

  const std::vector<JointIndex> empty_joints_to_lock;

  const Model reduced_humanoid_model =
    buildReducedModel(humanoid_model, empty_joints_to_lock, reference_config_humanoid);
  BOOST_CHECK(reduced_humanoid_model.njoints == humanoid_model.njoints);
  BOOST_CHECK(reduced_humanoid_model.frames == humanoid_model.frames);
  BOOST_CHECK(reduced_humanoid_model.jointPlacements == humanoid_model.jointPlacements);
  BOOST_CHECK(reduced_humanoid_model.joints == humanoid_model.joints);

  for (JointIndex joint_id = 1; joint_id < (JointIndex)reduced_humanoid_model.njoints; ++joint_id)
  {
    BOOST_CHECK(
      reduced_humanoid_model.inertias[joint_id].isApprox(humanoid_model.inertias[joint_id]));
  }
}

BOOST_AUTO_TEST_CASE(test_buildReducedModel)
{
  Model humanoid_model;
  buildModels::humanoid(humanoid_model);

  static const std::vector<JointIndex> empty_index_vector;

  humanoid_model.lowerPositionLimit.head<3>().fill(-1.);
  humanoid_model.upperPositionLimit.head<3>().fill(1.);

  humanoid_model.referenceConfigurations.insert(
    std::pair<std::string, Eigen::VectorXd>("neutral", neutral(humanoid_model)));
  Eigen::VectorXd reference_config_humanoid = randomConfiguration(humanoid_model);
  Model humanoid_copy_model =
    buildReducedModel(humanoid_model, empty_index_vector, reference_config_humanoid);

  BOOST_CHECK(humanoid_copy_model.names == humanoid_model.names);
  BOOST_CHECK(humanoid_copy_model.joints == humanoid_model.joints);
  BOOST_CHECK(humanoid_copy_model == humanoid_model);
  BOOST_CHECK(
    humanoid_copy_model.referenceConfigurations["neutral"].isApprox(neutral(humanoid_copy_model)));

  std::vector<JointIndex> joints_to_lock;
  const std::string joint1_to_lock = "rarm_shoulder2_joint";
  joints_to_lock.push_back(humanoid_model.getJointId(joint1_to_lock));
  const std::string joint2_to_lock = "larm_shoulder2_joint";
  joints_to_lock.push_back(humanoid_model.getJointId(joint2_to_lock));

  Model reduced_humanoid_model =
    buildReducedModel(humanoid_model, joints_to_lock, reference_config_humanoid);
  BOOST_CHECK(
    reduced_humanoid_model.njoints == humanoid_model.njoints - (int)joints_to_lock.size());

  BOOST_CHECK(reduced_humanoid_model != humanoid_model);

  Model reduced_humanoid_model_other_signature;
  buildReducedModel(
    humanoid_model, joints_to_lock, reference_config_humanoid,
    reduced_humanoid_model_other_signature);
  BOOST_CHECK(reduced_humanoid_model == reduced_humanoid_model_other_signature);

  // Check that forward kinematics give same results
  Data data(humanoid_model), reduced_data(reduced_humanoid_model);
  Eigen::VectorXd q = reference_config_humanoid;
  Eigen::VectorXd reduced_q(reduced_humanoid_model.nq);

  for (JointIndex joint_id = 1; joint_id < (JointIndex)reduced_humanoid_model.njoints; ++joint_id)
  {
    const JointIndex reference_joint_id =
      humanoid_model.getJointId(reduced_humanoid_model.names[joint_id]);

    reduced_humanoid_model.joints[joint_id].jointConfigSelector(reduced_q) =
      humanoid_model.joints[reference_joint_id].jointConfigSelector(q);
  }

  BOOST_CHECK(reduced_humanoid_model.referenceConfigurations["neutral"].isApprox(
    neutral(reduced_humanoid_model)));

  framesForwardKinematics(humanoid_model, data, q);
  framesForwardKinematics(reduced_humanoid_model, reduced_data, reduced_q);

  for (size_t frame_id = 0; frame_id < reduced_humanoid_model.frames.size(); ++frame_id)
  {
    const Frame & reduced_frame = reduced_humanoid_model.frames[frame_id];
    switch (reduced_frame.type)
    {
    case JOINT:
    case FIXED_JOINT: {
      // May not be present in the original model
      if (humanoid_model.existJointName(reduced_frame.name))
      {
        const JointIndex joint_id = humanoid_model.getJointId(reduced_frame.name);
        BOOST_CHECK(data.oMi[joint_id].isApprox(reduced_data.oMf[frame_id]));
      }
      else if (humanoid_model.existFrame(reduced_frame.name))
      {
        const FrameIndex humanoid_frame_id = humanoid_model.getFrameId(reduced_frame.name);
        BOOST_CHECK(data.oMf[humanoid_frame_id].isApprox(reduced_data.oMf[frame_id]));
      }
      else
      {
        BOOST_CHECK_MESSAGE(
          false, "The frame " << reduced_frame.name << " is not presend in the humanoid_model");
      }
      break;
    }
    default: {
      BOOST_CHECK(humanoid_model.existFrame(reduced_frame.name));
      const FrameIndex humanoid_frame_id = humanoid_model.getFrameId(reduced_frame.name);
      BOOST_CHECK(data.oMf[humanoid_frame_id].isApprox(reduced_data.oMf[frame_id]));
      break;
    }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_aligned_vector_of_model)
{
  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Model) VectorOfModels;

  VectorOfModels models;
  for (size_t k = 0; k < 100; ++k)
  {
    models.push_back(Model());
    buildModels::humanoidRandom(models[k]);
  }
}

#ifdef PINOCCHIO_WITH_HPP_FCL
BOOST_AUTO_TEST_CASE(test_buildReducedModel_with_geom)
{
  Model humanoid_model;
  buildModels::humanoid(humanoid_model);

  humanoid_model.lowerPositionLimit.head<3>().fill(-1.);
  humanoid_model.upperPositionLimit.head<3>().fill(1.);
  const Eigen::VectorXd reference_config_humanoid = randomConfiguration(humanoid_model);

  GeometryModel humanoid_geometry;
  buildModels::humanoidGeometries(humanoid_model, humanoid_geometry);

  static const std::vector<JointIndex> empty_index_vector;

  Model humanoid_copy_model;
  GeometryModel humanoid_copy_geometry;
  buildReducedModel(
    humanoid_model, humanoid_geometry, empty_index_vector, reference_config_humanoid,
    humanoid_copy_model, humanoid_copy_geometry);

  BOOST_CHECK(humanoid_copy_model == humanoid_model);
  BOOST_CHECK(humanoid_copy_geometry == humanoid_geometry);

  std::vector<JointIndex> joints_to_lock;
  const std::string joint1_to_lock = "rarm_shoulder2_joint";
  joints_to_lock.push_back(humanoid_model.getJointId(joint1_to_lock));
  const std::string joint2_to_lock = "larm_shoulder2_joint";
  joints_to_lock.push_back(humanoid_model.getJointId(joint2_to_lock));

  Model reduced_humanoid_model;
  GeometryModel reduced_humanoid_geometry;
  buildReducedModel(
    humanoid_model, humanoid_geometry, joints_to_lock, reference_config_humanoid,
    reduced_humanoid_model, reduced_humanoid_geometry);

  BOOST_CHECK(reduced_humanoid_geometry.ngeoms == humanoid_geometry.ngeoms);
  BOOST_CHECK(reduced_humanoid_geometry.collisionPairs == humanoid_geometry.collisionPairs);
  BOOST_CHECK(
    reduced_humanoid_geometry.geometryObjects.size() == humanoid_geometry.geometryObjects.size());

  for (Index i = 0; i < humanoid_geometry.geometryObjects.size(); ++i)
  {
    const GeometryObject & go1 = humanoid_geometry.geometryObjects[i];
    const GeometryObject & go2 = reduced_humanoid_geometry.geometryObjects[i];
    BOOST_CHECK_EQUAL(go1.name, go2.name);
    BOOST_CHECK_EQUAL(go1.geometry, go2.geometry);
    BOOST_CHECK_EQUAL(go1.meshPath, go2.meshPath);
    BOOST_CHECK_EQUAL(go1.meshScale, go2.meshScale);
    BOOST_CHECK_EQUAL(go1.overrideMaterial, go2.overrideMaterial);
    BOOST_CHECK_EQUAL(go1.meshColor, go2.meshColor);
    BOOST_CHECK_EQUAL(go1.meshTexturePath, go2.meshTexturePath);
    BOOST_CHECK_EQUAL(go1.parentFrame, go2.parentFrame);
    BOOST_CHECK_EQUAL(
      humanoid_model.frames[go1.parentFrame].name,
      reduced_humanoid_model.frames[go2.parentFrame].name);
  }

  Data data(humanoid_model), reduced_data(reduced_humanoid_model);
  const Eigen::VectorXd q = reference_config_humanoid;
  Eigen::VectorXd reduced_q(reduced_humanoid_model.nq);

  for (JointIndex joint_id = 1; joint_id < (JointIndex)reduced_humanoid_model.njoints; ++joint_id)
  {
    const JointIndex reference_joint_id =
      humanoid_model.getJointId(reduced_humanoid_model.names[joint_id]);

    reduced_humanoid_model.joints[joint_id].jointConfigSelector(reduced_q) =
      humanoid_model.joints[reference_joint_id].jointConfigSelector(q);
  }

  framesForwardKinematics(humanoid_model, data, q);
  framesForwardKinematics(reduced_humanoid_model, reduced_data, reduced_q);

  for (size_t frame_id = 0; frame_id < reduced_humanoid_model.frames.size(); ++frame_id)
  {
    const Frame & reduced_frame = reduced_humanoid_model.frames[frame_id];
    switch (reduced_frame.type)
    {
    case JOINT:
    case FIXED_JOINT: {
      // May not be present in the original model
      if (humanoid_model.existJointName(reduced_frame.name))
      {
        const JointIndex joint_id = humanoid_model.getJointId(reduced_frame.name);
        BOOST_CHECK(data.oMi[joint_id].isApprox(reduced_data.oMf[frame_id]));
      }
      else if (humanoid_model.existFrame(reduced_frame.name))
      {
        const FrameIndex humanoid_frame_id = humanoid_model.getFrameId(reduced_frame.name);
        BOOST_CHECK(data.oMf[humanoid_frame_id].isApprox(reduced_data.oMf[frame_id]));
      }
      else
      {
        BOOST_CHECK_MESSAGE(
          false, "The frame " << reduced_frame.name << " is not presend in the humanoid_model");
      }
      break;
    }
    default: {
      BOOST_CHECK(humanoid_model.existFrame(reduced_frame.name));
      const FrameIndex humanoid_frame_id = humanoid_model.getFrameId(reduced_frame.name);
      BOOST_CHECK(data.oMf[humanoid_frame_id].isApprox(reduced_data.oMf[frame_id]));
      break;
    }
    }
  }

  // Test GeometryObject placements
  GeometryData geom_data(humanoid_geometry), reduded_geom_data(reduced_humanoid_geometry);
  updateGeometryPlacements(humanoid_model, data, humanoid_geometry, geom_data);
  updateGeometryPlacements(
    reduced_humanoid_model, reduced_data, reduced_humanoid_geometry, reduded_geom_data);

  BOOST_CHECK(geom_data.oMg.size() == reduded_geom_data.oMg.size());
  for (FrameIndex i = 0; i < geom_data.oMg.size(); ++i)
  {
    BOOST_CHECK(geom_data.oMg[i].isApprox(reduded_geom_data.oMg[i]));
  }

  // Test other signature
  std::vector<GeometryModel> full_geometry_models;
  full_geometry_models.push_back(humanoid_geometry);
  full_geometry_models.push_back(humanoid_geometry);
  full_geometry_models.push_back(humanoid_geometry);

  std::vector<GeometryModel> reduced_geometry_models;

  Model reduced_humanoid_model_other_sig;
  buildReducedModel(
    humanoid_model, full_geometry_models, joints_to_lock, reference_config_humanoid,
    reduced_humanoid_model_other_sig, reduced_geometry_models);

  BOOST_CHECK(reduced_geometry_models[0] == reduced_humanoid_geometry);
  BOOST_CHECK(reduced_geometry_models[1] == reduced_humanoid_geometry);
  BOOST_CHECK(reduced_geometry_models[2] == reduced_humanoid_geometry);
}
#endif // PINOCCHIO_WITH_HPP_FCL

BOOST_AUTO_TEST_CASE(test_findCommonAncestor)
{
  Model model;
  buildModels::humanoid(model);

  {
    size_t id_ancestor1, id_ancestor2;
    JointIndex ancestor = findCommonAncestor(model, 0, 0, id_ancestor1, id_ancestor2);
    BOOST_CHECK(ancestor == 0);
    BOOST_CHECK(id_ancestor1 == 0);
    BOOST_CHECK(id_ancestor2 == 0);
  }

  {
    size_t id_ancestor1, id_ancestor2;
    JointIndex ancestor =
      findCommonAncestor(model, 0, (JointIndex)(model.njoints - 1), id_ancestor1, id_ancestor2);
    BOOST_CHECK(ancestor == 0);
    BOOST_CHECK(id_ancestor1 == 0);
    BOOST_CHECK(id_ancestor2 == 0);
  }

  {
    size_t id_ancestor1, id_ancestor2;
    JointIndex ancestor =
      findCommonAncestor(model, (JointIndex)(model.njoints - 1), 0, id_ancestor1, id_ancestor2);
    BOOST_CHECK(ancestor == 0);
    BOOST_CHECK(id_ancestor1 == 0);
    BOOST_CHECK(id_ancestor2 == 0);
  }

  {
    size_t id_ancestor1, id_ancestor2;
    JointIndex ancestor =
      findCommonAncestor(model, (JointIndex)(model.njoints - 1), 1, id_ancestor1, id_ancestor2);
    BOOST_CHECK(ancestor == 1);
    BOOST_CHECK(model.supports[(JointIndex)(model.njoints - 1)][id_ancestor1] == ancestor);
    BOOST_CHECK(model.supports[1][id_ancestor2] == ancestor);
  }
}

BOOST_AUTO_TEST_CASE(test_has_configuration_limit)
{
  using namespace Eigen;

  // Test joint specific function hasConfigurationLimit
  JointModelFreeFlyer test_joint_ff = JointModelFreeFlyer();
  std::vector<bool> cf_limits_ff = test_joint_ff.hasConfigurationLimit();
  std::vector<bool> cf_limits_tangent_ff = test_joint_ff.hasConfigurationLimitInTangent();
  std::vector<bool> expected_cf_limits_ff({true, true, true, false, false, false, false});
  std::vector<bool> expected_cf_limits_tangent_ff({true, true, true, false, false, false});
  BOOST_CHECK(cf_limits_ff == expected_cf_limits_ff);
  BOOST_CHECK(cf_limits_tangent_ff == expected_cf_limits_tangent_ff);

  JointModelPlanar test_joint_planar = JointModelPlanar();
  std::vector<bool> cf_limits_planar = test_joint_planar.hasConfigurationLimit();
  std::vector<bool> cf_limits_tangent_planar = test_joint_planar.hasConfigurationLimitInTangent();
  std::vector<bool> expected_cf_limits_planar({true, true, false, false});
  std::vector<bool> expected_cf_limits_tangent_planar({true, true, false});
  BOOST_CHECK(cf_limits_planar == expected_cf_limits_planar);
  BOOST_CHECK(cf_limits_tangent_planar == expected_cf_limits_tangent_planar);

  JointModelPX test_joint_p = JointModelPX();
  std::vector<bool> cf_limits_prismatic = test_joint_p.hasConfigurationLimit();
  std::vector<bool> cf_limits_tangent_prismatic = test_joint_p.hasConfigurationLimitInTangent();
  std::vector<bool> expected_cf_limits_prismatic({true});
  std::vector<bool> expected_cf_limits_tangent_prismatic({true});
  BOOST_CHECK(cf_limits_prismatic == expected_cf_limits_prismatic);
  BOOST_CHECK(cf_limits_tangent_prismatic == expected_cf_limits_tangent_prismatic);

  // Test model.hasConfigurationLimit() function
  Model model;
  JointIndex jointId = 0;

  jointId = model.addJoint(jointId, JointModelFreeFlyer(), SE3::Identity(), "Joint0");
  jointId = model.addJoint(jointId, JointModelRZ(), SE3::Identity(), "Joint1");
  jointId = model.addJoint(
    jointId, JointModelRUBZ(), SE3(Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)), "Joint2");

  std::vector<bool> expected_cf_limits_model(
    {true, true, true,           // translation of FF
     false, false, false, false, // rotation of FF
     true,                       // roational joint
     false, false});             // unbounded rotational joint
  std::vector<bool> model_cf_limits = model.hasConfigurationLimit();
  BOOST_CHECK((model_cf_limits == expected_cf_limits_model));

  // Test model.hasConfigurationLimitInTangent() function
  std::vector<bool> expected_cf_limits_tangent_model(
    {true, true, true,    // translation of FF
     false, false, false, // rotation of FF
     true,                // roational joint
     false});             // unbounded rotational joint
  std::vector<bool> model_cf_limits_tangent = model.hasConfigurationLimitInTangent();
  BOOST_CHECK((model_cf_limits_tangent == expected_cf_limits_tangent_model));
}

BOOST_AUTO_TEST_SUITE_END()
