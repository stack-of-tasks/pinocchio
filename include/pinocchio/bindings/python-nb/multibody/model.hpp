// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"
#include "pinocchio/algorithm/check-data.hpp"

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/copyable.hpp"
#include "../utils/printable.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Model>
void exposeModel(nb::module_ m)
{
  using namespace nb::literals;
  using Scalar = typename Model::Scalar;
  using Index = typename Model::Index;
  using JointIndex = typename Model::JointIndex;
  using IndexVector = typename Model::IndexVector;
  using SE3 = typename Model::SE3;
  using Data = typename Model::Data;
  using JointModel = typename Model::JointModel;
  using VectorXs = typename Model::VectorXs;

  constexpr pinocchio::FrameType kAllFrameTypes = static_cast<pinocchio::FrameType>(
    pinocchio::JOINT | pinocchio::FIXED_JOINT | pinocchio::BODY | pinocchio::OP_FRAME
    | pinocchio::SENSOR);

  nb::bind_map<typename Model::ConfigVectorMap>(m, "StdMap_String_VectorXd");

  nb::class_<Model>(m, "Model", "Articulated rigid-body model.")
    .def(nb::init<>(), "Default constructor")
    .def(nb::init<const Model &>(), "other"_a)
    // --- dimensions
    .def_ro("nq", &Model::nq, "Dimension of the configuration vector representation.")
    .def_ro("nv", &Model::nv, "Dimension of the velocity vector space.")
    .def_ro("nvExtended", &Model::nvExtended, "Dimension of the Jacobian matrix input space.")
    .def_ro("njoints", &Model::njoints, "Number of joints.")
    .def_ro("nbodies", &Model::nbodies, "Number of bodies.")
    .def_ro("nframes", &Model::nframes, "Number of frames.")
    // --- joint-indexed data
    .def_ro("inertias", &Model::inertias, "Vector of spatial inertias supported by each joint.")
    .def_rw(
      "jointPlacements", &Model::jointPlacements,
      "Vector of joint placements: placement of joint i with respect to its parent joint frame.")
    .def_ro("joints", &Model::joints, "Vector of joint models.")
    .def_ro(
      "idx_qs", &Model::idx_qs,
      "Vector of starting index of the i-th joint in the configuration space.")
    .def_ro("nqs", &Model::nqs, "Vector of dimension of the joint configuration subspace.")
    .def_ro(
      "idx_vs", &Model::idx_vs,
      "Starting index of the i-th joint in the tangent configuration space.")
    .def_ro("nvs", &Model::nvs, "Dimension of the i-th joint tangent subspace.")
    .def_ro(
      "idx_vExtendeds", &Model::idx_vExtendeds,
      "Starting index of the i-th joint in the Jacobian space.")
    .def_ro("nvExtendeds", &Model::nvExtendeds, "Dimension of the i-th joint Jacobian subspace.")
    .def_ro(
      "parents", &Model::parents,
      "Vector of parent joint indexes. The parent of joint i is parents[i].")
    .def_ro(
      "children", &Model::children,
      "Vector of children joints. Children of joint i are children[i].")
    .def_ro("names", &Model::names, "Name of the joints.")
    .def_rw("name", &Model::name, "Name of the model.")
    .def_rw(
      "referenceConfigurations", &Model::referenceConfigurations,
      "Map of reference configurations, indexed by user given names.")
    // --- dynamics parameters
    .def_rw("armature", &Model::armature, "Armature vector.")
    .def_rw("rotorInertia", &Model::rotorInertia, "Vector of rotor inertia parameters.")
    .def_rw("rotorGearRatio", &Model::rotorGearRatio, "Vector of rotor gear ratio parameters.")
    .def_rw(
      "upperDryFrictionLimit", &Model::upperDryFrictionLimit, "Vector of maximum joint friction.")
    .def_rw(
      "lowerDryFrictionLimit", &Model::lowerDryFrictionLimit, "Vector of minimum joint friction.")
    .def_prop_rw(
      "friction", [](const Model & m) { return m.upperDryFrictionLimit; },
      [](Model & m, const VectorXs & v) { m.upperDryFrictionLimit = v; },
      "Deprecated. Alias for upperDryFrictionLimit.")
    .def_rw("damping", &Model::damping, "Vector of joint damping parameters.")
    .def_rw("upperEffortLimit", &Model::upperEffortLimit, "Joint max effort.")
    .def_rw("lowerEffortLimit", &Model::lowerEffortLimit, "Joint min effort.")
    .def_prop_rw(
      "effortLimit", [](const Model & m) { return m.upperEffortLimit; },
      [](Model & m, const VectorXs & v) { m.upperEffortLimit = v; },
      "Deprecated. Alias for upperEffortLimit.")
    .def_rw("upperVelocityLimit", &Model::upperVelocityLimit, "Joint max velocity.")
    .def_rw("lowerVelocityLimit", &Model::lowerVelocityLimit, "Joint min velocity.")
    .def_prop_rw(
      "velocityLimit", [](const Model & m) { return m.upperVelocityLimit; },
      [](Model & m, const VectorXs & v) { m.upperVelocityLimit = v; },
      "Deprecated. Alias for upperVelocityLimit.")
    .def_rw("lowerPositionLimit", &Model::lowerPositionLimit, "Lower limit for joint position.")
    .def_rw("upperPositionLimit", &Model::upperPositionLimit, "Upper limit for joint position.")
    .def_rw("positionLimitMargin", &Model::positionLimitMargin, "Margin for joint position limit.")
    // --- kinematic tree data
    .def_rw("frames", &Model::frames, "Vector of frames contained in the model.")
    .def_rw(
      "supports", &Model::supports,
      "Vector of supports. supports[j] is the list of joints on the path from joint j to the root.")
    .def_rw("mimic_joint_supports", &Model::mimic_joint_supports, "Vector of mimic joint supports.")
    .def_rw(
      "subtrees", &Model::subtrees,
      "Vector of subtrees. subtrees[j] is the subtree supported by joint j.")
    .def_rw(
      "sparsity_pattern_vector", &Model::sparsity_pattern_vector,
      "Sparsity pattern for each joint (boolean vector of size nv).")
    .def_rw(
      "span_indexes_vector", &Model::span_indexes_vector,
      "Column-wise span indexes of nonzero entries for each joint.")
    .def_rw(
      "mimicking_joints", &Model::mimicking_joints,
      "Vector of mimicking joints in the tree (with type MimicTpl).")
    .def_rw("mimicked_joints", &Model::mimicked_joints, "Vector of mimicked joints in the tree.")
    .def_rw("gravity", &Model::gravity, "Gravity field expressed in the world frame.")
    .def_prop_ro(
      "gravity981", [](const Model &) { return Model::gravity981; },
      "Default gravity field value on the Earth.")
    // --- model building
    .def(
      "addJoint",
      [](
        Model & self, JointIndex parent_id, const JointModel & jmodel, const SE3 & placement,
        const std::string & joint_name) {
        return self.addJoint(parent_id, jmodel, placement, joint_name);
      },
      "parent_id"_a, "joint_model"_a, "joint_placement"_a, "joint_name"_a,
      "Add a joint to the kinematic tree.")
    .def(
      "addJoint",
      [](
        Model & self, JointIndex parent_id, const JointModel & jmodel, const SE3 & placement,
        const std::string & joint_name, const VectorXs & max_effort, const VectorXs & max_velocity,
        const VectorXs & min_config, const VectorXs & max_config) {
        return self.addJoint(
          parent_id, jmodel, placement, joint_name, max_effort, max_velocity, min_config,
          max_config);
      },
      "parent_id"_a, "joint_model"_a, "joint_placement"_a, "joint_name"_a, "max_effort"_a,
      "max_velocity"_a, "min_config"_a, "max_config"_a,
      "Add a joint to the kinematic tree with given bounds.")
    .def(
      "addJoint",
      [](
        Model & self, JointIndex parent_id, const JointModel & jmodel, const SE3 & placement,
        const std::string & joint_name, const VectorXs & min_effort, const VectorXs & max_effort,
        const VectorXs & min_velocity, const VectorXs & max_velocity, const VectorXs & min_config,
        const VectorXs & max_config, const VectorXs & min_friction, const VectorXs & max_friction,
        const VectorXs & damping) {
        return self.addJoint(
          parent_id, jmodel, placement, joint_name, min_effort, max_effort, min_velocity,
          max_velocity, min_config, max_config, min_friction, max_friction, damping);
      },
      "parent_id"_a, "joint_model"_a, "joint_placement"_a, "joint_name"_a, "min_effort"_a,
      "max_effort"_a, "min_velocity"_a, "max_velocity"_a, "min_config"_a, "max_config"_a,
      "min_friction"_a, "max_friction"_a, "damping"_a,
      "Add a joint to the kinematic tree with given bounds, friction and damping.")
    .def(
      "addJoint",
      [](
        Model & self, JointIndex parent_id, const JointModel & jmodel, const SE3 & placement,
        const std::string & joint_name, const VectorXs & min_effort, const VectorXs & max_effort,
        const VectorXs & min_velocity, const VectorXs & max_velocity, const VectorXs & min_config,
        const VectorXs & max_config, const VectorXs & config_limit_margin,
        const VectorXs & min_friction, const VectorXs & max_friction, const VectorXs & damping) {
        return self.addJoint(
          parent_id, jmodel, placement, joint_name, min_effort, max_effort, min_velocity,
          max_velocity, min_config, max_config, config_limit_margin, min_friction, max_friction,
          damping);
      },
      "parent_id"_a, "joint_model"_a, "joint_placement"_a, "joint_name"_a, "min_effort"_a,
      "max_effort"_a, "min_velocity"_a, "max_velocity"_a, "min_config"_a, "max_config"_a,
      "config_limit_margin"_a, "min_friction"_a, "max_friction"_a, "damping"_a,
      "Add a joint to the kinematic tree with given bounds, config limit margin, friction and "
      "damping.")
    .def(
      "addJointFrame", &Model::addJointFrame, "joint_id"_a, "frame_id"_a = 0,
      "Add the joint given by joint_id as a frame to the frame tree.")
    .def(
      "appendBodyToJoint", &Model::appendBodyToJoint, "joint_id"_a, "body_inertia"_a,
      "body_placement"_a, "Append a body to the joint given by its index.")
    .def(
      "addBodyFrame", &Model::addBodyFrame, "body_name"_a, "parentJoint"_a, "body_placement"_a,
      "previous_frame"_a, "Add a body to the frame tree.")
    .def(
      "getBodyId", &Model::getBodyId, "name"_a,
      "Return the index of a frame of type BODY given by its name.")
    .def(
      "existBodyName", &Model::existBodyName, "name"_a,
      "Check if a frame of type BODY with this name exists.")
    .def(
      "getJointId", &Model::getJointId, "name"_a, "Return the index of a joint given by its name.")
    .def(
      "existJointName", &Model::existJointName, "name"_a, "Check if a joint by this name exists.")
    .def(
      "getFrameId", &Model::getFrameId, "name"_a, "type"_a = kAllFrameTypes,
      "Return the index of the frame given by its name and type. Returns nframes if not found.")
    .def(
      "existFrame", &Model::existFrame, "name"_a, "type"_a = kAllFrameTypes,
      "Return true if a frame with the given name and type exists.")
    .def(
      "addFrame", &Model::addFrame, "frame"_a, "append_inertia"_a = true,
      "Add a frame to the vector of frames.")
    .def("createData", &Model::createData, "Create a Data object for the given model.")
    .def(
      "check", [](const Model & self, const Data & data) { return self.check(data); }, "data"_a,
      "Check consistency of data wrt the model.")
    .def(
      "hasConfigurationLimit", &Model::hasConfigurationLimit,
      "Returns list of booleans of whether joints have a configuration limit.")
    .def(
      "hasConfigurationLimitInTangent", &Model::hasConfigurationLimitInTangent,
      "Returns list of booleans of whether joints have a configuration limit in tangent space.")
    .def(
      "getChildJoints", &Model::getChildJoints,
      "Return the vector of children joints of the kinematic tree.")
    // --- operators
    .def(ComparableVisitor<Model>())
    .def(CopyableVisitor<Model>())
    .def(PrintableVisitor<Model>());

  nb::bind_vector<std::vector<Scalar>>(m, "StdVec_Scalar");
  nb::bind_vector<std::vector<bool>>(m, "StdVec_Bool");
  nb::bind_vector<std::vector<Index>>(m, "StdVec_Index");
  nb::bind_vector<std::vector<IndexVector>, nb::rv_policy::reference_internal>(m, "IndexVecVec");
  nb::bind_vector<std::vector<std::string>>(m, "StdVec_String");
}
PINOCCHIO_PYTHON_NAMESPACE_END
