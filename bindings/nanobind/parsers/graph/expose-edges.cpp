// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/boost-variant.hpp"

#include "pinocchio/parsers/graph.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/optional.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeJointsGraph(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::class_<JointFixed>(m, "JointFixed", "Represents a fixed joint in the graph.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const pinocchio::SE3 &>(), "pose"_a, "Constructor with joint offset.")
    .def_rw("joint_offset", &JointFixed::joint_offset, "Offset of the joint.")
    .def_ro_static("nq", &JointFixed::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointFixed::nv, "Number of tangent variables.");

  nb::class_<JointRevolute>(m, "JointRevolute", "Represents a revolute joint.")
    .def(nb::init<const Eigen::Vector3d &>(), "axis"_a, "Constructor with rotation axis.")
    .def_rw("axis", &JointRevolute::axis, "Rotation axis.")
    .def_ro_static("nq", &JointRevolute::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointRevolute::nv, "Number of tangent variables.");

  nb::class_<JointRevoluteUnbounded>(
    m, "JointRevoluteUnbounded", "Represents an unbounded revolute joint.")
    .def(nb::init<const Eigen::Vector3d &>(), "axis"_a, "Constructor with rotation axis.")
    .def_rw("axis", &JointRevoluteUnbounded::axis, "Rotation axis.")
    .def_ro_static("nq", &JointRevoluteUnbounded::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointRevoluteUnbounded::nv, "Number of tangent variables.");

  nb::class_<JointPrismatic>(m, "JointPrismatic", "Represents a prismatic joint.")
    .def(nb::init<const Eigen::Vector3d &>(), "axis"_a, "Constructor with translation axis.")
    .def_rw("axis", &JointPrismatic::axis, "Translation axis.")
    .def_ro_static("nq", &JointPrismatic::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointPrismatic::nv, "Number of tangent variables.");

  nb::class_<JointFreeFlyer>(m, "JointFreeFlyer", "Represents a free-flyer joint.")
    .def(nb::init<>(), "Default constructor.")
    .def_ro_static("nq", &JointFreeFlyer::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointFreeFlyer::nv, "Number of tangent variables.");

  nb::class_<JointSpherical>(m, "JointSpherical", "Represents a spherical joint.")
    .def(nb::init<>(), "Default constructor.")
    .def_ro_static("nq", &JointSpherical::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointSpherical::nv, "Number of tangent variables.");

  nb::class_<JointSphericalZYX>(m, "JointSphericalZYX", "Represents a spherical ZYX joint.")
    .def(nb::init<>(), "Default constructor.")
    .def_ro_static("nq", &JointSphericalZYX::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointSphericalZYX::nv, "Number of tangent variables.");

  nb::class_<JointEllipsoid>(m, "JointEllipsoid", "Represents an ellipsoidal joint.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<double, double, double>(), "radius_x"_a, "radius_y"_a, "radius_z"_a,
      "Constructor with radii.")
    .def_rw("radius_x", &JointEllipsoid::radius_x, "Semi-axis length in the x direction.")
    .def_rw("radius_y", &JointEllipsoid::radius_y, "Semi-axis length in the y direction.")
    .def_rw("radius_z", &JointEllipsoid::radius_z, "Semi-axis length in the z direction.")
    .def_ro_static("nq", &JointEllipsoid::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointEllipsoid::nv, "Number of tangent variables.");

  nb::class_<JointTranslation>(m, "JointTranslation", "Represents a translation joint.")
    .def(nb::init<>(), "Default constructor.")
    .def_ro_static("nq", &JointTranslation::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointTranslation::nv, "Number of tangent variables.");

  nb::class_<JointPlanar>(m, "JointPlanar", "Represents a planar joint.")
    .def(nb::init<>(), "Default constructor.")
    .def_ro_static("nq", &JointPlanar::nq, "Number of configuration variables ")
    .def_ro_static("nv", &JointPlanar::nv, "Number of tangent variables ");

  nb::class_<JointHelical>(m, "JointHelical", "Represents a helical joint.")
    .def(
      nb::init<const Eigen::Vector3d &, double>(), "axis"_a, "pitch"_a,
      "Constructor with axis and pitch.")
    .def_rw("axis", &JointHelical::axis, "Axis of the helical joint.")
    .def_rw("pitch", &JointHelical::pitch, "Pitch of the helical joint.")
    .def_ro_static("nq", &JointHelical::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointHelical::nv, "Number of tangent variables.");

  nb::class_<JointUniversal>(m, "JointUniversal", "Represents a universal joint.")
    .def(
      nb::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(), "axis1"_a, "axis2"_a,
      "Constructor with two axes.")
    .def_rw("axis1", &JointUniversal::axis1, "First axis of the universal joint.")
    .def_rw("axis2", &JointUniversal::axis2, "Second axis of the universal joint.")
    .def_ro_static("nq", &JointUniversal::nq, "Number of configuration variables.")
    .def_ro_static("nv", &JointUniversal::nv, "Number of tangent variables.");

  nb::class_<JointComposite>(m, "JointComposite", "Represents a composite joint.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const JointVariant &, const pinocchio::SE3 &>(), "joint_variant"_a, "joint_pose"_a,
      "Constructor with a single joint and its placement.")
    .def(
      nb::init<const std::vector<JointVariant> &, const std::vector<SE3> &>(), "joints_variants"_a,
      "joint_poses"_a, "Constructor with multiple joints and their placements.")
    .def_rw("joints", &JointComposite::joints, "List of joints in the composite joint.")
    .def_rw(
      "jointsPlacements", &JointComposite::jointsPlacements, "List of placements for the joints.")
    .def_rw(
      "nq", &JointComposite::nq, "Total number of configuration variables for the composite joint.")
    .def_rw(
      "nv", &JointComposite::nv, "Total number of configuration variables for the composite joint.")
    .def(
      "addJoint", &JointComposite::addJoint, "joint_model"_a, "pose"_a = pinocchio::SE3::Identity(),
      "Adds a joint to the composite joint with an optional placement.");

  nb::class_<JointMimic>(m, "JointMimic", "Represents a mimic joint.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const JointVariant &, const std::string &, double, double>(),
      "secondary_joint_model"_a, "primary_name"_a, "scaling"_a, "offset"_a,
      "Constructor for mimic joint.")
    .def_rw("primary_name", &JointMimic::primary_name, "Name of the primary joint being mimicked.")
    .def_rw(
      "secondary_joint", &JointMimic::secondary_joint,
      "The model of the secondary (mimicking) joint.")
    .def_rw("scaling", &JointMimic::scaling, "Scaling factor for the mimicry.")
    .def_rw("offset", &JointMimic::offset, "Offset for the mimicry.")
    .def_ro_static(
      "nq", &JointMimic::nq,
      "Number of configuration variables (0 for mimic, as it depends on primary).")
    .def_ro_static(
      "nv", &JointMimic::nv,
      "Number of tangent variables (0 for mimic, as it depends on primary).");

  nb::bind_vector<std::vector<JointVariant>>(m, "StdVec_JointVariant");
}

void exposeJointLimits(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::class_<JointLimits>(m, "JointLimits")
    .def(nb::init<>(), "Default constructor.")
    .def_rw("maxEffort", &JointLimits::maxEffort, "Max effort ")
    .def_rw("maxVel", &JointLimits::maxVel, "Max velocity ")
    .def_rw("maxConfig", &JointLimits::maxConfig, "Max position ")
    .def_rw("minConfig", &JointLimits::minConfig, "Min position ")
    .def_rw("friction", &JointLimits::friction, "Friction ")
    .def_rw("damping", &JointLimits::damping, "Damping ")
    .def_rw("armature", &JointLimits::armature, "Armature inertia ")
    .def_rw("frictionLoss", &JointLimits::frictionLoss, "Dry friction loss ")
    .def(
      "append", &JointLimits::append, "jlimit"_a, "nq"_a, "nv"_a,
      "Appends data from another JointLimits object.");
}

void exposeEdgesAlgo(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::class_<EdgeParameters>(
    m, "EdgeParameters", "Parameters for defining an edge (joint) in the ModelGraph.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      "__init__",
      [](
        EdgeParameters * self, const std::string & name, const std::string & source_vertex,
        const SE3 & source_to_joint, const std::string & target_vertex, const SE3 & joint_to_target,
        const JointVariant & joint, const std::optional<Eigen::VectorXd> & q_ref) {
        new (self) EdgeParameters(
          name, source_vertex, source_to_joint, target_vertex, joint_to_target, joint,
          q_ref ? boost::optional<Eigen::VectorXd>(*q_ref) : boost::none);
      },
      "name"_a, "source_vertex"_a, "source_to_joint"_a, "target_vertex"_a, "joint_to_target"_a,
      "joint"_a, "q_ref"_a = std::nullopt,
      "Constructor to define an edge with specific parameters.")
    .def_rw("name", &EdgeParameters::name, "Name of the edge/joint.")
    .def_rw(
      "source_vertex", &EdgeParameters::source_vertex, "Name of the source vertex (parent body).")
    .def_rw(
      "target_vertex", &EdgeParameters::target_vertex, "Name of the target vertex (child body).")
    .def_rw(
      "source_to_joint", &EdgeParameters::source_to_joint,
      "Transformation from source_vertex to the joint origin.")
    .def_rw(
      "joint_to_target", &EdgeParameters::joint_to_target,
      "Transformation from joint origin to the target_vertex.")
    .def_rw("q_ref", &EdgeParameters::q_ref, "Optional reference configuration for the joint.")
    .def_rw(
      "joint", &EdgeParameters::joint, "Type of the joint (e.g., fixed, revolute, prismatic).")
    .def_rw("jlimit", &EdgeParameters::jlimit, "Limits of the joint");

  nb::class_<EdgeBuilder>(
    m, "EdgeBuilder",
    "A builder class for conveniently constructing and adding edges (joints) to a ModelGraph.")
    .def(
      nb::init<ModelGraph &>(), "graph"_a,
      "Constructs an EdgeBuilder associated with a ModelGraph instance.")
    .def("withName", &EdgeBuilder::withName, "name"_a, "Sets the name of the edge/joint")
    .def(
      "withTargetVertex", &EdgeBuilder::withTargetVertex, "target_name"_a,
      "Sets the target vertex name")
    .def(
      "withSourceVertex", &EdgeBuilder::withSourceVertex, "source_name"_a,
      "Sets the source vertex name.")
    .def(
      "withTargetPose", &EdgeBuilder::withTargetPose, "target_pose"_a,
      "Sets the transformation from joint origin to target vertex.")
    .def(
      "withSourcePose", &EdgeBuilder::withSourcePose, "source_pose"_a,
      "Sets the transformation from source vertex to joint origin")
    .def("withJointType", &EdgeBuilder::withJointType, "jtype"_a, "Sets the type of the joint.")
    .def(
      "withQref", &EdgeBuilder::withQref, "qref"_a,
      "Sets the optional reference configuration for the joint")
    .def("withMinConfig", &EdgeBuilder::withMinConfig, "minConfig"_a, "Sets the min configuration")
    .def("withMaxConfig", &EdgeBuilder::withMaxConfig, "maxConfig"_a, "Sets the max configuration")
    .def("withMaxVel", &EdgeBuilder::withMaxVel, "maxVel"_a, "Sets the maximum velocity")
    .def("withMaxEffort", &EdgeBuilder::withMaxEffort, "maxEffort"_a, "Sets the maximum effort")
    .def("withDamping", &EdgeBuilder::withDamping, "damping"_a, "Sets the damping")
    .def("withFriction", &EdgeBuilder::withFriction, "friction"_a, "Sets the Friction")
    .def("withArmature", &EdgeBuilder::withArmature, "armature"_a, "Sets joint's armature")
    .def("withFrictionLoss", &EdgeBuilder::withFrictionLoss, "frictionLoss"_a, "Sets friction loss")
    .def(
      "build", &EdgeBuilder::build,
      "Builds the edge/joint parameters and adds the joint to the associated ModelGraph.")
    .def_rw(
      "param", &EdgeBuilder::param, "Direct access to the EdgeParameters object being built.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
