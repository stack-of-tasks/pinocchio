//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

#include <eigenpy/variant.hpp>

namespace pinocchio
{
  namespace graph
  {
    // Define all static const int members for Boost.Python exposure
    const int JointFixed::nq;
    const int JointFixed::nv;

    const int JointRevolute::nq;
    const int JointRevolute::nv;

    const int JointRevoluteUnbounded::nq;
    const int JointRevoluteUnbounded::nv;

    const int JointPrismatic::nq;
    const int JointPrismatic::nv;

    const int JointFreeFlyer::nq;
    const int JointFreeFlyer::nv;

    const int JointSpherical::nq;
    const int JointSpherical::nv;

    const int JointSphericalZYX::nq;
    const int JointSphericalZYX::nv;

    const int JointTranslation::nq;
    const int JointTranslation::nv;

    const int JointPlanar::nq;
    const int JointPlanar::nv;

    const int JointHelical::nq;
    const int JointHelical::nv;

    const int JointUniversal::nq;
    const int JointUniversal::nv;

    const int JointMimic::nq;
    const int JointMimic::nv;
  } // namespace graph

  namespace python
  {
    namespace bp = boost::python;

    void exposeJointsGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<JointFixed>(
        "JointFixed", "Represents a fixed joint in the graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::SE3 &>(
          bp::args("self", "pose"), "Constructor with joint offset."))
        .def_readwrite("joint_offset", &JointFixed::joint_offset, "Offset of the joint.")
        .def_readonly("nq", &JointFixed::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointFixed::nv, "Number of tangent variables.");

      bp::class_<JointRevolute>(
        "JointRevolute", "Represents a revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevolute::axis, "Rotation axis.")
        .def_readonly("nq", &JointRevolute::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointRevolute::nv, "Number of tangent variables.");

      bp::class_<JointRevoluteUnbounded>(
        "JointRevoluteUnbounded", "Represents an unbounded revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevoluteUnbounded::axis, "Rotation axis.")
        .def_readonly("nq", &JointRevoluteUnbounded::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointRevoluteUnbounded::nv, "Number of tangent variables.");

      bp::class_<JointPrismatic>(
        "JointPrismatic", "Represents a prismatic joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with translation axis."))
        .def_readwrite("axis", &JointPrismatic::axis, "Translation axis.")
        .def_readonly("nq", &JointPrismatic::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointPrismatic::nv, "Number of tangent variables.");

      bp::class_<JointFreeFlyer>(
        "JointFreeFlyer", "Represents a free-flyer joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointFreeFlyer::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointFreeFlyer::nv, "Number of tangent variables.");

      bp::class_<JointSpherical>(
        "JointSpherical", "Represents a spherical joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointSpherical::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointSpherical::nv, "Number of tangent variables.");

      bp::class_<JointSphericalZYX>(
        "JointSphericalZYX", "Represents a spherical ZYX joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointSphericalZYX::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointSphericalZYX::nv, "Number of tangent variables.");

      bp::class_<JointTranslation>(
        "JointTranslation", "Represents a translation joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointTranslation::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointTranslation::nv, "Number of tangent variables.");

      bp::class_<JointPlanar>(
        "JointPlanar", "Represents a planar joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointPlanar::nq, "Number of configuration variables ")
        .def_readonly("nv", &JointPlanar::nv, "Number of tangent variables ");

      bp::class_<JointHelical>(
        "JointHelical", "Represents a helical joint.",
        bp::init<const Eigen::Vector3d &, double>(
          bp::args("self", "axis", "pitch"), "Constructor with axis and pitch."))
        .def_readwrite("axis", &JointHelical::axis, "Axis of the helical joint.")
        .def_readwrite("pitch", &JointHelical::pitch, "Pitch of the helical joint.")
        .def_readonly("nq", &JointHelical::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointHelical::nv, "Number of tangent variables.");

      bp::class_<JointUniversal>(
        "JointUniversal", "Represents a universal joint.",
        bp::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(
          bp::args("self", "axis1", "axis2"), "Constructor with two axes."))
        .def_readwrite("axis1", &JointUniversal::axis1, "First axis of the universal joint.")
        .def_readwrite("axis2", &JointUniversal::axis2, "Second axis of the universal joint.")
        .def_readonly("nq", &JointUniversal::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointUniversal::nv, "Number of tangent variables.");

      bp::class_<JointComposite>(
        "JointComposite", "Represents a composite joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointVariant &, const pinocchio::SE3 &>(
          bp::args("self", "joint_variant", "joint_pose"),
          "Constructor with a single joint and its placement."))
        .def(bp::init<const std::vector<JointVariant> &, const std::vector<SE3> &>(
          bp::args("self", "joints_variants", "joint_poses"),
          "Constructor with multiple joints and their placements."))
        .def_readwrite("joints", &JointComposite::joints, "List of joints in the composite joint.")
        .def_readwrite(
          "jointsPlacements", &JointComposite::jointsPlacements,
          "List of placements for the joints.")
        .def_readwrite(
          "nq", &JointComposite::nq,
          "Total number of configuration variables for the composite joint.")
        .def_readwrite(
          "nv", &JointComposite::nv,
          "Total number of configuration variables for the composite joint.")
        .def(
          "addJoint", &JointComposite::addJoint,
          (bp::arg("self"), bp::arg("joint_model"), bp::arg("pose") = pinocchio::SE3::Identity()),
          "Adds a joint to the composite joint with an optional placement.");

      bp::class_<JointMimic>(
        "JointMimic", "Represents a mimic joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointVariant &, const std::string &, double, double>(
          bp::args("self", "secondary_joint_model", "primary_name", "scaling", "offset"),
          "Constructor for mimic joint."))
        .def_readwrite(
          "primary_name", &JointMimic::primary_name, "Name of the primary joint being mimicked.")
        .def_readwrite(
          "secondary_joint", &JointMimic::secondary_joint,
          "The model of the secondary (mimicking) joint.")
        .def_readwrite("scaling", &JointMimic::scaling, "Scaling factor for the mimicry.")
        .def_readwrite("offset", &JointMimic::offset, "Offset for the mimicry.")
        .def_readonly(
          "nq", &JointMimic::nq,
          "Number of configuration variables (0 for mimic, as it depends on primary).")
        .def_readonly(
          "nv", &JointMimic::nv,
          "Number of tangent variables (0 for mimic, as it depends on primary).");

      typedef eigenpy::VariantConverter<JointVariant> Converter;
      Converter::registration();

      StdAlignedVectorPythonVisitor<JointVariant>::expose("StdVec_JointVariant");
    }

    void exposeJointLimits()
    {
      using namespace pinocchio::graph;

      bp::class_<JointLimits>("JointLimits", bp::init<>())
        .def_readwrite("maxEffort", &JointLimits::maxEffort, "Max effort ")
        .def_readwrite("maxVel", &JointLimits::maxVel, "Max velocity ")
        .def_readwrite("maxConfig", &JointLimits::maxConfig, "Max position ")
        .def_readwrite("minConfig", &JointLimits::minConfig, "Min position ")
        .def_readwrite("friction", &JointLimits::friction, "Friction ")
        .def_readwrite("damping", &JointLimits::damping, "Damping ")
        .def_readwrite("armature", &JointLimits::armature, "Armature inertia ")
        .def_readwrite("frictionLoss", &JointLimits::frictionLoss, "Dry friction loss ")
        .def(
          "append", &JointLimits::append, (bp::arg("jlimit"), bp::arg("nq"), bp::arg("nv")),
          "Appends data from another JointLimits object.");
    }

    void exposeEdgesAlgo()
    {
      using namespace pinocchio::graph;

      bp::class_<EdgeParameters>(
        "EdgeParameters", "Parameters for defining an edge (joint) in the ModelGraph.")
        .def(bp::init<>())
        .def(bp::init<
             const std::string &, const std::string &, const SE3 &, const std::string &,
             const SE3 &, const JointVariant &, const boost::optional<Eigen::VectorXd>>(
          (bp::arg("name"), bp::arg("source_vertex"), bp::arg("source_to_joint"),
           bp::arg("target_vertex"), bp::arg("joint_to_target"), bp::arg("joint"),
           bp::arg("q_ref") = boost::none),
          "Constructor to define an edge with specific parameters."))
        .def_readwrite("name", &EdgeParameters::name, "Name of the edge/joint.")
        .def_readwrite(
          "source_vertex", &EdgeParameters::source_vertex,
          "Name of the source vertex (parent body).")
        .def_readwrite(
          "target_vertex", &EdgeParameters::target_vertex,
          "Name of the target vertex (child body).")
        .def_readwrite(
          "source_to_joint", &EdgeParameters::source_to_joint,
          "Transformation from source_vertex to the joint origin.")
        .def_readwrite(
          "joint_to_target", &EdgeParameters::joint_to_target,
          "Transformation from joint origin to the target_vertex.")
        .def_readwrite(
          "q_ref", &EdgeParameters::q_ref, "Optional reference configuration for the joint.")
        .def_readwrite(
          "joint", &EdgeParameters::joint, "Type of the joint (e.g., fixed, revolute, prismatic).")
        .def_readwrite("jlimit", &EdgeParameters::jlimit, "Limits of the joint");

      bp::class_<EdgeBuilder>(
        "EdgeBuilder",
        "A builder class for conveniently constructing and adding edges (joints) to a ModelGraph.",
        bp::init<ModelGraph &>(
          (bp::arg("graph")), "Constructs an EdgeBuilder associated with a ModelGraph instance."))
        .def(
          "withName", &EdgeBuilder::withName, bp::return_self<>(), bp::arg("name"),
          "Sets the name of the edge/joint")
        .def(
          "withTargetVertex", &EdgeBuilder::withTargetVertex, bp::return_self<>(),
          bp::arg("target_name"), "Sets the target vertex name")
        .def(
          "withSourceVertex", &EdgeBuilder::withSourceVertex, bp::return_self<>(),
          bp::arg("source_name"), "Sets the source vertex name.")
        .def(
          "withTargetPose", &EdgeBuilder::withTargetPose, bp::return_self<>(),
          bp::arg("target_pose"), "Sets the transformation from joint origin to target vertex.")
        .def(
          "withSourcePose", &EdgeBuilder::withSourcePose, bp::return_self<>(),
          bp::arg("source_pose"), "Sets the transformation from source vertex to joint origin")
        .def(
          "withJointType", &EdgeBuilder::withJointType, bp::return_self<>(), bp::arg("jtype"),
          "Sets the type of the joint.")
        .def(
          "withQref", &EdgeBuilder::withQref, bp::return_self<>(), bp::arg("qref"),
          "Sets the optional reference configuration for the joint")
        .def(
          "withMinConfig", &EdgeBuilder::withMinConfig, bp::return_self<>(), bp::arg("minConfig"),
          "Sets the min configuration")
        .def(
          "withMaxConfig", &EdgeBuilder::withMaxConfig, bp::return_self<>(), bp::arg("maxConfig"),
          "Sets the max configuration")
        .def(
          "withMaxVel", &EdgeBuilder::withMaxVel, bp::return_self<>(), bp::arg("maxVel"),
          "Sets the maximum velocity")
        .def(
          "withMaxEffort", &EdgeBuilder::withMaxEffort, bp::return_self<>(), bp::arg("maxEffort"),
          "Sets the maximum effort")
        .def(
          "withDamping", &EdgeBuilder::withDamping, bp::return_self<>(), bp::arg("damping"),
          "Sets the damping")
        .def(
          "withFriction", &EdgeBuilder::withFriction, bp::return_self<>(), bp::arg("friction"),
          "Sets the Friction")
        .def(
          "withArmature", &EdgeBuilder::withArmature, bp::return_self<>(), bp::arg("armature"),
          "Sets joint's armature")
        .def(
          "withFrictionLoss", &EdgeBuilder::withFrictionLoss, bp::return_self<>(),
          bp::arg("frictionLoss"), "Sets friction loss")
        .def(
          "build", &EdgeBuilder::build,
          "Builds the edge/joint parameters and adds the joint to the associated ModelGraph.")
        .def_readwrite(
          "param", &EdgeBuilder::param, "Direct access to the EdgeParameters object being built.");
    }
  } // namespace python
} // namespace pinocchio
