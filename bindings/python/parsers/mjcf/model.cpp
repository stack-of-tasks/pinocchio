//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <eigenpy/deprecation-policy.hpp>

#include <boost/python.hpp>
#include <boost/filesystem/fstream.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    Model buildModelFromMJCF(const bp::object & filename)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(path(filename), model);
      return model;
    }

    Model buildModelFromMJCFAndRootJoint(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(path(filename), root_joint, root_joint_name, model);
      return model;
    }

    bp::tuple buildModelFromMJCFAndRootJointDeprecated(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      std::vector<RigidConstraintModel> contact_models;
      ::pinocchio::mjcf::buildModel(
        path(filename), root_joint, root_joint_name, model, contact_models);
      return bp::make_tuple(model, contact_models);
    }

    bp::tuple buildModelAndConstraintsFromMJCF(const bp::object & filename)
    {
      Model model;
      std::vector<PointAnchorConstraintModel> point_anchor_constraint_models;
      std::vector<FrameAnchorConstraintModel> frame_anchor_constraint_models;
      ::pinocchio::mjcf::buildModel(
        path(filename), model, point_anchor_constraint_models, frame_anchor_constraint_models);
      return bp::make_tuple(model, point_anchor_constraint_models, frame_anchor_constraint_models);
    }

    bp::tuple buildModelAndConstraintsFromMJCFAndRootJoint(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      std::vector<PointAnchorConstraintModel> point_anchor_constraint_models;
      std::vector<FrameAnchorConstraintModel> frame_anchor_constraint_models;
      ::pinocchio::mjcf::buildModel(
        path(filename), root_joint, root_joint_name, model, point_anchor_constraint_models,
        frame_anchor_constraint_models);
      return bp::make_tuple(model, point_anchor_constraint_models, frame_anchor_constraint_models);
    }

    bp::tuple buildModelAndLegacyConstraintsFromMJCF(const bp::object & filename)
    {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      ::pinocchio::mjcf::buildModel(path(filename), model, constraint_models);
      return bp::make_tuple(model, constraint_models);
    }

    bp::tuple buildModelAndLegacyConstraintsFromMJCFAndRootJoint(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      ::pinocchio::mjcf::buildModel(
        path(filename), root_joint, root_joint_name, model, constraint_models);
      return bp::make_tuple(model, constraint_models);
    }

    void exposeMJCFModel()
    {
      bp::def(
        "buildModelFromMJCF", pinocchio::python::buildModelFromMJCF, bp::args("mjcf_filename"),
        "Parse the MJCF file given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromMJCF", pinocchio::python::buildModelFromMJCFAndRootJoint,
        bp::args("mjcf_filename", "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint.");

      bp::def(
        "buildModelFromMJCFAndRootJoint", pinocchio::python::buildModelFromMJCFAndRootJoint,
        (bp::args("mjcf_filename"), bp::args("root_joint"),
         bp::args("root_joint_name") = "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint.");

      bp::def(
        "buildModelFromMJCF", pinocchio::python::buildModelFromMJCFAndRootJointDeprecated,
        bp::args("mjcf_filename", "root_joint", "root_joint_name"),
        eigenpy::deprecated_function<>(
          "Deprecated function. Use buildModelAndLegacyConstraintsFromMJCF "
          "instead."),
        "Parse the MJCF file and return a pinocchio Model with "
        "the given root Joint and its "
        "specified name as well as a constraint list if some "
        "are present in the MJCF file.");

      bp::def(
        "buildModelAndConstraintsFromMJCF", pinocchio::python::buildModelAndConstraintsFromMJCF,
        bp::args("mjcf_filename"),
        "Parse the MJCF file given in input and return a pinocchio Model as well as a "
        "PointAnchorConstraintModel and a FrameAnchorConstraintModel list.");

      bp::def(
        "buildModelAndConstraintsFromMJCF",
        pinocchio::python::buildModelAndConstraintsFromMJCFAndRootJoint,
        (bp::args("mjcf_filename"), bp::args("root_joint"),
         bp::args("root_joint_name") = "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with "
        "the given root Joint and its "
        "specified name as well as a PointAnchorConstraintModel and a FrameAnchorConstraintModel "
        "list.");

      bp::def(
        "buildModelAndLegacyConstraintsFromMJCF",
        pinocchio::python::buildModelAndLegacyConstraintsFromMJCF, bp::args("mjcf_filename"),
        "Parse the MJCF file given in input and return a pinocchio Model as well as a "
        "RigidConstraintModel list.");

      bp::def(
        "buildModelAndLegacyConstraintsFromMJCF",
        pinocchio::python::buildModelAndLegacyConstraintsFromMJCFAndRootJoint,
        (bp::args("mjcf_filename"), bp::args("root_joint"),
         bp::args("root_joint_name") = "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with "
        "the given root Joint and its "
        "specified name as well as a RigidConstraintModel "
        "list.");
    }
  } // namespace python
} // namespace pinocchio
