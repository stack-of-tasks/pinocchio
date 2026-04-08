// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/deprecation.hpp"

#include "pinocchio/parsers/sdf.hpp"

#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

// Deprecation messages for buildModelFromSdf overloads.
constexpr char kBuildModelFromSdfDeprecationMsg[] =
  "Deprecated function. Use buildModelAndLegacyConstraintsFromSdf instead.";

void exposeSDFModel(nb::module_ m)
{
  // buildModelAndConstraintsFromSdf (PointAnchorConstraintModel) - no root joint
  m.def(
    "buildModelAndConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const std::string & root_link_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<PointAnchorConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint models.");

  // buildModelAndConstraintsFromSdf (PointAnchorConstraintModel) - with root joint
  m.def(
    "buildModelAndConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<PointAnchorConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint.");

  // buildModelAndConstraintsFromSdf (PointAnchorConstraintModel) - with root joint + name
  m.def(
    "buildModelAndConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::string & root_joint_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<PointAnchorConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, root_joint_name, model, constraint_models, root_link_name,
        parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "root_joint_name"_a,
    "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint and its specified name.");

  // buildModelAndLegacyConstraintsFromSdf (RigidConstraintModel) - no root joint
  m.def(
    "buildModelAndLegacyConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const std::string & root_link_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint models.");

  // buildModelAndLegacyConstraintsFromSdf (RigidConstraintModel) - with root joint
  m.def(
    "buildModelAndLegacyConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint.");

  // buildModelAndLegacyConstraintsFromSdf (RigidConstraintModel) - with root joint + name
  m.def(
    "buildModelAndLegacyConstraintsFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::string & root_joint_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, root_joint_name, model, constraint_models, root_link_name,
        parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "root_joint_name"_a,
    "parent_guidance"_a = nb::list(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint and its specified name.");

  // Deprecated: buildModelFromSdf - no root joint
  m.def(
    "buildModelFromSdf",
    [](
      const std::filesystem::path & filename, const std::string & root_link_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    nb::call_guard<deprecated_guard<kBuildModelFromSdfDeprecationMsg>>(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint models.");

  // Deprecated: buildModelFromSdf - with root joint
  m.def(
    "buildModelFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, model, constraint_models, root_link_name, parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "parent_guidance"_a = nb::list(),
    nb::call_guard<deprecated_guard<kBuildModelFromSdfDeprecationMsg>>(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint.");

  // Deprecated: buildModelFromSdf - with root joint + name
  m.def(
    "buildModelFromSdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_link_name, const std::string & root_joint_name,
      const std::vector<std::string> & parent_guidance) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::sdf::buildModel(
        filename.string(), root_joint, root_joint_name, model, constraint_models, root_link_name,
        parent_guidance);
      return nb::make_tuple(model, constraint_models);
    },
    "sdf_filename"_a, "root_joint"_a, "root_link_name"_a, "root_joint_name"_a,
    "parent_guidance"_a = nb::list(),
    nb::call_guard<deprecated_guard<kBuildModelFromSdfDeprecationMsg>>(),
    "Parse the SDF file given in input and return a pinocchio Model and constraint "
    "models starting with the given root joint and its specified name.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
