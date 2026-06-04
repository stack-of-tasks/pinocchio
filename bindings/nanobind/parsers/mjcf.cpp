// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/deprecation.hpp"

#include "pinocchio/parsers/mjcf.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h> // for MeshLoaderPtr

#ifdef PINOCCHIO_WITH_COLLISION
  #include <coal/mesh_loader/loader.h>
#endif

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeMJCF(nb::module_ m)
{
  // buildModelFromMJCF - no root joint
  m.def(
    "buildModelFromMJCF",
    [](const std::filesystem::path & filename) {
      Model model;
      pinocchio::mjcf::buildModel(filename, model);
      return model;
    },
    "mjcf_filename"_a, "Parse the MJCF file given in input and return a pinocchio Model.");

  // buildModelFromMJCF - with root joint (name defaults to "root_joint")
  m.def(
    "buildModelFromMJCF",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name) {
      Model model;
      pinocchio::mjcf::buildModel(filename, root_joint, root_joint_name, model);
      return model;
    },
    "mjcf_filename"_a, "root_joint"_a, "root_joint_name"_a = nb::str("root_joint"),
    "Parse the MJCF file and return a pinocchio Model with the given root joint.");

  // buildModelFromMJCFAndRootJointDeprecated
  m.def(
    "buildModelFromMJCF",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name) -> nb::tuple {
      static constexpr char msg[] =
        "Deprecated function. Use buildModelAndLegacyConstraintsFromMJCF instead.";
      deprecated_guard<msg> guard;
      Model model;
      std::vector<RigidConstraintModel> contact_models;
      pinocchio::mjcf::buildModel(
        filename.string(), root_joint, root_joint_name, model, contact_models);
      return nb::make_tuple(model, contact_models);
    },
    "mjcf_filename"_a, "root_joint"_a, "root_joint_name"_a = nb::str("root_joint"),
    "Parse the MJCF file and return a pinocchio Model with the given root Joint and its specified "
    "name as well as a constraint list if some are present in the MJCF file.");

  // buildModelFromMJCFAndRootJoint - explicit name (alias for backwards comp)
  m.def(
    "buildModelFromMJCFAndRootJoint",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name) {
      Model model;
      pinocchio::mjcf::buildModel(filename, root_joint, root_joint_name, model);
      return model;
    },
    "mjcf_filename"_a, "root_joint"_a, "root_joint_name"_a = nb::str("root_joint"),
    "Parse the MJCF file and return a pinocchio Model with the given root joint.");

  // buildModelAndConstraintsFromMJCF - no root joint
  m.def(
    "buildModelAndConstraintsFromMJCF",
    [](const std::filesystem::path & filename) {
      Model model;
      std::vector<PointAnchorConstraintModel> point_anchor_constraint_models;
      std::vector<FrameAnchorConstraintModel> frame_anchor_constraint_models;
      pinocchio::mjcf::buildModel(
        filename, model, point_anchor_constraint_models, frame_anchor_constraint_models);
      return nb::make_tuple(model, point_anchor_constraint_models, frame_anchor_constraint_models);
    },
    "mjcf_filename"_a,
    "Parse the MJCF file given in input and return a pinocchio Model as well as a "
    "PointAnchorConstraintModel and a FrameAnchorConstraintModel list.");

  // buildModelAndConstraintsFromMJCF - with root joint
  m.def(
    "buildModelAndConstraintsFromMJCF",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name) {
      Model model;
      std::vector<PointAnchorConstraintModel> point_anchor_constraint_models;
      std::vector<FrameAnchorConstraintModel> frame_anchor_constraint_models;
      pinocchio::mjcf::buildModel(
        filename, root_joint, root_joint_name, model, point_anchor_constraint_models,
        frame_anchor_constraint_models);
      return nb::make_tuple(model, point_anchor_constraint_models, frame_anchor_constraint_models);
    },
    "mjcf_filename"_a, "root_joint"_a, "root_joint_name"_a = nb::str("root_joint"),
    "Parse the MJCF file and return a pinocchio Model with the given root joint as well as a "
    "PointAnchorConstraintModel and a FrameAnchorConstraintModel list.");

  // buildModelAndLegacyConstraintsFromMJCF - no root joint
  m.def(
    "buildModelAndLegacyConstraintsFromMJCF",
    [](const std::filesystem::path & filename) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::mjcf::buildModel(filename, model, constraint_models);
      return nb::make_tuple(model, constraint_models);
    },
    "mjcf_filename"_a,
    "Parse the MJCF file given in input and return a pinocchio Model as well as a "
    "RigidConstraintModel list.");

  // buildModelAndLegacyConstraintsFromMJCF - with root joint
  m.def(
    "buildModelAndLegacyConstraintsFromMJCF",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name) {
      Model model;
      std::vector<RigidConstraintModel> constraint_models;
      pinocchio::mjcf::buildModel(filename, root_joint, root_joint_name, model, constraint_models);
      return nb::make_tuple(model, constraint_models);
    },
    "mjcf_filename"_a, "root_joint"_a, "root_joint_name"_a = nb::str("root_joint"),
    "Parse the MJCF file and return a pinocchio Model with the given root joint as well as a "
    "RigidConstraintModel list.");

  // buildGeomFromMJCF - no mesh loader
  m.def(
    "buildGeomFromMJCF",
    [](Model & model, const std::filesystem::path & filename, const GeometryType & geom_type) {
      GeometryModel geometry_model;
      pinocchio::mjcf::buildGeom(model, filename, geom_type, geometry_model);
      return geometry_model;
    },
    "model"_a, "mjcf_filename"_a, "geom_type"_a,
    "Parse the MJCF file given as input looking for the geometry of the given input model and\n"
    "return a GeometryModel containing either the collision geometries "
    "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tmjcf_filename: path to the MJCF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the MJCF file (either the VISUAL for "
    "display or the COLLISION for collision detection).");

#ifdef PINOCCHIO_WITH_COLLISION
  // buildGeomFromMJCF - with mesh loader
  m.def(
    "buildGeomFromMJCF",
    [](
      Model & model, const std::filesystem::path & filename, const GeometryType & geom_type,
      ::coal::MeshLoaderPtr mesh_loader) {
      GeometryModel geometry_model;
      pinocchio::mjcf::buildGeom(model, filename, geom_type, geometry_model, mesh_loader);
      return geometry_model;
    },
    "model"_a, "mjcf_filename"_a, "geom_type"_a, "mesh_loader"_a,
    "Parse the MJCF file given as input looking for the geometry of the given input model and\n"
    "return a GeometryModel containing either the collision geometries "
    "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tmjcf_filename: path to the MJCF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the MJCF file (either the VISUAL for "
    "display or the COLLISION for collision detection).\n"
    "\tmesh_loader: a coal mesh loader (to load only once the related geometries).");
#endif
}

PINOCCHIO_PYTHON_NAMESPACE_END
