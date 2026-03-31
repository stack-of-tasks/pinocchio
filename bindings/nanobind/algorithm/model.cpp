// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/model.hpp"

#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeModelAlgo(nb::module_ m)
{
  using JointIndex = Model::JointIndex;

  m.def(
    "appendModel",
    [](const Model & modelA, const Model & modelB, FrameIndex frame_in_modelA, const SE3 & aMb)
      -> Model { return appendModel(modelA, modelB, frame_in_modelA, aMb); },
    "modelA"_a, "modelB"_a, "frame_in_modelA"_a, "aMb"_a,
    "Append a child model into a parent model, after a specific frame given by its index.\n\n"
    "Parameters:\n"
    "\tmodelA: the parent model\n"
    "\tmodelB: the child model\n"
    "\tframe_in_modelA: index of the frame of modelA where to append modelB\n"
    "\taMb: pose of modelB universe joint (index 0) in frame_in_modelA");

  m.def(
    "appendModel",
    [](
      const Model & modelA, const Model & modelB, const GeometryModel & geomModelA,
      const GeometryModel & geomModelB, FrameIndex frame_in_modelA, const SE3 & aMb) -> nb::tuple {
      Model model;
      GeometryModel geom_model;
      appendModel(modelA, modelB, geomModelA, geomModelB, frame_in_modelA, aMb, model, geom_model);
      return nb::make_tuple(std::move(model), std::move(geom_model));
    },
    "modelA"_a, "modelB"_a, "geomModelA"_a, "geomModelB"_a, "frame_in_modelA"_a, "aMb"_a,
    "Append a child (geometry) model into a parent (geometry) model, after a specific frame "
    "given by its index.\n\n"
    "Parameters:\n"
    "\tmodelA: the parent model\n"
    "\tmodelB: the child model\n"
    "\tgeomModelA: the parent geometry model\n"
    "\tgeomModelB: the child geometry model\n"
    "\tframe_in_modelA: index of the frame of modelA where to append modelB\n"
    "\taMb: pose of modelB universe joint (index 0) in frame_in_modelA");

  m.def(
    "buildReducedModel",
    [](
      const Model & model, const std::vector<JointIndex> & list_of_joints_to_lock,
      ConstVectorRef reference_configuration) -> Model {
      return buildReducedModel(model, list_of_joints_to_lock, reference_configuration);
    },
    "model"_a, "list_of_joints_to_lock"_a, "reference_configuration"_a,
    "Build a reduced model from a given input model and a list of joints to lock.\n\n"
    "Parameters:\n"
    "\tmodel: input kinematic model to reduce\n"
    "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
    "\treference_configuration: reference configuration to compute the placement of the locked "
    "joints");

  m.def(
    "buildReducedModel",
    [](
      const Model & model, const GeometryModel & geom_model,
      const std::vector<JointIndex> & list_of_joints_to_lock,
      ConstVectorRef reference_configuration) -> nb::tuple {
      Model reduced_model;
      GeometryModel reduced_geom_model;
      buildReducedModel(
        model, geom_model, list_of_joints_to_lock, reference_configuration, reduced_model,
        reduced_geom_model);
      return nb::make_tuple(std::move(reduced_model), std::move(reduced_geom_model));
    },
    "model"_a, "geom_model"_a, "list_of_joints_to_lock"_a, "reference_configuration"_a,
    "Build a reduced model and a reduced geometry model from a given input model, "
    "an input geometry model and a list of joints to lock.\n\n"
    "Parameters:\n"
    "\tmodel: input kinematic model to reduce\n"
    "\tgeom_model: input geometry model to reduce\n"
    "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
    "\treference_configuration: reference configuration to compute the placement of the locked "
    "joints");

  m.def(
    "buildReducedModel",
    [](
      const Model & model, const std::vector<GeometryModel> & list_of_geom_models,
      const std::vector<JointIndex> & list_of_joints_to_lock,
      ConstVectorRef reference_configuration) -> nb::tuple {
      Model reduced_model;
      std::vector<GeometryModel> reduced_geom_models;
      buildReducedModel(
        model, list_of_geom_models, list_of_joints_to_lock, reference_configuration, reduced_model,
        reduced_geom_models);
      return nb::make_tuple(std::move(reduced_model), std::move(reduced_geom_models));
    },
    "model"_a, "list_of_geom_models"_a, "list_of_joints_to_lock"_a, "reference_configuration"_a,
    "Build a reduced model and the related reduced geometry models from a given input model, "
    "a list of input geometry models and a list of joints to lock.\n\n"
    "Parameters:\n"
    "\tmodel: input kinematic model to reduce\n"
    "\tlist_of_geom_models: input geometry models to reduce\n"
    "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
    "\treference_configuration: reference configuration to compute the placement of the locked "
    "joints");

  m.def(
    "findCommonAncestor",
    [](const Model & model, JointIndex joint1_id, JointIndex joint2_id) -> nb::tuple {
      size_t index_ancestor_in_support1, index_ancestor_in_support2;
      JointIndex ancestor_id = findCommonAncestor(
        model, joint1_id, joint2_id, index_ancestor_in_support1, index_ancestor_in_support2);
      return nb::make_tuple(ancestor_id, index_ancestor_in_support1, index_ancestor_in_support2);
    },
    "model"_a, "joint1_id"_a, "joint2_id"_a,
    "Computes the common ancestor between two joints belonging to the same kinematic tree.\n\n"
    "Parameters:\n"
    "\tmodel: input model\n"
    "\tjoint1_id: index of the first joint\n"
    "\tjoint2_id: index of the second joint\n"
    "Returns a tuple containing the index of the common joint ancestor, the position of this "
    "ancestor in model.supports[joint1_id] and model.supports[joint2_id].");

  m.def(
    "transformJointIntoMimic",
    [](
      const Model & input_model, JointIndex index_mimicked, JointIndex index_mimicking,
      Scalar scaling, Scalar offset) -> Model {
      Model model;
      transformJointIntoMimic(input_model, index_mimicked, index_mimicking, scaling, offset, model);
      return model;
    },
    "input_model"_a, "index_mimicked"_a, "index_mimicking"_a, "scaling"_a, "offset"_a,
    "Transform a joint of the model into a mimic joint. Keep the type of the joint as it "
    "was previously.\n\n"
    "Parameters:\n"
    "\tinput_model: the input model to take joints from\n"
    "\tindex_mimicked: index of the joint to mimic\n"
    "\tindex_mimicking: index of the joint that will mimic\n"
    "\tscaling: scaling of joint velocity and configuration\n"
    "\toffset: offset of joint configuration");
}

PINOCCHIO_PYTHON_NAMESPACE_END
