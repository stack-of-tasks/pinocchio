// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/multibody/sample-models.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeSampleModels(nb::module_ m)
{
  m.def(
    "buildSampleModelHumanoidRandom",
    [](bool usingFF, bool mimic) {
      Model model;
      buildModels::humanoidRandom(model, usingFF, mimic);
      return model;
    },
    "using_free_flyer"_a = true, "mimic"_a = false,
    "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint "
    "placements.\nOnly meant for unit tests.");

  m.def(
    "buildSampleModelManipulator",
    [](bool mimic) {
      Model model;
      buildModels::manipulator(model, mimic);
      return model;
    },
    "mimic"_a = false, "Generate a (hard-coded) model of a simple manipulator.");

#ifdef PINOCCHIO_WITH_COLLISION
  m.def(
    "buildSampleGeometryModelManipulator",
    [](const Model & model) {
      GeometryModel geom;
      buildModels::manipulatorGeometries(model, geom);
      return geom;
    },
    "model"_a, "Generate a (hard-coded) geometry model of a simple manipulator.");
#endif

  m.def(
    "buildSampleModelHumanoid",
    [](bool usingFF) {
      Model model;
      buildModels::humanoid(model, usingFF);
      return model;
    },
    "using_free_flyer"_a = true, "Generate a (hard-coded) model of a simple humanoid.");

#ifdef PINOCCHIO_WITH_COLLISION
  m.def(
    "buildSampleGeometryModelHumanoid",
    [](const Model & model) {
      GeometryModel geom;
      buildModels::humanoidGeometries(model, geom);
      return geom;
    },
    "model"_a, "Generate a (hard-coded) geometry model of a simple humanoid.");
#endif
}
PINOCCHIO_PYTHON_NAMESPACE_END
