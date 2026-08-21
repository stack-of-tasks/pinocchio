//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    context::Model buildSampleModelHumanoidRandom(bool usingFF, bool mimic)
    {
      context::Model model;
      buildModels::humanoidRandom(model, usingFF, mimic);
      return model;
    }

    context::Model buildSampleModelManipulator(bool mimic)
    {
      context::Model model;
      buildModels::manipulator(model, mimic);
      return model;
    }

#ifdef PINOCCHIO_WITH_COLLISION
    GeometryModel buildSampleGeometryModelManipulator(const context::Model & model)
    {
      GeometryModel geom;
      buildModels::manipulatorGeometries(model, geom);
      return geom;
    }
#endif

    context::Model buildSampleModelHumanoid(bool usingFF)
    {
      context::Model model;
      buildModels::humanoid(model, usingFF);
      return model;
    }

#ifdef PINOCCHIO_WITH_COLLISION
    GeometryModel buildSampleGeometryModelHumanoid(const context::Model & model)
    {
      GeometryModel geom;
      buildModels::humanoidGeometries(model, geom);
      return geom;
    }
#endif

    void exposeSampleModels()
    {
      bp::def(
        "buildSampleModelHumanoidRandom",
        static_cast<context::Model (*)(bool, bool)>(
          pinocchio::python::buildSampleModelHumanoidRandom),
        (bp::arg("using_free_flyer") = true, bp::arg("mimic") = false),
        "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint "
        "placements.\nOnly meant for unit tests.");

      bp::def(
        "buildSampleModelManipulator",
        static_cast<context::Model (*)(bool)>(pinocchio::python::buildSampleModelManipulator),
        (bp::arg("mimic") = false), "Generate a (hard-coded) model of a simple manipulator.");

#ifdef PINOCCHIO_WITH_COLLISION
      bp::def(
        "buildSampleGeometryModelManipulator",
        static_cast<GeometryModel (*)(const context::Model &)>(
          pinocchio::python::buildSampleGeometryModelManipulator),
        bp::args("model"), "Generate a (hard-coded) geometry model of a simple manipulator.");
#endif

      bp::def(
        "buildSampleModelHumanoid",
        static_cast<context::Model (*)(bool)>(pinocchio::python::buildSampleModelHumanoid),
        (bp::arg("using_free_flyer") = true),
        "Generate a (hard-coded) model of a simple humanoid.");

#ifdef PINOCCHIO_WITH_COLLISION
      bp::def(
        "buildSampleGeometryModelHumanoid",
        static_cast<GeometryModel (*)(const context::Model &)>(
          pinocchio::python::buildSampleGeometryModelHumanoid),
        bp::args("model"), "Generate a (hard-coded) geometry model of a simple humanoid.");
#endif
    }
  } // namespace python

} // namespace pinocchio
