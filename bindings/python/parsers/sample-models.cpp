//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/sample-models.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
  
    Model buildSampleModelHumanoidRandom()
    {
      Model model;
      buildModels::humanoidRandom(model);
      return model;
    }

    Model buildSampleModelHumanoidRandom(bool usingFF)
    {
      Model model;
      buildModels::humanoidRandom(model,usingFF);
      return model;
    }

    Model buildSampleModelManipulator()
    {
      Model model;
      buildModels::manipulator(model);
      return model;
    }
      
#ifdef PINOCCHIO_WITH_HPP_FCL
    GeometryModel buildSampleGeometryModelManipulator(const Model & model)
    {
      GeometryModel geom;
      buildModels::manipulatorGeometries(model,geom);
      return geom;
    }
#endif
      
    Model buildSampleModelHumanoid()
    {
      Model model;
      buildModels::humanoid(model);
      return model;
    }

    Model buildSampleModelHumanoid(bool usingFF)
    {
      Model model;
      buildModels::humanoid(model,usingFF);
      return model;
    }
      
#ifdef PINOCCHIO_WITH_HPP_FCL
    GeometryModel buildSampleGeometryModelHumanoid(const Model & model)
    {
      GeometryModel geom;
      buildModels::humanoidGeometries(model,geom);
      return geom;
    }
#endif

    void exposeSampleModels()
    {
      bp::def("buildSampleModelHumanoidRandom",
              static_cast <Model (*) ()> (pinocchio::python::buildSampleModelHumanoidRandom),
              "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.\nOnly meant for unit tests."
              );

      bp::def("buildSampleModelHumanoidRandom",
              static_cast <Model (*) (bool)> (pinocchio::python::buildSampleModelHumanoidRandom),
              bp::args("using_free_flyer"),
              "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.\nOnly meant for unit tests."
              );

      bp::def("buildSampleModelManipulator",
              static_cast <Model (*) ()> (pinocchio::python::buildSampleModelManipulator),
              "Generate a (hard-coded) model of a simple manipulator."
              );

#ifdef PINOCCHIO_WITH_HPP_FCL
      bp::def("buildSampleGeometryModelManipulator",
              static_cast <GeometryModel (*) (const Model&)> (pinocchio::python::buildSampleGeometryModelManipulator),
              bp::args("model"),
              "Generate a (hard-coded) geometry model of a simple manipulator."
              );
#endif
      
      bp::def("buildSampleModelHumanoid",
              static_cast <Model (*) ()> (pinocchio::python::buildSampleModelHumanoid),
              "Generate a (hard-coded) model of a simple humanoid."
              );

      bp::def("buildSampleModelHumanoid",
              static_cast <Model (*) (bool)> (pinocchio::python::buildSampleModelHumanoid),
              bp::args("using_free_flyer"),
              "Generate a (hard-coded) model of a simple humanoid."
              );

#ifdef PINOCCHIO_WITH_HPP_FCL
      bp::def("buildSampleGeometryModelHumanoid",
              static_cast <GeometryModel (*) (const Model&)> (pinocchio::python::buildSampleGeometryModelHumanoid),
              bp::args("model"),
              "Generate a (hard-coded) geometry model of a simple humanoid."
              );
#endif
      
    }
  }
    
} // namespace pinocchio::python
