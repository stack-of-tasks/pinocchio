//
// Copyright (c) 2015-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_sample_models_hpp__
#define __se3_python_sample_models_hpp__

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/bindings/python/multibody/data.hpp"
#include "pinocchio/bindings/python/multibody/geometry-model.hpp"

namespace se3
{
  namespace python
  {
    struct SampleModelsPythonVisitor
    {

      static Model buildSampleModelHumanoidRandom()
      {
        Model model;
        buildModels::humanoidRandom(model);
        return model;
      }

      static Model buildSampleModelHumanoidRandom(bool usingFF)
      {
        Model model;
        buildModels::humanoidRandom(model,usingFF);
        return model;
      }

      static Model buildSampleModelManipulator()
      {
        Model model;
        buildModels::manipulator(model);
        return model;
      }
      
#ifdef WITH_HPP_FCL
      static GeometryModel buildSampleGeometryModelManipulator(const Model& model)
      {
        GeometryModel geom;
        buildModels::manipulatorGeometries(model,geom);
        return geom;
      }
#endif
      
      static Model buildSampleModelHumanoid()
      {
        Model model;
        buildModels::humanoid(model);
        return model;
      }

      static Model buildSampleModelHumanoid(bool usingFF)
      {
        Model model;
        buildModels::humanoid(model,usingFF);
        return model;
      }
      
#ifdef WITH_HPP_FCL
      static GeometryModel buildSampleGeometryModelHumanoid(const Model& model)
      {
        GeometryModel geom;
        buildModels::humanoidGeometries(model,geom);
        return geom;
      }
#endif
      
      /* --- Expose --------------------------------------------------------- */
      static void expose();
    }; // struct SampleModelsPythonVisitor

    inline void SampleModelsPythonVisitor::expose()
    {
      bp::def("buildSampleModelHumanoidRandom",
              static_cast <Model (*) ()> (&SampleModelsPythonVisitor::buildSampleModelHumanoidRandom),
              "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.\nOnly meant for unit tests."
              );

      bp::def("buildSampleModelHumanoidRandom",
              static_cast <Model (*) (bool)> (&SampleModelsPythonVisitor::buildSampleModelHumanoidRandom),
              bp::args("bool (usingFreeFlyer)"),
              "Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.\nOnly meant for unit tests."
              );

      bp::def("buildSampleModelManipulator",
              static_cast <Model (*) ()> (&SampleModelsPythonVisitor::buildSampleModelManipulator),
              "Generate a (hard-coded) model of a simple manipulator."
              );

#ifdef WITH_HPP_FCL
      bp::def("buildSampleGeometryModelManipulator",
              static_cast <GeometryModel (*) (const Model&)> (&SampleModelsPythonVisitor::buildSampleGeometryModelManipulator),
              bp::args("Model (model)"),
              "Generate a (hard-coded) geometry model of a simple manipulator."
              );
#endif
      
      bp::def("buildSampleModelHumanoid",
              static_cast <Model (*) ()> (&SampleModelsPythonVisitor::buildSampleModelHumanoid),
              "Generate a (hard-coded) model of a simple humanoid."
              );

      bp::def("buildSampleModelHumanoid",
              static_cast <Model (*) (bool)> (&SampleModelsPythonVisitor::buildSampleModelHumanoid),
              bp::args("bool (usingFreeFlyer)"),
              "Generate a (hard-coded) model of a simple humanoid."
              );

#ifdef WITH_HPP_FCL
      bp::def("buildSampleGeometryModelHumanoid",
              static_cast <GeometryModel (*) (const Model&)> (&SampleModelsPythonVisitor::buildSampleGeometryModelHumanoid),
              bp::args("Model (model)"),
              "Generate a (hard-coded) geometry model of a simple humanoid."
              );
#endif
      
    }
    
  }
} // namespace se3::python

#endif // ifndef __se3_python_sample_models_hpp__
