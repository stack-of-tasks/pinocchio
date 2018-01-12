      //
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_python_parsers_hpp__
#define __se3_python_parsers_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/bindings/python/multibody/data.hpp"

#ifdef WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

  #include "pinocchio/bindings/python/multibody/geometry-model.hpp"
  #include "pinocchio/bindings/python/multibody/geometry-data.hpp"

#ifdef WITH_LUA5
  #include "pinocchio/parsers/lua.hpp"
#endif // #ifdef WITH_LUA5

#include "pinocchio/parsers/srdf.hpp"

namespace se3
{
  namespace python
  {
    struct ParsersPythonVisitor
    {

      template<class T1, class T2>
      struct PairToTupleConverter {
        static PyObject* convert(const std::pair<T1, T2>& pair) {
          return boost::python::incref(boost::python::make_tuple(pair.first, pair.second).ptr());
        }
      };
      
      
#ifdef WITH_URDFDOM

      static Model buildModelFromUrdf(const std::string & filename,
                                      bp::object & root_joint_object
                                      )
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        Model model;
        se3::urdf::buildModel(filename, root_joint, model);
        return model;
      }

      static Model buildModelFromUrdf(const std::string & filename)
      {
        Model model;
        se3::urdf::buildModel(filename, model);
        return model;
      }


      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type
                        )
      {
        std::vector<std::string> hints;
        GeometryModel geometry_model;
        se3::urdf::buildGeom(model,filename,type,geometry_model,hints);
        
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const std::vector<std::string> & package_dirs,
                        const GeometryType type
                        )
      {
        GeometryModel geometry_model;
        se3::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
        
        return geometry_model;
      }
      
#ifdef WITH_HPP_FCL
      static void removeCollisionPairsFromSrdf(Model & model,
                                               GeometryModel & geometry_model,
                                               const std::string & filename,
                                               bool verbose
                                               )
      {
        se3::srdf::removeCollisionPairsFromSrdf(model,geometry_model,filename,verbose);
      }

#endif // #ifdef WITH_HPP_FCL
#endif // #ifdef WITH_URDFDOM

#ifdef WITH_LUA5
      static Model buildModelFromLua(const std::string & filename,
                                            bool ff,
                                            bool verbose
                                            )
      {
        Model model;
        model = se3::lua::buildModel (filename, ff, verbose);
        return model;
      }
#endif // #ifdef WITH_LUA5

      static Eigen::VectorXd getNeutralConfigurationFromSrdf(Model & model,
                                                             const std::string & filename,
                                                             bool verbose
                                                            )
      {
        return se3::srdf::getNeutralConfigurationFromSrdf(model, filename, verbose);
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose();
    }; // struct ParsersPythonVisitor

    inline void ParsersPythonVisitor::expose()
    {
#ifdef WITH_URDFDOM
      
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)","Root Joint Model"),
              "Parse the urdf file given in input and return a pinocchio model starting with the given root joint model"
              "(remember to create the corresponding data structure)."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)"),
              "Parse the urdf file given in input and return a pinocchio model"
              "(remember to create the corresponding data structure)."
              );
      
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const std::vector<std::string> &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)", "package_dirs (vector of strings)"
                       ),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio geometry model "
              "(remember to create the corresponding data structures).");
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)"),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio  geometry model "
              "(remember to create the corresponding data structures).");
      
#ifdef WITH_HPP_FCL
      bp::def("removeCollisionPairsFromSrdf",removeCollisionPairsFromSrdf,
              bp::args("Model", "GeometryModel (where pairs are removed)","srdf filename (string)", "verbosity"
                       ),
              "Parse an srdf file in order to desactivate collision pairs for a specific GeometryData and GeometryModel ");

#endif // #ifdef WITH_HPP_FCL
#endif // #ifdef WITH_URDFDOM
      
#ifdef WITH_LUA5
      bp::def("buildModelFromLua",buildModelFromLua,
              bp::args("Filename (string)",
                       "Free flyer (bool, false for a fixed robot)",
                       "Verbose option "),
              "Parse the urdf file given in input and return a proper pinocchio model "
              "(remember to create the corresponding data structure).");
#endif // #ifdef WITH_LUA5

      bp::def("getNeutralConfigurationFromSrdf",getNeutralConfigurationFromSrdf,
              // static_cast <ModelHandler (*) ( const std::string &, bool)> (&ParsersPythonVisitor::getNeutralConfigurationFromSrdf),
              bp::args("Model for which we want the neutral config","srdf filename (string)", "verbosity"
                       ),
              "Get the neutral configuration of a given model associated to a SRDF file");

    }
    
  }
} // namespace se3::python

#endif // ifndef __se3_python_parsers_hpp__
