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

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#ifdef WITH_URDFDOM
  #include "pinocchio/multibody/parser/urdf.hpp"
#ifdef WITH_HPP_FCL
  #include "pinocchio/python/geometry-model.hpp"
  #include "pinocchio/python/geometry-data.hpp"
  #include "pinocchio/multibody/parser/urdf-with-geometry.hpp"
#endif
#endif

#ifdef WITH_LUA
  #include "pinocchio/multibody/parser/lua.hpp"
#endif // #ifdef WITH_LUA

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
      struct BuildModelVisitor : public boost::static_visitor<ModelHandler>
      {
        const std::string& _filename;

        BuildModelVisitor(const std::string& filename): _filename(filename){}

        template <typename JointModel> ModelHandler operator()(const JointModel & root_joint) const
        {
          Model * model = new Model();
          *model = se3::urdf::buildModel(_filename, root_joint);
          return ModelHandler(model,true);
        }
      };

      static ModelHandler buildModelFromUrdf(const std::string & filename,
                                             bp::object & root_joint_object
                                             )
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object);
        return boost::apply_visitor(BuildModelVisitor(filename), root_joint);
      }

      static ModelHandler buildModelFromUrdf(const std::string & filename)
      {
        Model * model = new Model();
        *model = se3::urdf::buildModel(filename);
        return ModelHandler(model,true);
      }


#ifdef WITH_HPP_FCL
      typedef std::pair<ModelHandler, GeometryModelHandler> ModelGeometryHandlerPair_t;
      
      static GeometryModelHandler
      buildGeomFromUrdf(const ModelHandler & model,
                        const std::string & filename,
                        const bool root_added
                        )
      {
        GeometryModel * geometry_model = new GeometryModel(se3::urdf::buildGeom(*model, filename, root_added));
        
        return GeometryModelHandler(geometry_model, true);
      }

      static GeometryModelHandler
      buildGeomFromUrdf(const ModelHandler & model,
                        const std::string & filename,
                        std::vector<std::string> & package_dirs,
                        const bool root_added
                        )
      {
        GeometryModel * geometry_model = new GeometryModel(se3::urdf::buildGeom(*model, filename, package_dirs, root_added));
        
        return GeometryModelHandler(geometry_model, true);
      }
      
#endif // #ifdef WITH_HPP_FCL
#endif // #ifdef WITH_URDFDOM

#ifdef WITH_LUA
      static ModelHandler buildModelFromLua(const std::string & filename,
                                            bool ff,
                                            bool verbose
                                            )
      {
        Model * model = new Model ();
        *model = se3::lua::buildModel (filename, ff, verbose);
        return ModelHandler (model,true);
      }
#endif // #ifdef WITH_LUA

      /* --- Expose --------------------------------------------------------- */
      static void expose();
    }; // struct ParsersPythonVisitor

    inline void ParsersPythonVisitor::expose()
    {
#ifdef WITH_URDFDOM
      
      
      bp::def("buildModelFromUrdf",
              static_cast <ModelHandler (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)","Root Joint Model"),
              "Parse the urdf file given in input and return a pinocchio model starting with the given root joint model"
              "(remember to create the corresponding data structure)."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <ModelHandler (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)"),
              "Parse the urdf file given in input and return a pinocchio model"
              "(remember to create the corresponding data structure)."
              );
      
#ifdef WITH_HPP_FCL
      
      bp::to_python_converter<std::pair<ModelHandler, GeometryModelHandler>, PairToTupleConverter<ModelHandler, GeometryModelHandler> >();
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModelHandler (*) (const ModelHandler &, const std::string &, std::vector<std::string> &, const bool)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)", "package_dirs (vector of strings)",
                       "bool stating if we added a custom root joint to the Model"),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio geometry model "
              "(remember to create the corresponding data structures).");
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModelHandler (*) (const ModelHandler &, const std::string &, const bool)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)", "bool stating if we added a custom root joint to the Model"),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio  geometry model "
              "(remember to create the corresponding data structures).");
      
#endif // #ifdef WITH_HPP_FCL
#endif // #ifdef WITH_URDFDOM
      
#ifdef WITH_LUA
      bp::def("buildModelFromLua",buildModelFromLua,
              bp::args("Filename (string)",
                       "Free flyer (bool, false for a fixed robot)",
                       "Verbose option "),
              "Parse the urdf file given in input and return a proper pinocchio model "
              "(remember to create the corresponding data structure).");
#endif // #ifdef WITH_LUA
    }
    
  }
} // namespace se3::python

#endif // ifndef __se3_python_parsers_hpp__
