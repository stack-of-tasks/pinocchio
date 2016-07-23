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
  #include "pinocchio/parsers/urdf.hpp"
#ifdef WITH_HPP_FCL
  #include "pinocchio/python/geometry-model.hpp"
  #include "pinocchio/python/geometry-data.hpp"
#endif
#endif

#ifdef WITH_LUA
  #include "pinocchio/parsers/lua.hpp"
#endif // #ifdef WITH_LUA

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
                        const GeometryType type
                        )
      {
        std::vector<std::string> hints;
        GeometryModel * geometry_model = new GeometryModel(se3::urdf::buildGeom(*model, filename,hints, type));
        
        return GeometryModelHandler(geometry_model, true);
      }

      static GeometryModelHandler
      buildGeomFromUrdf(const ModelHandler & model,
                        const std::string & filename,
                        std::vector<std::string> & package_dirs,
                        const GeometryType type
                        )
      {
        GeometryModel * geometry_model = new GeometryModel(se3::urdf::buildGeom(*model, filename, package_dirs, type));
        
        return GeometryModelHandler(geometry_model, true);
      }
      
      static void removeCollisionPairsFromSrdf(ModelHandler & model,
                                               GeometryModelHandler& geometry_model,
                                               GeometryDataHandler & geometry_data,
                                               const std::string & filename,
                                               bool verbose
                                               )
      {
        se3::srdf::removeCollisionPairsFromSrdf(*model, *geometry_model ,*geometry_data, filename, verbose);
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

      static Eigen::VectorXd getNeutralConfigurationFromSrdf(ModelHandler & model,
                                                             const std::string & filename,
                                                             bool verbose
                                                            )
      {
        return se3::srdf::getNeutralConfigurationFromSrdf(*model, filename, verbose);
      }

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
              static_cast <GeometryModelHandler (*) (const ModelHandler &, const std::string &, std::vector<std::string> &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)", "package_dirs (vector of strings)"
                       ),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio geometry model "
              "(remember to create the corresponding data structures).");
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModelHandler (*) (const ModelHandler &, const std::string &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","filename (string)"),
              "Parse the urdf file given in input looking for the geometry of the given Model and return a proper pinocchio  geometry model "
              "(remember to create the corresponding data structures).");
      
      bp::def("removeCollisionPairsFromSrdf",removeCollisionPairsFromSrdf,
              bp::args("Model associated to GeometryData", "GeometryModel associated to a GeometryData", "GeometryData for which we want to remove pairs of collision", "srdf filename (string)", "verbosity"
                       ),
              "Parse an srdf file in order to desactivate collision pairs for a specific GeometryData and GeometryModel ");

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

      bp::def("getNeutralConfigurationFromSrdf",getNeutralConfigurationFromSrdf,
              // static_cast <ModelHandler (*) ( const std::string &, bool)> (&ParsersPythonVisitor::getNeutralConfigurationFromSrdf),
              bp::args("Model for which we want the neutral config","srdf filename (string)", "verbosity"
                       ),
              "Get the neutral configuration of a given model associated to a SRDF file");

    }
    
  }
} // namespace se3::python

#endif // ifndef __se3_python_parsers_hpp__
