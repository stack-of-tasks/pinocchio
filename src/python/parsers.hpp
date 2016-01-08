//
// Copyright (c) 2015 CNRS
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
      struct build_model_visitor : public boost::static_visitor<ModelHandler>
      {
        const std::string& _filename;

        build_model_visitor(const std::string& filename): _filename(filename){}

        template <typename T> ModelHandler operator()( T & operand ) const
        {
          Model * model = new Model();
          *model = se3::urdf::buildModel(_filename, operand);
          return ModelHandler(model,true);
        }
      };

      static ModelHandler buildModelFromUrdfWithRoot( const std::string & filename,
                                      bp::object o
                                      )
      {
        JointModelVariant variant = bp::extract<JointModelVariant> (o);
        return boost::apply_visitor(build_model_visitor(filename), variant);
      }

      static ModelHandler buildModelFromUrdf( const std::string & filename)
      {
	Model * model = new Model();
	*model = se3::urdf::buildModel(filename);
	return ModelHandler(model,true);
      }


#ifdef WITH_HPP_FCL
      struct build_model_and_geom_visitor : public boost::static_visitor<std::pair<ModelHandler, GeometryModelHandler> >
      {
        const std::string& _filenameUrdf;
        const std::string& _filenameMeshRootDir;

        build_model_and_geom_visitor(const std::string& filenameUrdf,
                                     const std::string& filenameMeshRootDir): _filenameUrdf(filenameUrdf)
                                                                            , _filenameMeshRootDir(filenameMeshRootDir)
        {}

        template <typename T> std::pair<ModelHandler, GeometryModelHandler> operator()( T & operand ) const
        {


          Model * model = new Model();
          GeometryModel * geometry_model = new GeometryModel();
          std::pair < Model, GeometryModel > models = se3::urdf::buildModelAndGeom(_filenameUrdf, _filenameMeshRootDir, operand);
          *model = models.first;
          *geometry_model = models.second;
          return std::pair<ModelHandler, GeometryModelHandler> ( ModelHandler(model, true),
                                                                 GeometryModelHandler(geometry_model, true)
                                                               );
        }
      };

      static std::pair<ModelHandler, GeometryModelHandler> buildModelAndGeomFromUrdfWithRoot( const std::string & filenameUrdf,
                                                      const std::string & filenameMeshRootDir,
                                                      bp::object o
                                                      )
      {
        JointModelVariant variant = bp::extract<JointModelVariant> (o);
        return boost::apply_visitor(build_model_and_geom_visitor(filenameUrdf, filenameMeshRootDir), variant);
      }

      static std::pair<ModelHandler, GeometryModelHandler> buildModelAndGeomFromUrdf( const std::string & filenameUrdf,
                                                                                      const std::string & filenameMeshRootDir)
      {
        Model * model = new Model();
        GeometryModel * geometry_model = new GeometryModel();
        std::pair < Model, GeometryModel > models = se3::urdf::buildModelAndGeom(filenameUrdf, filenameMeshRootDir);
        *model = models.first;
        *geometry_model = models.second;
        return std::pair<ModelHandler, GeometryModelHandler> ( ModelHandler(model, true),
                                                               GeometryModelHandler(geometry_model, true)
                                                             );
      }
#endif

#endif

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
      static void expose()
      {
        bp::def("buildModelFromUrdfWithRoot",buildModelFromUrdfWithRoot,
          bp::args("Filename (string)",
              "Root Joint Model"),
          "Parse the urdf file given in input and return a proper pinocchio model "
          "(remember to create the corresponding data structure).");

#ifdef WITH_URDFDOM
	bp::def("buildModelFromUrdf",buildModelFromUrdf,
		bp::args("Filename (string)"),
		"Parse the urdf file given in input and return a proper pinocchio model "
		"(remember to create the corresponding data structure).");
  #ifdef WITH_HPP_FCL

      bp::to_python_converter<std::pair<ModelHandler, GeometryModelHandler>, PairToTupleConverter<ModelHandler, GeometryModelHandler> >();

      bp::def("buildModelAndGeomFromUrdfWithRoot",buildModelAndGeomFromUrdfWithRoot,
          bp::args("FilenameUrdf (string)", "FilenameMeshRootDirhRootDir string) ", 
              "Root Joint Model"),
          "Parse the urdf file given in input and return a proper pinocchio model starting with a given root joint and geometry model "
          "(remember to create the corresponding data structures).");

      bp::def("buildModelAndGeomFromUrdf",buildModelAndGeomFromUrdf,
          bp::args("Filename (string)"), "FilenameMeshRootDirhRootDir string) ",
          "Parse the urdf file given in input and return a proper pinocchio model and geometry model"
          "(remember to create the corresponding data structures).");
  #endif
#endif

#ifdef WITH_LUA
        bp::def("buildModelFromLua",buildModelFromLua,
                bp::args("Filename (string)",
                         "Free flyer (bool, false for a fixed robot)",
                         "Verbose option "),
                "Parse the urdf file given in input and return a proper pinocchio model "
                "(remember to create the corresponding data structure).");
#endif // #ifdef WITH_LUA
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_parsers_hpp__

