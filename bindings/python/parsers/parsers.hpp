//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_python_parsers_hpp__
#define __pinocchio_python_parsers_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/bindings/python/multibody/data.hpp"

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

  #include "pinocchio/bindings/python/multibody/geometry-model.hpp"
  #include "pinocchio/bindings/python/multibody/geometry-data.hpp"

#ifdef PINOCCHIO_WITH_LUA5
  #include "pinocchio/parsers/lua.hpp"
#endif // #ifdef PINOCCHIO_WITH_LUA5

#include "pinocchio/parsers/srdf.hpp"

namespace pinocchio
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
      
      
#ifdef PINOCCHIO_WITH_URDFDOM
      
      static Model buildModelFromUrdf(const std::string & filename)
      {
        Model model;
        pinocchio::urdf::buildModel(filename, model);
        return model;
      }
      
      static void buildModelFromUrdf(const std::string & filename,
                                     Model & model)
      {
        pinocchio::urdf::buildModel(filename, model);
      }

      static Model buildModelFromUrdf(const std::string & filename,
                                      bp::object & root_joint_object
                                      )
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        Model model;
        pinocchio::urdf::buildModel(filename, root_joint, model);
        return model;
      }
      
      static void buildModelFromUrdf(const std::string & filename,
                                     bp::object & root_joint_object,
                                     Model & model
                                     )
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        pinocchio::urdf::buildModel(filename, root_joint, model);
      }
      
      static Model buildModelFromXML(const std::string & XMLstream,
                                     bp::object & root_joint_object
                                     )
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        Model model;
        pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
        return model;
      }
      
      static Model buildModelFromXML(const std::string & XMLstream)
      {
        Model model;
        pinocchio::urdf::buildModelFromXML(XMLstream, model);
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
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,hints);
        
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
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
        
        return geometry_model;
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const fcl::MeshLoaderPtr& meshLoader
                        )
      {
        std::vector<std::string> hints;
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,hints,meshLoader);
        
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const std::vector<std::string> & package_dirs,
                        const GeometryType type,
                        const fcl::MeshLoaderPtr& meshLoader
                        )
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs,meshLoader);
        
        return geometry_model;
      }

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL

#endif // #ifdef PINOCCHIO_WITH_URDFDOM

#ifdef PINOCCHIO_WITH_LUA5
      static Model buildModelFromLua(const std::string & filename,
                                            bool ff,
                                            bool verbose
                                            )
      {
        Model model;
        model = pinocchio::lua::buildModel (filename, ff, verbose);
        return model;
      }
#endif // #ifdef PINOCCHIO_WITH_LUA5
      
      /* --- Expose --------------------------------------------------------- */
      static void expose();
    }; // struct ParsersPythonVisitor

    inline void ParsersPythonVisitor::expose()
    {
#ifdef PINOCCHIO_WITH_URDFDOM
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)","Root Joint Model"),
              "Parse the URDF file given in input and return a pinocchio model starting with the given root joint model"
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("Filename (string)"),
              "Parse the URDF file given in input and return a pinocchio model"
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <void (*) (const std::string &, Model &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("URDF filename (string)","model (class Model)"),
              "Append to a given model a URDF structure given by its filename"
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <void (*) (const std::string &, bp::object &, Model &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("URDF filename (string)","model (class Model)","root_joint (class JointModel)"),
              "Append to a given model a URDF structure given by its filename and the root joint"
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("XML stream (string)","Root Joint Model"),
              "Parse the URDF XML stream given in input and return a pinocchio model starting with the given root joint model"
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("XML stream (string)"),
              "Parse the URDF XML stream given in input and return a pinocchio model"
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const std::vector<std::string> &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","URDF filename (string)", "package_dirs (vector of strings)", "Geometry type (COLLISION or VISUAL)"),
              "Parse the URDF file given in input looking for the geometry of the given Model and return a proper pinocchio geometry model ");
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","URDF filename (string)","Geometry type (COLLISION or VISUAL)"),
              "Parse the URDF file given in input looking for the geometry of the given Model and return a proper pinocchio  geometry model ");

#ifdef PINOCCHIO_WITH_HPP_FCL

      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const std::vector<std::string> &, const GeometryType, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","URDF filename (string)", "package_dirs (vector of strings)","Geometry type (COLLISION or VISUAL)", "Mesh loader"),
              "Parse the URDF file given in input looking for the geometry of the given Model and return a proper pinocchio geometry model ");
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("Model to assosiate the Geometry","URDF filename (string)","Geometry type (COLLISION or VISUAL)", "Mesh loader"),
              "Parse the URDF file given in input looking for the geometry of the given Model and return a proper pinocchio  geometry model ");
      
      bp::def("removeCollisionPairs",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairs),
              bp::args("Model", "GeometryModel (where pairs are removed)","SRDF filename (string)", "verbosity"),
              "Parse an SRDF file in order to desactivte collision pairs for a specific GeometryModel.");
      
      bp::def("removeCollisionPairsFromXML",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairsFromXML),
              bp::args("Model", "GeometryModel (where pairs are removed)","string containing the XML-SRDF", "verbosity"),
              "Parse an SRDF file in order to desactivte collision pairs for a specific GeometryModel.");

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif // #ifdef PINOCCHIO_WITH_URDFDOM
      
#ifdef PINOCCHIO_WITH_LUA5
      bp::def("buildModelFromLua",buildModelFromLua,
              bp::args("Filename (string)",
                       "Free flyer (bool, false for a fixed robot)",
                       "Verbose option "),
              "Parse the URDF file given in input and return a proper pinocchio model");
#endif // #ifdef PINOCCHIO_WITH_LUA5

      bp::def("loadReferenceConfigurations",
              static_cast<void (*)(Model &, const std::string &, const bool)>(&srdf::loadReferenceConfigurations),
              bp::args("Model for which we want the neutral config","srdf filename (string)", "verbosity"
                       ),
              "Get the reference configurations of a given model from the SRDF file.");
     
      bp::def("loadRotorParameters",
              static_cast<bool (*)(Model &, const std::string &, const bool)>(&srdf::loadRotorParameters),
              bp::args("Model for which we are loading the rotor parameters",
                       "SRDF filename (string)", "verbosity"),
              "Load the rotor parameters of a given model from an SRDF file.\n"
              "Results are stored in model.rotorInertia and model.rotorGearRatio.");
    }
    
  }
} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_parsers_hpp__
