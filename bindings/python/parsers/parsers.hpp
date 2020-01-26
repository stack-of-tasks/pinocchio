//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_parsers_hpp__
#define __pinocchio_python_parsers_hpp__

#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/bindings/python/multibody/data.hpp"

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

#include "pinocchio/bindings/python/multibody/geometry-model.hpp"
#include "pinocchio/bindings/python/multibody/geometry-data.hpp"

#include "pinocchio/parsers/srdf.hpp"

namespace pinocchio
{
  namespace python
  {
    struct ParsersPythonVisitor
    {

      template<class T1, class T2>
      struct PairToTupleConverter
      {
        static PyObject* convert(const std::pair<T1, T2> & pair)
        {
          return bp::incref(bp::make_tuple(pair.first, pair.second).ptr());
        }
      };
      
      
#ifdef PINOCCHIO_WITH_URDFDOM
      
      static Model buildModelFromUrdf(const std::string & filename)
      {
        Model model;
        pinocchio::urdf::buildModel(filename, model);
        return model;
      }
      
      static Model & buildModelFromUrdf(const std::string & filename,
                                        Model & model)
      {
        return pinocchio::urdf::buildModel(filename, model);
      }

      static Model buildModelFromUrdf(const std::string & filename,
                                      bp::object & root_joint_object)
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant>(root_joint_object)();
        Model model;
        pinocchio::urdf::buildModel(filename, root_joint, model);
        return model;
      }
      
      static Model & buildModelFromUrdf(const std::string & filename,
                                        bp::object & root_joint_object,
                                        Model & model)
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant>(root_joint_object)();
        return pinocchio::urdf::buildModel(filename, root_joint, model);
      }
      
      static Model buildModelFromXML(const std::string & XMLstream,
                                     bp::object & root_joint_object)
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        Model model;
        pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
        return model;
      }
      
      static Model & buildModelFromXML(const std::string & XMLstream,
                                     bp::object & root_joint_object,
                                     Model & model)
      {
        JointModelVariant root_joint = bp::extract<JointModelVariant> (root_joint_object)();
        pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
        return model;
      }
      
      static Model buildModelFromXML(const std::string & XMLstream)
      {
        Model model;
        pinocchio::urdf::buildModelFromXML(XMLstream, model);
        return model;
      }
      
      static Model & buildModelFromXML(const std::string & XMLstream,
                                     Model & model)
      {
        pinocchio::urdf::buildModelFromXML(XMLstream, model);
        return model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type)
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model);
        
        return geometry_model;
      }

      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model)
      {
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model);
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const std::vector<std::string> & package_dirs)
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
        
        return geometry_model;
      }

      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model,
                        const std::vector<std::string> & package_dirs)
      {
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
        
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const std::string & package_dir)
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dir);

        return geometry_model;
      }

      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model,
                        const std::string & package_dir)
      {
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dir);

        return geometry_model;
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        std::vector<std::string> hints;
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,hints,meshLoader);
        
        return geometry_model;
      }
      
      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        std::vector<std::string> hints;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,hints,meshLoader);
        
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const std::vector<std::string> & package_dirs,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs,meshLoader);
        
        return geometry_model;
      }

      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model,
                        const std::vector<std::string> & package_dirs,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs,meshLoader);
        
        return geometry_model;
      }

      static GeometryModel
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        const std::string & package_dir,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        GeometryModel geometry_model;
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dir,meshLoader);

        return geometry_model;
      }

      static GeometryModel &
      buildGeomFromUrdf(const Model & model,
                        const std::string & filename,
                        const GeometryType type,
                        GeometryModel & geometry_model,
                        const std::string & package_dir,
                        const fcl::MeshLoaderPtr & meshLoader)
      {
        pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dir,meshLoader);

        return geometry_model;
      }

      BOOST_PYTHON_FUNCTION_OVERLOADS(removeCollisionPairs_overload,
                                      srdf::removeCollisionPairs,
                                      3,4)

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL

#endif // #ifdef PINOCCHIO_WITH_URDFDOM
      
      BOOST_PYTHON_FUNCTION_OVERLOADS(loadReferenceConfigurations_overload,
                                      srdf::loadReferenceConfigurations,
                                      2,3)
      
      static void loadReferenceConfigurationsFromXML(Model& model,
                                                     const std::string & xmlStream,
                                                     const bool verbose = false)
      {
        std::istringstream iss (xmlStream);
        pinocchio::srdf::loadReferenceConfigurationsFromXML(model, iss, verbose);
      }

      BOOST_PYTHON_FUNCTION_OVERLOADS(loadReferenceConfigurationsFromXML_overload,
                                      loadReferenceConfigurationsFromXML,
                                      2,3)
      
      BOOST_PYTHON_FUNCTION_OVERLOADS(loadRotorParameters_overload,
                                      srdf::loadRotorParameters,
                                      2,3)
      
      /* --- Expose --------------------------------------------------------- */
      static void expose();
    }; // struct ParsersPythonVisitor

    inline void ParsersPythonVisitor::expose()
    {
#ifdef PINOCCHIO_WITH_URDFDOM
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("urdf_filename","root_joint"),
              "Parse the URDF file given in input and return a pinocchio Model starting with the given root joint."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("urdf_filename"),
              "Parse the URDF file given in input and return a pinocchio Model."
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model & (*) (const std::string &, Model &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("urdf_filename","model"),
              "Append to a given model a URDF structure given by its filename.",
              bp::return_internal_reference<2>()
              );
      
      bp::def("buildModelFromUrdf",
              static_cast <Model & (*) (const std::string &, bp::object &, Model &)> (&ParsersPythonVisitor::buildModelFromUrdf),
              bp::args("urdf_filename","root_joint","model"),
              "Append to a given model a URDF structure given by its filename and the root joint.",
              bp::return_internal_reference<3>()
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &, bp::object &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("urdf_xml_stream","root_joint"),
              "Parse the URDF XML stream given in input and return a pinocchio Model starting with the given root joint."
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model & (*) (const std::string &, bp::object &, Model &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("urdf_xml_stream","root_joint","model"),
              "Parse the URDF XML stream given in input and append it to the input model with the given interfacing joint.",
              bp::return_internal_reference<3>()
              );
      
      bp::def("buildModelFromXML",
              static_cast <Model (*) (const std::string &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("urdf_xml_stream"),
              "Parse the URDF XML stream given in input and return a pinocchio Model."
              );
              
      bp::def("buildModelFromXML",
              static_cast <Model & (*) (const std::string &, Model &)> (&ParsersPythonVisitor::buildModelFromXML),
              bp::args("urdf_xml_stream","model"),
              "Parse the URDF XML stream given in input and append it to the input model.",
              bp::return_internal_reference<2>()
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::vector<std::string> &)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","package_dirs"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tpackage_dirs: vector of paths pointing to the folders containing the model of the robot\n"
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::vector<std::string> &)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model","package_dirs"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "\tpackage_dirs: vector of paths pointing to the folders containing the model of the robot\n",
              bp::return_internal_reference<4>()
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "Note:\n"
              "This function does not take any hint concerning the location of the meshes of the robot."
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "Note:\n"
              "This function does not take any hint concerning the location of the meshes of the robot.",
              bp::return_internal_reference<4>()
              );

      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::string &)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","package_dir"  ),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
              );

      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::string &)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model","package_dir"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n",
              bp::return_internal_reference<4>()
              );

#ifdef PINOCCHIO_WITH_HPP_FCL
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::vector<std::string> &, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","package_dirs","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tpackage_dirs: vector of paths pointing to the folders containing the model of the robot\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries)."
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::vector<std::string> &, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model","package_dirs","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "\tpackage_dirs: vector of paths pointing to the folders containing the model of the robot\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).",
              bp::return_internal_reference<4>()
              );

      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::string &, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","package_dir","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries)."
              );

      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::string &, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model","package_dir","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).",
              bp::return_internal_reference<4>()
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
              "Note:\n"
              "This function does not take any hint concerning the location of the meshes of the robot."
              );
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const fcl::MeshLoaderPtr&)> (&ParsersPythonVisitor::buildGeomFromUrdf),
              bp::args("model","urdf_filename","geom_type","geom_model","mesh_loader"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: reference where to store the parsed information\n"
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
              "Note:\n"
              "This function does not take any hint concerning the location of the meshes of the robot.",
              bp::return_internal_reference<4>()
              );
      
      bp::def("removeCollisionPairs",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairs),
              removeCollisionPairs_overload(bp::args("model", "geom_model","srdf_filename", "verbose"),
                                            "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n""Parameters:\n"
                                            "Parameters:\n"
                                            "\tmodel: model of the robot\n"
                                            "\tgeom_model: geometry model of the robot\n"
                                            "\tsrdf_filename: path to the SRDF file containing the collision pairs to remove\n"
                                            "\tverbose: [optional] display to the current terminal some internal information"
                                            ));
      
      bp::def("removeCollisionPairsFromXML",
              static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(&srdf::removeCollisionPairsFromXML),
              removeCollisionPairs_overload(bp::args("model", "geom_model","srdf_xml_stream", "verbose"),
                                            "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n""Parameters:\n"
                                            "Parameters:\n"
                                            "\tmodel: model of the robot\n"
                                            "\tgeom_model: geometry model of the robot\n"
                                            "\tsrdf_xml_stream: XML stream containing the SRDF information with the collision pairs to remove\n"
                                            "\tverbose: [optional] display to the current terminal some internal information"
                                            ));

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif // #ifdef PINOCCHIO_WITH_URDFDOM

      bp::def("loadReferenceConfigurations",
              static_cast<void (*)(Model &, const std::string &, const bool)>(&srdf::loadReferenceConfigurations),
              loadReferenceConfigurations_overload(bp::args("model","srdf_filename","verbose"),
                                                   "Retrieve all the reference configurations of a given model from the SRDF file.\n"
                                                   "Parameters:\n"
                                                   "\tmodel: model of the robot\n"
                                                   "\tsrdf_filename: path to the SRDF file containing the reference configurations\n"
                                                   "\tverbose: [optional] display to the current terminal some internal information"
                                                   ));

      bp::def("loadReferenceConfigurationsFromXML",
              static_cast<void (*)(Model &, const std::string &, const bool)>(&srdf::loadReferenceConfigurations),
              loadReferenceConfigurationsFromXML_overload(bp::args("model","srdf_xml_stream","verbose"),
                                                          "Retrieve all the reference configurations of a given model from the SRDF file.\n"
                                                          "Parameters:\n"
                                                          "\tmodel: model of the robot\n"
                                                          "\tsrdf_xml_stream: XML stream containing the SRDF information with the reference configurations\n"
                                                          "\tverbose: [optional] display to the current terminal some internal information"
                                                          ));
     
      bp::def("loadRotorParameters",
              static_cast<bool (*)(Model &, const std::string &, const bool)>(&srdf::loadRotorParameters),
              loadRotorParameters_overload(bp::args("model","srdf_filename","verbose"),
                                           "Load the rotor parameters of a given model from a SRDF file.\n"
                                           "Results are stored in model.rotorInertia and model.rotorGearRatio."
                                           "Parameters:\n"
                                           "\tmodel: model of the robot\n"
                                           "\tsrdf_filename: path to the SRDF file containing the rotor parameters\n"
                                           "\tverbose: [optional] display to the current terminal some internal information"
                                           ));
    }
    
  }
} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_parsers_hpp__
