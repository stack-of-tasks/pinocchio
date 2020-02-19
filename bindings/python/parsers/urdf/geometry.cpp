//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/urdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
  
#ifdef PINOCCHIO_WITH_URDFDOM
    
    GeometryModel
    buildGeomFromUrdf(const Model & model,
                      const std::string & filename,
                      const GeometryType type)
    {
      GeometryModel geometry_model;
      pinocchio::urdf::buildGeom(model,filename,type,geometry_model);
      
      return geometry_model;
    }

    GeometryModel &
    buildGeomFromUrdf(const Model & model,
                      const std::string & filename,
                      const GeometryType type,
                      GeometryModel & geometry_model)
    {
      pinocchio::urdf::buildGeom(model,filename,type,geometry_model);
      return geometry_model;
    }

    GeometryModel
    buildGeomFromUrdf(const Model & model,
                      const std::string & filename,
                      const GeometryType type,
                      const std::vector<std::string> & package_dirs)
    {
      GeometryModel geometry_model;
      pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
      
      return geometry_model;
    }

    GeometryModel &
    buildGeomFromUrdf(const Model & model,
                      const std::string & filename,
                      const GeometryType type,
                      GeometryModel & geometry_model,
                      const std::vector<std::string> & package_dirs)
    {
      pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dirs);
      
      return geometry_model;
    }

    GeometryModel
    buildGeomFromUrdf(const Model & model,
                      const std::string & filename,
                      const GeometryType type,
                      const std::string & package_dir)
    {
      GeometryModel geometry_model;
      pinocchio::urdf::buildGeom(model,filename,type,geometry_model,package_dir);

      return geometry_model;
    }

    GeometryModel &
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
    GeometryModel
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
        
    GeometryModel &
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

    GeometryModel
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

    GeometryModel &
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

    GeometryModel
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

    GeometryModel &
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

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif
  
    void exposeURDFGeometry()
    {

#ifdef PINOCCHIO_WITH_URDFDOM
      
      bp::def("buildGeomFromUrdf",
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::vector<std::string> &)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::vector<std::string> &)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::string &)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::string &)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::vector<std::string> &, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::vector<std::string> &, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const std::string &, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const std::string &, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel (*) (const Model &, const std::string &, const GeometryType, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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
              static_cast <GeometryModel & (*) (const Model &, const std::string &, const GeometryType, GeometryModel &, const fcl::MeshLoaderPtr&)> (pinocchio::python::buildGeomFromUrdf),
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

#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif // #ifdef PINOCCHIO_WITH_URDFDOM
    }
  }
}

