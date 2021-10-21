//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
  
#ifdef PINOCCHIO_WITH_URDFDOM

    void
    buildGeomFromUrdf_existing(const Model & model,
                               const std::istream & stream,
                               const GeometryType type,
                               GeometryModel & geometry_model,
                               bp::object py_pkg_dirs,
                               bp::object py_mesh_loader)
    {
      ::hpp::fcl::MeshLoaderPtr mesh_loader = ::hpp::fcl::MeshLoaderPtr();
      if (!py_mesh_loader.is_none()) {
#ifdef PINOCCHIO_WITH_HPP_FCL
        mesh_loader = bp::extract<::hpp::fcl::MeshLoaderPtr>(py_mesh_loader);
#else
        PyErr_WarnEx(PyExc_UserWarning, "Mesh loader is ignored because Pinocchio is not built with hpp-fcl", 1);
#endif
      }

      std::vector<std::string> pkg_dirs;

      bp::extract<std::string> pkg_dir_extract(py_pkg_dirs);
      bp::extract<bp::list> pkg_dirs_list_extract(py_pkg_dirs);
      bp::extract<const std::vector<std::string>&> pkg_dirs_vect_extract(py_pkg_dirs);
      if (py_pkg_dirs.is_none()) {} // Provided None
      else if (pkg_dir_extract.check()) // Provided a string
        pkg_dirs.push_back(pkg_dir_extract());
      else if (pkg_dirs_list_extract.check()) // Provided a list of string
        extract(pkg_dirs_list_extract(), pkg_dirs);
      else if (pkg_dirs_vect_extract.check()) // Provided a vector of string
        pkg_dirs = pkg_dirs_vect_extract();
      else { // Did not understand the provided argument
        std::string what = bp::extract<std::string>(py_pkg_dirs.attr("__str__")())();
        throw std::invalid_argument("pkg_dirs must be either None, a string or a list of strings. Provided " + what);
      }

      pinocchio::urdf::buildGeom(model,stream,type,geometry_model,pkg_dirs,mesh_loader);
    }

    bp::object
    buildGeomFromUrdf_new(const Model & model,
                          const std::istream & stream,
                          const GeometryType type,
                          bp::object py_pkg_dirs,
                          bp::object mesh_loader)
    {
      GeometryModel* geometry_model = new GeometryModel;
      buildGeomFromUrdf_existing(model, stream, type, *geometry_model, py_pkg_dirs, mesh_loader);
      bp::manage_new_object::apply<GeometryModel*>::type converter;
      return bp::object(bp::handle<>(converter(geometry_model)));
    }

    // This function is complex in order to keep backward compatibility.
    bp::object
    buildGeomFromUrdfStream(const Model & model,
                            const std::istream & stream,
                            const GeometryType type,
                            bp::object geom_model,
                            bp::object package_dirs,
                            bp::object mesh_loader)
    {
      if (geom_model.is_none()) {
        return buildGeomFromUrdf_new(model, stream, type, package_dirs, mesh_loader);
      } else {
        bp::extract<GeometryModel&> geom_model_extract(geom_model);
        if (geom_model_extract.check()) {
          buildGeomFromUrdf_existing(model, stream, type, geom_model_extract(), package_dirs, mesh_loader);
          return geom_model;
        }
        // When backward compat is removed, what comes after this comment can be
        // replaced by
        // throw std::invalid_argument("Argument geometry_model should be a GeometryModel or None");
        PyErr_WarnEx(PyExc_UserWarning,
          "You passed package dir(s) via argument geometry_model and provided package_dirs.",1);

        // At this stage, geom_model contains the package dir(s). mesh_loader can
        // be passed either by package_dirs or mesh_loader
        if (!package_dirs.is_none() && !mesh_loader.is_none())
          throw std::invalid_argument("package_dirs and mesh_loader cannot be both provided since you passed the package dirs via argument geometry_model.");
        try {
          // If geom_model is not a valid package_dir(s), then rethrow with clearer message
          return buildGeomFromUrdf_new(model, stream, type, geom_model, (mesh_loader.is_none() ? package_dirs : mesh_loader));
        } catch (std::invalid_argument const& e) {
          std::cout << "Caught: " << e.what() << std::endl;
          throw std::invalid_argument("Argument geometry_model should be a GeometryModel");
        }
      }
    }

    bp::object
    buildGeomFromUrdfFile(const Model & model,
                          const std::string & filename,
                          const GeometryType type,
                          bp::object geom_model,
                          bp::object package_dirs,
                          bp::object mesh_loader)
    {
      std::ifstream stream(filename.c_str());
      if (!stream.is_open())
      {
        throw std::invalid_argument(filename + " does not seem to be a valid file.");
      }
      return buildGeomFromUrdfStream(model, stream, type, geom_model, package_dirs, mesh_loader);
    }

    bp::object
    buildGeomFromUrdfString(const Model & model,
                            const std::string & xmlString,
                            const GeometryType type,
                            bp::object geom_model,
                            bp::object package_dirs,
                            bp::object mesh_loader)
    {
      std::istringstream stream(xmlString);
      return buildGeomFromUrdfStream(model, stream, type, geom_model, package_dirs, mesh_loader);
    }

#endif
  
    void exposeURDFGeometry()
    {

#ifdef PINOCCHIO_WITH_URDFDOM

      bp::def("buildGeomFromUrdf", buildGeomFromUrdfFile,
              (bp::arg("model"), bp::arg("urdf_filename"), bp::arg("geom_type"),
              bp::arg("geom_model") = bp::object(),
              bp::arg("package_dirs") = bp::object(),
              bp::arg("mesh_loader") = bp::object()),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one\n"
              "\tpackage_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot\n"
#ifdef PINOCCHIO_WITH_HPP_FCL
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
#else // #ifdef PINOCCHIO_WITH_HPP_FCL
              "\tmesh_loader: unused because the Pinocchio is built without hpp-fcl\n"
#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
              );

      bp::def("buildGeomFromUrdfString", buildGeomFromUrdfString,
              (bp::arg("model"), bp::arg("urdf_string"), bp::arg("geom_type"),
              bp::arg("geom_model") = bp::object(),
              bp::arg("package_dirs") = bp::object(),
              bp::arg("mesh_loader") = bp::object()),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_string: a string containing the URDF model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tgeom_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one\n"
              "\tpackage_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot\n"
#ifdef PINOCCHIO_WITH_HPP_FCL
              "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
#else // #ifdef PINOCCHIO_WITH_HPP_FCL
              "\tmesh_loader: unused because the Pinocchio is built without hpp-fcl\n"
#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
              );

#endif // #ifdef PINOCCHIO_WITH_URDFDOM
    }
  }
}

