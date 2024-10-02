//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif
#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_URDFDOM
    typedef ::hpp::fcl::MeshLoaderPtr MeshLoaderPtr;

    void buildGeomFromUrdf_existing(
      const Model & model,
      const std::istream & stream,
      const GeometryType type,
      GeometryModel & geometry_model,
      bp::object py_pkg_dirs,
      bp::object py_mesh_loader)
    {
      MeshLoaderPtr mesh_loader = MeshLoaderPtr();
      if (!py_mesh_loader.is_none())
      {
  #ifdef PINOCCHIO_WITH_HPP_FCL
        PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
        PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
        mesh_loader = bp::extract<::hpp::fcl::MeshLoaderPtr>(py_mesh_loader);
        PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  #else
        PyErr_WarnEx(
          PyExc_UserWarning, "Mesh loader is ignored because Pinocchio is not built with hpp-fcl",
          1);
  #endif
      }

      std::vector<std::string> pkg_dirs;
      if (py_pkg_dirs.ptr() == Py_None)
      {
      }
      else if (PyList_Check(py_pkg_dirs.ptr()))
      {
        pkg_dirs = pathList(py_pkg_dirs);
      }
      else
      {
        pkg_dirs.push_back(path(py_pkg_dirs));
      }

      pinocchio::urdf::buildGeom(model, stream, type, geometry_model, pkg_dirs, mesh_loader);
    }

    // This function is complex in order to keep backward compatibility.
    GeometryModel * buildGeomFromUrdfStream(
      const Model & model,
      const std::istream & stream,
      const GeometryType type,
      bp::object py_geom_model,
      bp::object package_dirs,
      bp::object mesh_loader)
    {
      GeometryModel * geom_model;
      if (py_geom_model.is_none())
        geom_model = new GeometryModel;
      else
      {
        bp::extract<GeometryModel *> geom_model_extract(py_geom_model);
        if (geom_model_extract.check())
          geom_model = geom_model_extract();
        else
        {
          // When backward compat is removed, the code in this `else` section
          // can be removed and the argument py_geom_model changed into a GeometryModel*
          PyErr_WarnEx(
            PyExc_UserWarning,
            "You passed package dir(s) via argument geometry_model and provided package_dirs.", 1);

          // At this stage, py_geom_model contains the package dir(s). mesh_loader can
          // be passed either by package_dirs or mesh_loader
          bp::object new_pkg_dirs = py_geom_model;
          if (!package_dirs.is_none() && !mesh_loader.is_none())
            throw std::invalid_argument(
              "package_dirs and mesh_loader cannot be both provided since you passed the package "
              "dirs via argument geometry_model.");
          if (mesh_loader.is_none())
            mesh_loader = package_dirs;
          try
          {
            // If geom_model is not a valid package_dir(s), then rethrow with clearer message
            geom_model = new GeometryModel;
            buildGeomFromUrdf_existing(model, stream, type, *geom_model, new_pkg_dirs, mesh_loader);
            return geom_model;
          }
          catch (std::invalid_argument const & e)
          {
            std::cout << "Caught: " << e.what() << std::endl;
            throw std::invalid_argument("Argument geometry_model should be a GeometryModel");
          }
        }
      }
      buildGeomFromUrdf_existing(model, stream, type, *geom_model, package_dirs, mesh_loader);
      return geom_model;
    }

    GeometryModel * buildGeomFromUrdfFile(
      const Model & model,
      const bp::object & filename,
      const GeometryType type,
      bp::object geom_model,
      bp::object package_dirs,
      bp::object mesh_loader)
    {
      const std::string filename_s = path(filename);
      std::ifstream stream(filename_s.c_str());
      if (!stream.is_open())
      {
        throw std::invalid_argument(filename_s + " does not seem to be a valid file.");
      }
      return buildGeomFromUrdfStream(model, stream, type, geom_model, package_dirs, mesh_loader);
    }

    GeometryModel * buildGeomFromUrdfString(
      const Model & model,
      const std::string & xmlString,
      const GeometryType type,
      bp::object geom_model,
      bp::object package_dirs,
      bp::object mesh_loader)
    {
      std::istringstream stream(xmlString);
      return buildGeomFromUrdfStream(model, stream, type, geom_model, package_dirs, mesh_loader);
    }

  #ifdef PINOCCHIO_WITH_HPP_FCL
    #define MESH_LOADER_DOC                                                                        \
      "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
  #else // #ifdef PINOCCHIO_WITH_HPP_FCL
    #define MESH_LOADER_DOC "\tmesh_loader: unused because the Pinocchio is built without hpp-fcl\n"
  #endif // #ifdef PINOCCHIO_WITH_HPP_FCL
    template<std::size_t owner_arg = 1>
    struct return_value_policy : bp::return_internal_reference<owner_arg>
    {
    public:
      template<class ArgumentPackage>
      static PyObject * postcall(ArgumentPackage const & args_, PyObject * result)
      {
        // If owner_arg exists, we run bp::return_internal_reference postcall
        // result lifetime will be tied to the owner_arg lifetime
        PyObject * patient = bp::detail::get_prev<owner_arg>::execute(args_, result);
        if (patient != Py_None)
          return bp::return_internal_reference<owner_arg>::postcall(args_, result);
        // If owner_arg doesn't exist, then Python will have to manage the result lifetime
        bp::extract<GeometryModel *> geom_model_extract(result);
        if (geom_model_extract.check())
        {
          return bp::to_python_indirect<GeometryModel, bp::detail::make_owning_holder>()(
            geom_model_extract());
        }
        // If returned value is not a GeometryModel*, then raise an error
        PyErr_SetString(
          PyExc_RuntimeError,
          "pinocchio::python::return_value_policy only works on GeometryModel* data type");
        return 0;
      }
    };

    template<typename F>
    void defBuildUrdf(const char * name, F f, const char * urdf_arg, const char * urdf_doc)
    {
      std::ostringstream doc;
      doc << "Parse the URDF file given as input looking for the geometry of the given input model "
             "and\n"
             "and store either the collision geometries (GeometryType.COLLISION) or the visual "
             "geometries (GeometryType.VISUAL) in a GeometryModel object.\n"
             "Parameters:\n"
             "\tmodel: model of the robot\n"
             "\n"
          << urdf_arg << ": " << urdf_doc
          << "\n"
             "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for "
             "display or the COLLISION for collision detection).\n"
             "\tgeometry_model: if provided, this geometry model will be used to store the parsed "
             "information instead of creating a new one\n"
             "\tpackage_dirs: either a single path or a vector of paths pointing to folders "
             "containing the model of the robot\n" MESH_LOADER_DOC "\n"
             "Retuns:\n"
             "\ta new GeometryModel if `geometry_model` is None else `geometry_model` (that has "
             "been updated).\n";

      bp::def(
        name, f,
        (bp::arg("model"), bp::arg(urdf_arg), bp::arg("geom_type"),
         bp::arg("geometry_model") = static_cast<GeometryModel *>(NULL),
         bp::arg("package_dirs") = bp::object(), bp::arg("mesh_loader") = bp::object()),
        doc.str().c_str(), return_value_policy<4>());
    }

#endif

    void exposeURDFGeometry()
    {
#ifdef PINOCCHIO_WITH_URDFDOM
      defBuildUrdf(
        "buildGeomFromUrdf", buildGeomFromUrdfFile, "urdf_filename",
        "path to the URDF file containing the model of the robot");
      defBuildUrdf(
        "buildGeomFromUrdfString", buildGeomFromUrdfString, "urdf_string",
        "a string containing the URDF model of the robot");
#endif // #ifdef PINOCCHIO_WITH_URDFDOM
    }
  } // namespace python
} // namespace pinocchio
