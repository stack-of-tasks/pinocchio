// Copyright (c) 2026 INRIA

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
  #include <coal/mesh_loader/loader.h>
#endif

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

#ifdef PINOCCHIO_WITH_URDFDOM

static std::vector<std::string> pkgDirsFromObject(nb::object py_pkg_dirs)
{
  std::vector<std::string> pkg_dirs;
  if (py_pkg_dirs.is_none())
    return pkg_dirs;
  if (nb::isinstance<nb::list>(py_pkg_dirs))
  {
    for (auto && item : nb::cast<nb::list>(py_pkg_dirs))
      pkg_dirs.push_back(nb::cast<std::filesystem::path>(item).string());
  }
  else
  {
    pkg_dirs.push_back(nb::cast<std::filesystem::path>(py_pkg_dirs).string());
  }
  return pkg_dirs;
}

static ::coal::MeshLoaderPtr meshLoaderFromObject(nb::object py_mesh_loader)
{
  if (py_mesh_loader.is_none())
    return nullptr;
  #ifdef PINOCCHIO_WITH_COLLISION
  return nb::cast<::coal::MeshLoaderPtr>(py_mesh_loader);
  #else
  PyErr_WarnEx(
    PyExc_UserWarning, "Mesh loader is ignored because Pinocchio is not built with coal", 1);
  return nullptr;
  #endif
}

#endif // PINOCCHIO_WITH_URDFDOM

void exposeURDFGeometry(nb::module_ m)
{
#ifdef PINOCCHIO_WITH_URDFDOM
  const char * doc_file_new =
    "Parse the URDF file given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in a new GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\turdf_filename: path to the URDF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the model of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\ta new GeometryModel";

  const char * doc_file_existing =
    "Parse the URDF file given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in the provided GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\turdf_filename: path to the URDF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\tgeometry_model: the geometry model to fill with the parsed information\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the model of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\tgeometry_model (that has been updated)";

  const char * doc_string_new =
    "Parse the URDF string given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in a new GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\turdf_string: a string containing the URDF model of the robot\n"
    "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the model of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\ta new GeometryModel";

  const char * doc_string_existing =
    "Parse the URDF string given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in the provided GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\turdf_string: a string containing the URDF model of the robot\n"
    "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\tgeometry_model: the geometry model to fill with the parsed information\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the model of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\tgeometry_model (that has been updated)";

  // buildGeomFromUrdf (file) - creates new GeometryModel
  m.def(
    "buildGeomFromUrdf",
    [](
      const Model & model, const std::filesystem::path & filename, const GeometryType type,
      nb::object package_dirs, nb::object mesh_loader) {
      GeometryModel geom_model;
      pinocchio::urdf::buildGeom(
        model, filename.string(), type, geom_model, pkgDirsFromObject(std::move(package_dirs)),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "urdf_filename"_a, "geom_type"_a, "package_dirs"_a = nb::none(),
    "mesh_loader"_a = nb::none(), doc_file_new);

  // buildGeomFromUrdf (file) - fills existing GeometryModel
  m.def(
    "buildGeomFromUrdf",
    [](
      const Model & model, const std::filesystem::path & filename, const GeometryType type,
      GeometryModel & geom_model, nb::object package_dirs,
      nb::object mesh_loader) -> GeometryModel & {
      pinocchio::urdf::buildGeom(
        model, filename.string(), type, geom_model, pkgDirsFromObject(std::move(package_dirs)),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "urdf_filename"_a, "geom_type"_a, "geometry_model"_a, "package_dirs"_a = nb::none(),
    "mesh_loader"_a = nb::none(), doc_file_existing, nb::rv_policy::reference);

  // buildGeomFromUrdfString - creates new GeometryModel
  m.def(
    "buildGeomFromUrdfString",
    [](
      const Model & model, const std::string & urdf_string, const GeometryType type,
      nb::object package_dirs, nb::object mesh_loader) {
      std::istringstream stream(urdf_string);
      GeometryModel geom_model;
      pinocchio::urdf::buildGeom(
        model, stream, type, geom_model, pkgDirsFromObject(std::move(package_dirs)),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "urdf_string"_a, "geom_type"_a, "package_dirs"_a = nb::none(),
    "mesh_loader"_a = nb::none(), doc_string_new);

  // buildGeomFromUrdfString - fills existing GeometryModel
  m.def(
    "buildGeomFromUrdfString",
    [](
      const Model & model, const std::string & urdf_string, const GeometryType type,
      GeometryModel & geom_model, nb::object package_dirs,
      nb::object mesh_loader) -> GeometryModel & {
      std::istringstream stream(urdf_string);
      pinocchio::urdf::buildGeom(
        model, stream, type, geom_model, pkgDirsFromObject(std::move(package_dirs)),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "urdf_string"_a, "geom_type"_a, "geometry_model"_a, "package_dirs"_a = nb::none(),
    "mesh_loader"_a = nb::none(), doc_string_existing, nb::rv_policy::reference);
#endif
  (void)m;
}

PINOCCHIO_PYTHON_NAMESPACE_END
