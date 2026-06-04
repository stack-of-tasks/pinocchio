// Copyright (c) 2026 INRIA

#ifdef PINOCCHIO_WITH_SDFORMAT
  #include "pinocchio/parsers/sdf.hpp"
#endif

#include "../conversion-util.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeSDFGeometry(nb::module_ m)
{
#ifdef PINOCCHIO_WITH_SDFORMAT
  const char * doc_new =
    "Parse the SDF file given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in a new GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tsdf_filename: path to the SDF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\troot_link_name: name of the root link\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the meshes of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\ta new GeometryModel";

  const char * doc_existing =
    "Parse the SDF file given as input looking for the geometry of the given input model\n"
    "and store either the collision geometries (GeometryType.COLLISION) or the visual\n"
    "geometries (GeometryType.VISUAL) in the provided GeometryModel.\n\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tsdf_filename: path to the SDF file containing the model of the robot\n"
    "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for\n"
    "\t\tdisplay or the COLLISION for collision detection)\n"
    "\tgeom_model: the geometry model to fill with the parsed information\n"
    "\troot_link_name: name of the root link\n"
    "\tpackage_dirs: either a single path or a vector of paths pointing to folders\n"
    "\t\tcontaining the meshes of the robot\n"
    "\tmesh_loader: an coal mesh loader (to load only once the related geometries)\n\n"
    "Returns:\n"
    "\tgeom_model (that has been updated)";

  // buildGeomFromSdf - creates new GeometryModel
  m.def(
    "buildGeomFromSdf",
    [](
      const Model & model, const std::filesystem::path & filename, const GeometryType type,
      const std::string & root_link_name, const PyPkgDirArg & package_dirs,
      nb::object mesh_loader) {
      GeometryModel geom_model;
      pinocchio::sdf::buildGeom(
        model, filename.string(), type, geom_model, root_link_name, pkgDirsFromArg(package_dirs),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "sdf_filename"_a, "geom_type"_a, "root_link_name"_a = "",
    "package_dirs"_a = nb::list(), "mesh_loader"_a = nb::none(), doc_new);

  // buildGeomFromSdf - fills existing GeometryModel
  m.def(
    "buildGeomFromSdf",
    [](
      const Model & model, const std::filesystem::path & filename, const GeometryType type,
      GeometryModel & geom_model, const std::string & root_link_name,
      const PyPkgDirArg & package_dirs, nb::object mesh_loader) -> GeometryModel & {
      pinocchio::sdf::buildGeom(
        model, filename.string(), type, geom_model, root_link_name, pkgDirsFromArg(package_dirs),
        meshLoaderFromObject(std::move(mesh_loader)));
      return geom_model;
    },
    "model"_a, "sdf_filename"_a, "geom_type"_a, "geom_model"_a, "root_link_name"_a = "",
    "package_dirs"_a = nb::list(), "mesh_loader"_a = nb::none(), doc_existing,
    nb::rv_policy::reference);
#endif
  (void)m;
}

PINOCCHIO_PYTHON_NAMESPACE_END
