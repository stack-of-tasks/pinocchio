//
// Copyright (c) 2020-2021 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_SDFORMAT
  #include "pinocchio/parsers/sdf.hpp"
#endif
#include "pinocchio/bindings/python/parsers/sdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_SDFORMAT

    GeometryModel
    buildGeomFromSdf(const Model & model, const std::string & filename, const GeometryType type)
    {
      GeometryModel geometry_model;
      const std::string & rootLinkName = "";
      const std::string & packageDir = "";
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, packageDir);
      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName)
    {
      GeometryModel geometry_model;
      const std::string & packageDir = "";
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, packageDir);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName)
    {
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName);
      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName,
      const std::string & packageDir)
    {
      GeometryModel geometry_model;
      const std::vector<std::string> dirs(1, packageDir);
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, dirs);

      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName,
      const std::vector<std::string> & package_dirs)
    {
      GeometryModel geometry_model;
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, package_dirs);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName,
      const std::vector<std::string> & package_dirs)
    {
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, package_dirs);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName,
      const std::string & package_dir)
    {
      pinocchio::sdf::buildGeom(model, filename, type, geometry_model, rootLinkName, package_dir);
      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      std::vector<std::string> hints;
      GeometryModel geometry_model;
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, hints, meshLoader);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      std::vector<std::string> hints;
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, hints, meshLoader);
      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName,
      const std::vector<std::string> & package_dirs,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      GeometryModel geometry_model;
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, package_dirs, meshLoader);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName,
      const std::vector<std::string> & package_dirs,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, package_dirs, meshLoader);
      return geometry_model;
    }

    GeometryModel buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      const std::string & rootLinkName,
      const std::string & package_dir,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      GeometryModel geometry_model;
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, package_dir, meshLoader);
      return geometry_model;
    }

    GeometryModel & buildGeomFromSdf(
      const Model & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geometry_model,
      const std::string & rootLinkName,
      const std::string & package_dir,
      const hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      pinocchio::sdf::buildGeom(
        model, filename, type, geometry_model, rootLinkName, package_dir, meshLoader);
      return geometry_model;
    }

#endif // #ifdef PINOCCHIO_WITH_SDFORMAT

    void exposeSDFGeometry()
    {
#ifdef PINOCCHIO_WITH_SDFORMAT

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(const Model &, const std::string &, const GeometryType)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &,
          const std::string &)>(pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type", "root_link_name", "package_dir"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &,
          const std::vector<std::string> &)>(pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type", "root_link_name", "package_dirs"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tpackage_dirs: vector of paths pointing to the folders containing the model of the "
        "robot\n");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &,
                                        const std::vector<std::string> &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "geom_model", "root_link_name", "package_dirs"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "\tpackage_dirs: vector of paths pointing to the folders containing the model of the "
        "robot\n",
        bp::return_internal_reference<4>());

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type", "root_link_name"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "Note:\n"
        "This function does not take any hint concerning the location of the meshes of the robot.");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type", "geom_model", "root_link_name"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "Note:\n"
        "This function does not take any hint concerning the location of the meshes of the robot.",
        bp::return_internal_reference<4>());

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &, const std::string &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "geom_model", "root_link_name", "package_dir"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n",
        bp::return_internal_reference<4>());

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &,
          const std::vector<std::string> &, const hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "root_link_name", "package_dirs", "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tpackage_dirs: vector of paths pointing to the folders containing the model of the "
        "robot\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &,
                                        const std::vector<std::string> &,
                                        const hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "geom_model", "root_link_name", "package_dirs",
          "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "\tpackage_dirs: vector of paths pointing to the folders containing the model of the "
        "robot\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).",
        bp::return_internal_reference<4>());

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &,
          const std::string &, const hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "root_link_name", "package_dir", "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &, const std::string &,
                                        const hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "geom_model", "root_link_name", "package_dir",
          "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).",
        bp::return_internal_reference<4>());

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel (*)(
          const Model &, const std::string &, const GeometryType, const std::string &,
          const hpp::fcl::MeshLoaderPtr &)>(pinocchio::python::buildGeomFromSdf),
        bp::args("model", "sdf_filename", "geom_type", "root_link_name", "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
        "Note:\n"
        "This function does not take any hint concerning the location of the meshes of the robot.");

      bp::def(
        "buildGeomFromSdf",
        static_cast<GeometryModel & (*)(const Model &, const std::string &, const GeometryType,
                                        GeometryModel &, const std::string &,
                                        const hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromSdf),
        bp::args(
          "model", "sdf_filename", "geom_type", "geom_model", "root_link_name", "mesh_loader"),
        "Parse the SDF file given as input looking for the geometry of the given input model and\n"
        "and store either the collision geometries (GeometryType.COLLISION) or the visual "
        "geometries (GeometryType.VISUAL) in the geom_model given as input.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsdf_filename: path to the SDF file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the SDF file (either the VISUAL for display "
        "or the COLLISION for collision detection).\n"
        "\tgeom_model: reference where to store the parsed information\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n"
        "Note:\n"
        "This function does not take any hint concerning the location of the meshes of the robot.",
        bp::return_internal_reference<4>());
#endif
    }
  } // namespace python
} // namespace pinocchio
