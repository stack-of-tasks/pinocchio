//
// Copyright (c) 2015-2022 CNRS INRIA
//

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    GeometryModel
    buildGeomFromMJCF(Model & model, const bp::object & filename, const GeometryType & type)
    {
      GeometryModel geometry_model;
      ::pinocchio::mjcf::buildGeom(model, path(filename), type, geometry_model);
      return geometry_model;
    }

    GeometryModel buildGeomFromMJCF(
      Model & model,
      const bp::object & filename,
      const GeometryType & type,
      ::hpp::fcl::MeshLoaderPtr & meshLoader)
    {
      GeometryModel geometry_model;
      ::pinocchio::mjcf::buildGeom(model, path(filename), type, geometry_model, meshLoader);
      return geometry_model;
    }

    void exposeMJCFGeom()
    {
      bp::def(
        "buildGeomFromMJCF",
        static_cast<GeometryModel (*)(Model &, const bp::object &, const GeometryType &)>(
          pinocchio::python::buildGeomFromMJCF),
        bp::args("model", "mjcf_filename", "geom_type"),
        "Parse the Mjcf file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tfilename: path to the mjcf file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the mjcf file (either the VISUAL for "
        "display or the COLLISION for collision detection).\n");

      bp::def(
        "buildGeomFromMJCF",
        static_cast<GeometryModel (*)(
          Model &, const bp::object &, const GeometryType &, ::hpp::fcl::MeshLoaderPtr &)>(
          pinocchio::python::buildGeomFromMJCF),
        bp::args("model", "mjcf_filename", "geom_type", "mesh_loader"),
        "Parse the Mjcf file given as input looking for the geometry of the given input model and\n"
        "return a GeometryModel containing either the collision geometries "
        "(GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tfilename: path to the mjcf file containing the model of the robot\n"
        "\tgeom_type: type of geometry to extract from the mjcf file (either the VISUAL for "
        "display or the COLLISION for collision detection).\n"
        "\tmesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).\n");
    }
  } // namespace python
} // namespace pinocchio
