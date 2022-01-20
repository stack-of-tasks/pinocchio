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
    buildGeomFromSdf(Model & model,
                     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                     const std::string & filename,
                     const GeometryType type,
                     const std::string & packageDir)
    {
      GeometryModel geometry_model;
      const std::vector<std::string> dirs(1,packageDir);
      pinocchio::sdf::buildGeom(model,contact_models,
                                filename,type,geometry_model,dirs);
      
      return geometry_model;
    }
#endif

    void exposeSDFGeometry()
    {
#ifdef PINOCCHIO_WITH_SDFORMAT
      bp::def("buildGeomFromSdf",
              static_cast <GeometryModel (*) (Model &, PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)&, const std::string &, const GeometryType, const std::string &)> (pinocchio::python::buildGeomFromSdf),
              bp::args("model","contact_models","urdf_filename","geom_type","package_dir"),
              "Parse the URDF file given as input looking for the geometry of the given input model and\n"
              "return a GeometryModel containing either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL).\n"
              "Parameters:\n"
              "\tmodel: model of the robot\n"
              "\turdf_filename: path to the URDF file containing the model of the robot\n"
              "\tgeom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).\n"
              "\tpackage_dir: path pointing to the folder containing the meshes of the robot\n"
              );
#endif
    }
  }
}
