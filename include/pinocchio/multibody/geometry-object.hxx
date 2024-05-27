//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifndef __pinocchio_multibody_geometry_object_hxx__
#define __pinocchio_multibody_geometry_object_hxx__

#include <limits>

namespace pinocchio
{

  inline std::ostream & operator<<(std::ostream & os, const GeometryObject & geom_object)
  {
    os << "Name: \t \n"
       << geom_object.name << "\n"
       << "Parent frame ID: \t \n"
       << geom_object.parentFrame << "\n"
       << "Parent joint ID: \t \n"
       << geom_object.parentJoint << "\n"
       << "Position in parent frame: \t \n"
       << geom_object.placement << "\n"
       << "Absolute path to mesh file: \t \n"
       << geom_object.meshPath << "\n"
       << "Scale for transformation of the mesh: \t \n"
       << geom_object.meshScale.transpose() << "\n"
       << "Disable collision: \t \n"
       << geom_object.disableCollision << "\n"
       << std::endl;
    return os;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_geometry_object_hxx__
