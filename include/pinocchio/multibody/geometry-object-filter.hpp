//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_geometry_object_filter_hpp__
#define __pinocchio_multibody_geometry_object_filter_hpp__

#include "pinocchio/multibody/instance-filter.hpp"

namespace pinocchio
{

  struct GeometryObjectFilterBase : InstanceFilterBase<GeometryObject>
  {

  }; // struct GeometryObjectFilterBase

  struct GeometryObjectFilterNothing final : GeometryObjectFilterBase
  {
    bool operator()(const GeometryObject &) const
    {
      return true;
    }

  }; // struct GeometryObjectFilterNothing

  struct GeometryObjectFilterSelectByJoint final : GeometryObjectFilterBase
  {
    GeometryObjectFilterSelectByJoint(const size_t joint_id)
    : joint_id(joint_id)
    {
    }

    bool operator()(const GeometryObject & geometry_object) const
    {
      return geometry_object.parentJoint == joint_id;
    }

    const size_t joint_id;

  }; // struct GeometryObjectFilterSelectByJoint

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_geometry_object_filter_hpp__
