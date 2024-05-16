//
// Copyright (c) 2021-2024 INRIA
//

#ifndef __pinocchio_serialization_multibody_geometry_hpp__
#define __pinocchio_serialization_multibody_geometry_hpp__

#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#ifdef PINOCCHIO_WITH_HPP_FCL
  #define HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION
  #include <hpp/fcl/serialization/collision_data.h>
  #undef HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION
  #include <hpp/fcl/serialization/geometric_shapes.h>
  #include <hpp/fcl/serialization/hfield.h>
  #if HPP_FCL_VERSION_AT_LEAST(3, 0, 0)
    #include <hpp/fcl/serialization/octree.h>
  #endif
  #include <hpp/fcl/serialization/convex.h>
  #include <hpp/fcl/serialization/BVH_model.h>
#endif // PINOCCHIO_WITH_HPP_FCL

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/spatial.hpp"
#include "pinocchio/serialization/fcl.hpp"

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void serialize(
      Archive & ar, pinocchio::CollisionPair & collision_pair, const unsigned int /*version*/)
    {
      ar & make_nvp("pair", base_object<pinocchio::CollisionPair::Base>(collision_pair));
    }

    template<class Archive, typename Derived>
    void serialize(
      Archive & ar, pinocchio::ModelItem<Derived> & model_item, const unsigned int /*version*/)
    {
      ar & make_nvp("name", model_item.name);
      ar & make_nvp("parentFrame", model_item.parentFrame);
      ar & make_nvp("parentJoint", model_item.parentJoint);
      ar & make_nvp("placement", model_item.placement);
    }

    template<class Archive>
    void serialize(
      Archive & ar, pinocchio::GeometryObject & geometry_object, const unsigned int /*version*/)
    {
      ar & make_nvp("base", base_object<pinocchio::GeometryObject::Base>(geometry_object));
      ar & make_nvp("geometry", geometry_object.geometry);
      ar & make_nvp("meshPath", geometry_object.meshPath);
      ar & make_nvp("meshScale", geometry_object.meshScale);
      ar & make_nvp("overrideMaterial", geometry_object.overrideMaterial);
      ar & make_nvp("meshColor", geometry_object.meshColor);
      ar & make_nvp("meshTexturePath", geometry_object.meshTexturePath);
      ar & make_nvp("disableCollision", geometry_object.disableCollision);
    }

    template<class Archive>
    void save_construct_data(
      Archive & ar,
      const pinocchio::GeometryObject * geometry_object_ptr,
      const unsigned int /*version*/)
    {
      ar << make_nvp(
        "base", *static_cast<const pinocchio::GeometryObject::Base *>(geometry_object_ptr));
    }

    template<class Archive>
    void load_construct_data(
      Archive & ar, pinocchio::GeometryObject * geometry_object_ptr, const unsigned int /*version*/)
    {
      pinocchio::GeometryObject::Base base_obj;
      ar >> make_nvp("base", base_obj);

      new (geometry_object_ptr) pinocchio::GeometryObject(
        base_obj.name, base_obj.parentJoint, base_obj.parentFrame, base_obj.placement, nullptr);
    }

    template<class Archive>
    void
    serialize(Archive & ar, pinocchio::GeometryModel & geom_model, const unsigned int /*version*/)
    {
      ar & make_nvp("ngeoms", geom_model.ngeoms);
      ar & make_nvp("geometryObjects", geom_model.geometryObjects);
      ar & make_nvp("collisionPairs", geom_model.collisionPairs);
      ar & make_nvp("collisionPairMapping", geom_model.collisionPairMapping);
    }

    template<class Archive>
    void
    serialize(Archive & ar, pinocchio::GeometryData & geom_data, const unsigned int /*version*/)
    {
      ar & make_nvp("oMg", geom_data.oMg);

      ar & make_nvp("activeCollisionPairs", geom_data.activeCollisionPairs);

#ifdef PINOCCHIO_WITH_HPP_FCL
      // TODO(jcarpent): check whether the archive has been registered with HPP_FCL module ON or
      // OFF.
      ar & make_nvp("distanceRequests", geom_data.distanceRequests);
      ar & make_nvp("distanceResults", geom_data.distanceResults);
      ar & make_nvp("collisionRequests", geom_data.collisionRequests);
      ar & make_nvp("collisionResults", geom_data.collisionResults);

      ar & make_nvp("radius", geom_data.radius);

      ar & make_nvp("collisionPairIndex", geom_data.collisionPairIndex);
#endif // PINOCCHIO_WITH_HPP_FCL

      ar & make_nvp("innerObjects", geom_data.innerObjects);
      ar & make_nvp("outerObjects", geom_data.outerObjects);
    }

  } // namespace serialization
} // namespace boost

#endif // ifndef __pinocchio_serialization_multibody_geometry_hpp__
