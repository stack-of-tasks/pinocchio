//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_broadphase_manager_hpp__
#define __pinocchio_multibody_broadphase_manager_hpp__

#ifdef PINOCCHIO_WITH_HPP_FCL

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/algorithm/broadphase-callbacks.hpp"

namespace pinocchio
{

template<typename BroadPhaseManagerDerived>
struct BroadPhaseManagerTpl
: BroadPhaseManagerDerived
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef BroadPhaseManagerDerived Base;
  
  BroadPhaseManagerTpl() // for std::vector
  : geometry_model_ptr(nullptr)
  , geometry_data_ptr(nullptr)
  {}
  
  BroadPhaseManagerTpl(const GeometryModel * geometry_model_ptr,
                       GeometryData * geometry_data_ptr)
  : geometry_model_ptr(geometry_model_ptr)
  , geometry_data_ptr(geometry_data_ptr)
  {
    init();
  }
  
  BroadPhaseManagerTpl(const BroadPhaseManagerTpl & other)
  : geometry_model_ptr(other.geometry_model_ptr)
  , geometry_data_ptr(other.geometry_data_ptr)
  {
    init();
  }
  
  using Base::update;
  
  ///
  /// @brief Update the manager from the current geometry positions and update the underlying FCL broad phase manager.
  ///
  /// @param[in] compute_local_aabb whether to recompute the local AABB of the collision geometries which have changed.
  ///
  void update(bool compute_local_aabb = true)
  {
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size(); ++i)
    {
      const GeometryObject & geom_obj = geometry_model_ptr->geometryObjects[i];
      hpp::fcl::CollisionGeometryPtr_t new_geometry = geom_obj.geometry;
      
      hpp::fcl::CollisionObject & collision_obj = collision_objects[i];
      hpp::fcl::CollisionGeometryPtr_t geometry = collision_obj.collisionGeometry();
      
      if(new_geometry.get() != geometry.get())
      {
        collision_obj.setCollisionGeometry(new_geometry,compute_local_aabb);
      }
      
      collision_obj.setTransform(toFclTransform3f(geometry_data_ptr->oMg[i]));
    }
    
    assert(check() && "The status of the BroadPhaseManager is not valid");
    
    Base::update(); // because the position has changed.
  }
  
  void update(GeometryData * geom_data_new)
  {
    geometry_data_ptr = geom_data_new;
    update(false);
  }
  
  ~BroadPhaseManagerTpl()
  {
    // Delete geom_data associated to each Collision Geometry.
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size(); ++i)
    {
      hpp::fcl::CollisionObject & collision_obj = collision_objects[i];
      CollisionObjectData * data_ptr = static_cast<CollisionObjectData *>(collision_obj.getUserData());
      delete data_ptr;
      collision_obj.setUserData(nullptr);
    }
  }
  
  /// @brief Check whether the base broad phase manager is aligned with the current collision_objects.
  bool check() const
  {
    std::vector<hpp::fcl::CollisionObject*> collision_objects_ptr = this->getObjects();
    if(collision_objects_ptr.size() != collision_objects.size())
      return false;
    
    for(size_t i = 0; i < collision_objects.size(); ++i)
    {
      const hpp::fcl::CollisionObject & collision_obj = collision_objects[i];

      if(std::find(collision_objects_ptr.begin(), collision_objects_ptr.end(), &collision_obj) == collision_objects_ptr.end())
        return false;
      
      hpp::fcl::CollisionGeometryConstPtr_t geometry = collision_obj.collisionGeometry();
      const GeometryObject & geom_obj = geometry_model_ptr->geometryObjects[i];
      hpp::fcl::CollisionGeometryConstPtr_t geometry_of_geom_obj = geom_obj.geometry;
      
      if(geometry.get() != geometry_of_geom_obj.get())
        return false;
    }
    
    return true;
  }
  
  /// @brief Check whether the callback is inline with *this
  bool check(CollisionCallBackBase * callback) const
  {
    return
       &callback->getGeometryModel() == geometry_model_ptr
    && &callback->getGeometryData() == geometry_data_ptr;
  }
  
//  void setGeometryModel(const GeometryModel & geometry_model)
//  { geometry_model_ptr = &geometry_model; }
  const GeometryModel & getGeometryModel() const { return *geometry_model_ptr; }
  const GeometryData & getGeometryData() const { return *geometry_data_ptr; }
  GeometryData & getGeometryData() { return *geometry_data_ptr; }
  
protected:
  const GeometryModel * geometry_model_ptr;
  GeometryData * geometry_data_ptr;
  
  std::vector<hpp::fcl::CollisionObject> collision_objects;
  
  void init()
  {
    collision_objects.reserve(geometry_model_ptr->geometryObjects.size());
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size() ; ++i)
    {
      GeometryObject & geom_obj = const_cast<GeometryObject &>(geometry_model_ptr->geometryObjects[i]);
      collision_objects.push_back(hpp::fcl::CollisionObject(geom_obj.geometry));
      hpp::fcl::CollisionObject & collision_obj = collision_objects[i];
      
      // Attached user info
      CollisionObjectData * data_ptr = new CollisionObjectData();
      data_ptr->geometry_object_index = i;
      collision_obj.setUserData(data_ptr);
      
      assert(static_cast<CollisionObjectData *>(collision_obj.getUserData())->geometry_object_index == i);
      
      // Feed the base broadphase manager
      Base::registerObject(&collision_obj);
    }
  }
  
}; // struct BroadPhaseManagerTpl<BroadPhaseManagerDerived>

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/multibody/broadphase-manager.hxx"

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_multibody_broadphase_manager_hpp__
