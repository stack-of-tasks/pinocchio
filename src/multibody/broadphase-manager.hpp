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
  
  typedef std::vector<hpp::fcl::CollisionObject> CollisionObjectVector;
  typedef Eigen::VectorXd VectorXs;
  
  /// @brief Default constructor.
  BroadPhaseManagerTpl() // for std::vector
  : geometry_model_ptr(nullptr)
  , geometry_data_ptr(nullptr)
  {}
  
  /// @brief Constructor from a given geometry model and data.
  ///
  /// \param[in] geometry_model_ptr pointer to the geometry model.
  /// \param[in] geometry_data_ptr pointer to the geometry data.
  ///
  BroadPhaseManagerTpl(const GeometryModel * geometry_model_ptr,
                       GeometryData * geometry_data_ptr)
  : geometry_model_ptr(geometry_model_ptr)
  , geometry_data_ptr(geometry_data_ptr)
  , collision_object_inflation(geometry_model_ptr->ngeoms)
  {
    init();
  }
  
  /// @brief Copy contructor.
  ///
  /// \param[in] other manager to copy.
  ///
  BroadPhaseManagerTpl(const BroadPhaseManagerTpl & other)
  : geometry_model_ptr(other.geometry_model_ptr)
  , geometry_data_ptr(other.geometry_data_ptr)
  , collision_object_inflation(other.collision_object_inflation.size())
  {
    init();
  }
  
  using Base::update;
  
  ///
  /// @brief Update the manager from the current geometry positions and update the underlying FCL broad phase manager.
  ///
  /// @param[in] compute_local_aabb whether to recompute the local AABB of the collision geometries which have changed.
  ///
  void update(bool compute_local_aabb)
  {
    assert(geometry_model_ptr->ngeoms == collision_object_inflation.size());
    
    const GeometryModel & geom_model = *geometry_model_ptr;
    GeometryData & geom_data = *geometry_data_ptr;
    collision_object_inflation.setZero();
    
    for(size_t pair_id = 0; pair_id < geom_data.activeCollisionPairs.size(); ++pair_id)
    {
      const CollisionPair & pair = geom_model.collisionPairs[pair_id];
      const GeomIndex geom1_id = pair.first;
      const GeomIndex geom2_id = pair.second;
      
      const bool check_collision =
           geom_data.activeCollisionPairs[pair_id]
      && !(geom_model.geometryObjects[geom1_id].disableCollision || geom_model.geometryObjects[geom2_id].disableCollision);
      
      if(!check_collision) continue;
      
      const ::hpp::fcl::CollisionRequest & cr = geom_data.collisionRequests[pair_id];
      const double inflation = (cr.break_distance + cr.security_margin)*0.5;
      
      collision_object_inflation[geom1_id] = (std::max)(inflation,collision_object_inflation[geom1_id]);
      collision_object_inflation[geom2_id] = (std::max)(inflation,collision_object_inflation[geom2_id]);
    }
    
    for(size_t i = 0; i < geometry_model_ptr->geometryObjects.size(); ++i)
    {
      const GeometryObject & geom_obj = geometry_model_ptr->geometryObjects[i];
      hpp::fcl::CollisionGeometryPtr_t new_geometry = geom_obj.geometry;
      
      hpp::fcl::CollisionObject & collision_obj = collision_objects[i];
      hpp::fcl::CollisionGeometryPtr_t geometry = collision_obj.collisionGeometry();
      
      collision_obj.setTransform(toFclTransform3f(geom_data.oMg[i]));
      
      if(new_geometry.get() != geometry.get())
      {
        collision_obj.setCollisionGeometry(new_geometry,compute_local_aabb);
      }
      else
      {
        collision_obj.computeAABB();
      }
      
      collision_obj.getAABB().expand(collision_object_inflation[i]);
    }
    
    assert(check() && "The status of the BroadPhaseManager is not valid");
    
    Base::update(); // because the position has changed.
  }
  
  ///
  /// @brief Update the manager from the current geometry positions and update the underlying FCL broad phase manager.
  ///
  void update()
  {
    update(false);
  }
  
  ///
  /// @brief Update the manager with a new geometry data.
  ///
  /// \param[in] geom_data_ptr_new pointer to the new geometry data.
  ///
  void update(GeometryData * geom_data_ptr_new)
  {
    geometry_data_ptr = geom_data_ptr_new;
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
  
  /// @brief Returns the geometry model associated to the manager.
  const GeometryModel & getGeometryModel() const { return *geometry_model_ptr; }
  
  /// @brief Returns the geometry data associated to the manager.
  const GeometryData & getGeometryData() const { return *geometry_data_ptr; }
  /// @brief Returns the geometry data associated to the manager.
  GeometryData & getGeometryData() { return *geometry_data_ptr; }
  
  /// @brief Returns the vector of collision objects associated to the manager.
  const CollisionObjectVector & getCollisionObjects() const { return collision_objects; }
  /// @brief Returns the vector of collision objects associated to the manager.
  CollisionObjectVector & getCollisionObjects() { return collision_objects; }
  
  /// @brief Returns the inflation value related to each collision object.
  const VectorXs & getCollisionObjectInflation() { return collision_object_inflation; }
  
protected:
  
  /// @brief Pointer to the geometry model
  const GeometryModel * geometry_model_ptr;
  
  /// @brief Pointer to the geometry data
  GeometryData * geometry_data_ptr;
  
  /// @brief the vector of collision objects.
  CollisionObjectVector collision_objects;
  
  /// @brief the inflation value related to each collision object.
  VectorXs collision_object_inflation;
  
  /// @brief Initialialisation of BroadPhaseManagerTpl
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
