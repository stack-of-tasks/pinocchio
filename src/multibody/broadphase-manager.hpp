//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_broadphase_manager_hpp__
#define __pinocchio_multibody_broadphase_manager_hpp__

#ifdef PINOCCHIO_WITH_HPP_FCL

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/geometry-object-filter.hpp"

#include "pinocchio/algorithm/broadphase-callbacks.hpp"

namespace pinocchio
{

template<typename _Manager>
struct BroadPhaseManagerTpl
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef std::vector<CollisionObject> CollisionObjectVector;
  typedef Eigen::VectorXd VectorXs;
  typedef _Manager Manager;
  
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
                       GeometryData * geometry_data_ptr,
                       const GeometryObjectFilterBase & filter = GeometryObjectFilterNothing())
  : geometry_model_ptr(geometry_model_ptr)
  , geometry_data_ptr(geometry_data_ptr)
  {
    selected_geometry_objects = filter.apply(geometry_model_ptr->geometryObjects);
    
    geometry_to_collision_index.resize(selected_geometry_objects.size(), (std::numeric_limits<size_t>::max)());
    for(size_t k = 0; k < selected_geometry_objects.size(); ++k)
    {
      geometry_to_collision_index[selected_geometry_objects[k]] = k;
    }
    
    selected_collision_pairs.reserve(geometry_model_ptr->collisionPairs.size());
    for(size_t k = 0; k < geometry_model_ptr->collisionPairs.size(); ++k)
    {
      const CollisionPair & pair = geometry_model_ptr->collisionPairs[k];
      if(   geometry_to_collision_index[pair.first] != (std::numeric_limits<size_t>::max)()
         && geometry_to_collision_index[pair.second] != (std::numeric_limits<size_t>::max)())
      {
        selected_collision_pairs.push_back(k);
      }
      
      selected_collision_pairs.resize(selected_collision_pairs.size());
    }
    
    collision_object_inflation.resize(static_cast<Eigen::DenseIndex>(selected_geometry_objects.size()));
    
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
  , selected_geometry_objects(other.selected_geometry_objects)
  , geometry_to_collision_index(other.geometry_to_collision_index)
  , selected_collision_pairs(other.selected_collision_pairs)
  {
    init();
  }
  
  ///
  /// @brief Update the manager from the current geometry positions and update the underlying FCL broad phase manager.
  ///
  /// @param[in] compute_local_aabb whether to recompute the local AABB of the collision geometries which have changed.
  ///
  void update(bool compute_local_aabb = false);
  
  ///
  /// @brief Update the manager with a new geometry data.
  ///
  /// \param[in] geom_data_ptr_new pointer to the new geometry data.
  ///
  void update(GeometryData * geom_data_ptr_new);
  
  ~BroadPhaseManagerTpl();
  
  /// @brief Check whether the base broad phase manager is aligned with the current collision_objects.
  bool check() const;
  
  /// @brief Check whether the callback is inline with *this
  bool check(CollisionCallBackBase * callback) const;
  
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
  
  /// @brief Performs collision test between one object and all the objects belonging to the manager.
  bool collide(CollisionObject & obj, CollisionCallBackBase * callback) const;
  
  /// @brief Performs collision test for the objects belonging to the manager.
  bool collide(CollisionCallBackBase * callback) const;
  
  /// @brief Performs collision test with objects belonging to another manager.
  bool collide(BroadPhaseManagerTpl & other_manager, CollisionCallBackBase * callback) const;

//  /// @brief Performs distance computation between one object and all the objects belonging to the manager
//  void distance(CollisionObject* obj, DistanceCallBackBase * callback) const;

//  /// @brief Performs distance test for the objects belonging to the manager (i.e., N^2 self distance)
//  void distance(DistanceCallBackBase * callback) const;
  
//  /// @brief Performs distance test with objects belonging to another manager
//  void distance(BroadPhaseCollisionManager* other_manager, DistanceCallBackBase * callback) const;
  
  /// @brief Returns internal manager.
  Manager & getManager() { return manager; }

  /// @brief Returns internal manager.
  const Manager & getManager() const { return manager; }
  
protected:
  
  /// @brief internal manager
  Manager manager;
  
  /// @brief Pointer to the geometry model
  const GeometryModel * geometry_model_ptr;
  
  /// @brief Pointer to the geometry data
  GeometryData * geometry_data_ptr;
  
  /// @brief the vector of collision objects.
  CollisionObjectVector collision_objects;
  
  /// @brief the inflation value related to each collision object.
  VectorXs collision_object_inflation;
  
  /// @brief Selected geometry objects in the original geometry_model.
  std::vector<size_t> selected_geometry_objects;
  
  /// @brief Mapping between a given geometry index and a collision index.
  std::vector<size_t> geometry_to_collision_index;
  
  /// @brief Selected  collision pairs in the original geometry_model.
  std::vector<size_t> selected_collision_pairs;
  
  /// @brief Initialialisation of BroadPhaseManagerTpl
  void init();
  
}; // struct BroadPhaseManagerTpl<BroadPhaseManagerDerived>

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/multibody/broadphase-manager.hxx"

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_multibody_broadphase_manager_hpp__
