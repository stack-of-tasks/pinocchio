//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_collision_broadphase_manager_base_hpp__
#define __pinocchio_collision_broadphase_manager_base_hpp__

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/collision/broadphase-callbacks.hpp"

namespace pinocchio
{

  template<typename Derived>
  struct BroadPhaseManagerBase
  {
    /// @brief Default constructor
    BroadPhaseManagerBase() // for std::vector
    : model_ptr(nullptr)
    , geometry_model_ptr(nullptr)
    , geometry_data_ptr(nullptr)
    {
    }

    /// @brief Constructor from a given geometry model and geometry data
    BroadPhaseManagerBase(
      const Model * model_ptr,
      const GeometryModel * geometry_model_ptr,
      GeometryData * geometry_data_ptr)
    : model_ptr(model_ptr)
    , geometry_model_ptr(geometry_model_ptr)
    , geometry_data_ptr(geometry_data_ptr)
    {
    }

    /// @brief Copy constructor
    BroadPhaseManagerBase(const BroadPhaseManagerBase & other)
    : model_ptr(other.model_ptr)
    , geometry_model_ptr(other.geometry_model_ptr)
    , geometry_data_ptr(other.geometry_data_ptr)
    {
    }

    BroadPhaseManagerBase &
    operator=(const BroadPhaseManagerBase & other) // Copy assignment operator
    {
      model_ptr = other.model_ptr;
      geometry_model_ptr = other.geometry_model_ptr;
      geometry_data_ptr = other.geometry_data_ptr;
      return *this;
    }

    Derived & derived()
    {
      return static_cast<Derived &>(*this);
    }
    const Derived & derived() const
    {
      return static_cast<const Derived &>(*this);
    }

    /// @brief Check whether the base broad phase manager is aligned with the current
    /// collision_objects.
    bool check() const
    {
      return derived().check();
    }

    /// @brief Check whether the callback is inline with *this
    bool check(CollisionCallBackBase * callback) const
    {
      return derived().check(callback);
    }

    ///
    /// @brief Update the manager from the current geometry positions and update the underlying FCL
    /// broad phase manager.
    ///
    /// @param[in] compute_local_aabb whether to recompute the local AABB of the collision
    /// geometries which have changed.
    ///
    void update(bool compute_local_aabb = false)
    {
      derived().update(compute_local_aabb);
    }

    ///
    /// @brief Update the manager with a new geometry data.
    ///
    /// \param[in] geom_data_ptr_new pointer to the new geometry data.
    ///
    void update(GeometryData * geom_data_ptr_new)
    {
      derived().update(geom_data_ptr_new);
    }

    /// @brief Performs collision test between one object and all the objects belonging to the
    /// manager.
    bool collide(CollisionObject & obj, CollisionCallBackBase * callback) const
    {
      return derived().collide(obj, callback);
    }

    /// @brief Performs collision test for the objects belonging to the manager.
    bool collide(CollisionCallBackBase * callback) const
    {
      return derived().collide(callback);
    }

    /// @brief Performs collision test with objects belonging to another manager.
    bool collide(BroadPhaseManagerBase & other_manager, CollisionCallBackBase * callback) const
    {
      return derived().collide(other_manager.derived(), callback);
    }

    //  /// @brief Performs distance computation between one object and all the objects belonging to
    //  the manager void distance(CollisionObject* obj, DistanceCallBackBase * callback) const;

    //  /// @brief Performs distance test for the objects belonging to the manager (i.e., N^2 self
    //  distance) void distance(DistanceCallBackBase * callback) const;

    //  /// @brief Performs distance test with objects belonging to another manager
    //  void distance(BroadPhaseCollisionManager* other_manager, DistanceCallBackBase * callback)
    //  const;

    /// @brief Returns the model associated to the manager.
    const Model & getModel() const
    {
      return *model_ptr;
    }

    /// @brief Returns the geometry model associated to the manager.
    const GeometryModel & getGeometryModel() const
    {
      return *geometry_model_ptr;
    }

    /// @brief Returns the geometry data associated to the manager.
    const GeometryData & getGeometryData() const
    {
      return *geometry_data_ptr;
    }

    /// @brief Returns the geometry data associated to the manager.
    GeometryData & getGeometryData()
    {
      return *geometry_data_ptr;
    }

  protected:
    /// @brief Pointer to the model
    const Model * model_ptr;

    /// @brief Pointer to the geometry model
    const GeometryModel * geometry_model_ptr;

    /// @brief Pointer to the geometry data
    GeometryData * geometry_data_ptr;

  }; // struct BroadPhaseManagerBase<Derived>

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_broadphase_manager_base_hpp__
