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
  void update(bool compute_local_aabb);
  
  ///
  /// @brief Update the manager from the current geometry positions and update the underlying FCL broad phase manager.
  ///
  void update();
  
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
  void init();
  
}; // struct BroadPhaseManagerTpl<BroadPhaseManagerDerived>

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/multibody/broadphase-manager.hxx"

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_multibody_broadphase_manager_hpp__
