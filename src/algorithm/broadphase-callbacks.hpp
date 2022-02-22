//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algo_broadphase_callback_hpp__
#define __pinocchio_algo_broadphase_callback_hpp__

#ifdef PINOCCHIO_WITH_HPP_FCL

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <iostream>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/algorithm/collision.hpp"

namespace pinocchio
{

/// @brief Interface for Pinocchio collision callback functors
struct CollisionCallBackBase : hpp::fcl::CollisionCallBackBase
{
  CollisionCallBackBase(const GeometryModel & geometry_model,
                        GeometryData & geometry_data)
  : geometry_model_ptr(&geometry_model)
  , geometry_data_ptr(&geometry_data)
  , collision(false)
  {}
  
  const GeometryModel & getGeometryModel() const { return *geometry_model_ptr; }
  const GeometryData & getGeometryData() const { return *geometry_data_ptr; }
  GeometryData & getGeometryData() { return *geometry_data_ptr; }

protected:
  /// @brief Geometry model associated to the callback
  const GeometryModel * geometry_model_ptr;
  
  /// @brief Geometry data associated to the callback
  GeometryData * geometry_data_ptr;
  
public:
  
  /// @brief Whether there is a collision or not
  bool collision;
  
};

struct CollisionCallBackDefault : CollisionCallBackBase
{
  CollisionCallBackDefault(const GeometryModel & geometry_model,
                           GeometryData & geometry_data,
                           bool stopAtFirstCollision = false)
  : CollisionCallBackBase(geometry_model, geometry_data)
  , stopAtFirstCollision(stopAtFirstCollision)
  , count(0)
//  , visited(Eigen::MatrixXd::Zero(geometry_model.ngeoms,geometry_model.ngeoms))
  {}
  
  void init()
  {
    count = 0;
    collision = false;
    collisionPairIndex = std::numeric_limits<PairIndex>::max();
//    visited.setZero();
  }
  
  bool collide(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2)
  {
    CollisionObject & co1 = reinterpret_cast<CollisionObject&>(*o1);
    CollisionObject & co2 = reinterpret_cast<CollisionObject&>(*o2);
    
    const Eigen::DenseIndex go1_index = (Eigen::DenseIndex)co1.geometryObjectIndex;
    const Eigen::DenseIndex go2_index = (Eigen::DenseIndex)co2.geometryObjectIndex;
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(go1_index < (Eigen::DenseIndex)geometry_model_ptr->ngeoms && go1_index >= 0);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(go2_index < (Eigen::DenseIndex)geometry_model_ptr->ngeoms && go2_index >= 0);

    const int pair_index = geometry_model_ptr->collisionPairMapping(go1_index,go2_index);
    if(pair_index == -1)
      return false;
    count++;
    
    const bool res = computeCollision(*geometry_model_ptr, *geometry_data_ptr, (PairIndex)pair_index);
    
    if(res && !collision)
    {
      collision = true; collisionPairIndex = (PairIndex)pair_index;
    }
    
    if(!stopAtFirstCollision)
      return false;
    else
      return res;
  }

  /// @brief Whether to stop or not when localizing a first collision
  bool stopAtFirstCollision;
  
  /// @brief The collision index of the first pair in collision
  PairIndex collisionPairIndex;
  
  /// @brief Number of visits of the collide method
  size_t count;
  
//  Eigen::MatrixXd visited;
};

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/broadphase-callbacks.hxx"

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_algo_broadphase_callback_hpp__
