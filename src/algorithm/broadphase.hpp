//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algo_broadphase_manager_hpp__
#define __pinocchio_algo_broadphase_manager_hpp__

#ifdef PINOCCHIO_WITH_HPP_FCL

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/broadphase-manager.hpp"

#include "pinocchio/algorithm/broadphase-callbacks.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace pinocchio
{

///
/// \brief Calls computeCollision for every active pairs of GeometryData.
/// This function assumes that \ref updateGeometryPlacements has been called first.
///
/// \param[in] broadphase broadphase instance for collision detection.
/// \param[in] callback callback pointer used for collision detection.
/// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
///
/// \warning if stopAtFirstcollision = true, then the collisions vector will
/// not be entirely fulfilled (of course).
///
template<typename BroadPhaseManagerDerived>
bool computeCollisions(BroadPhaseManagerTpl<BroadPhaseManagerDerived> & broadphase_manager,
                       CollisionCallBackBase * callback)
{
  PINOCCHIO_CHECK_INPUT_ARGUMENT(broadphase_manager.check(callback));
  broadphase_manager.collide(callback);
  return callback->collision;
}

///
/// \brief Calls computeCollision for every active pairs of GeometryData.
/// This function assumes that \ref updateGeometryPlacements has been called first.
///
/// \param[in] broadphase_manager broadphase instance for collision detection.
/// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
///
/// \warning if stopAtFirstcollision = true, then the collisions vector will
/// not be entirely fulfilled (of course).
///
template<typename BroadPhaseManagerDerived>
bool computeCollisions(BroadPhaseManagerTpl<BroadPhaseManagerDerived> & broadphase_manager,
                       const bool stopAtFirstCollision = false)
{
  CollisionCallBackDefault callback(broadphase_manager.getGeometryModel(),
                                    broadphase_manager.getGeometryData(),
                                    stopAtFirstCollision);
  
  return computeCollisions(broadphase_manager, &callback);
}

///
/// Compute the forward kinematics, update the geometry placements and run the collision detection using the broadphase manager.
///
/// \tparam JointCollection Collection of Joint types.
/// \tparam ConfigVectorType Type of the joint configuration vector.
///
/// \param[in] model robot model (const)
/// \param[out] data corresponding data (nonconst) where the forward kinematics results are stored
/// \param[in] broadphase_manager broadphase manager for collision detection.
/// \param[in] callback callback pointer used for collision detection.///
/// \param[in] q robot configuration.
/// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
///
/// \warning if stopAtFirstcollision = true, then the collisions vector will
/// not be entirely fulfilled (of course).
/// \note A similar function is available without model, data and q, not recomputing the forward kinematics.
///
template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename BroadPhaseManagerDerived, typename ConfigVectorType>
inline bool computeCollisions(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              BroadPhaseManagerTpl<BroadPhaseManagerDerived> & broadphase_manager,
                              CollisionCallBackBase * callback,
                              const Eigen::MatrixBase<ConfigVectorType> & q)
{
  updateGeometryPlacements(model, data,
                           broadphase_manager.getGeometryModel(),
                           broadphase_manager.getGeometryData(),
                           q);
  
  broadphase_manager.update(false); // TODO: (jcarpent) - check the false entry
  return computeCollisions(broadphase_manager, &callback);
}

///
/// Compute the forward kinematics, update the geometry placements and run the collision detection using the broadphase manager.
///
/// \tparam JointCollection Collection of Joint types.
/// \tparam ConfigVectorType Type of the joint configuration vector.
///
/// \param[in] model robot model (const)
/// \param[out] data corresponding data (nonconst) where the forward kinematics results are stored
/// \param[in] broadphase_manager broadphase manager for collision detection.
/// \param[in] q robot configuration.
/// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
///
/// \warning if stopAtFirstcollision = true, then the collisions vector will
/// not be entirely fulfilled (of course).
/// \note A similar function is available without model, data and q, not recomputing the forward kinematics.
///
template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename BroadPhaseManagerDerived, typename ConfigVectorType>
inline bool computeCollisions(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              BroadPhaseManagerTpl<BroadPhaseManagerDerived> & broadphase_manager,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const bool stopAtFirstCollision = false)
{
  updateGeometryPlacements(model, data,
                           broadphase_manager.getGeometryModel(),
                           broadphase_manager.getGeometryData(),
                           q);
  
  broadphase_manager.update(true); // TODO: (jcarpent) - check the false entry

  CollisionCallBackDefault callback(broadphase_manager.getGeometryModel(),
                                    broadphase_manager.getGeometryData(),
                                    stopAtFirstCollision);
  return computeCollisions(broadphase_manager, &callback);
  return true;
}

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/broadphase.hxx"

#endif // ifdef PINOCCHIO_WITH_HPP_FCL

#endif // ifndef __pinocchio_algo_broadphase_manager_hpp__
