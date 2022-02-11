//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_collision_hpp__
#define __pinocchio_algo_collision_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{

#ifdef PINOCCHIO_WITH_HPP_FCL

  ///
  /// \brief Compute the collision status between a *SINGLE* collision pair.
  /// The result is store in the collisionResults vector.
  ///
  /// \param[in] GeomModel the geometry model (const)
  /// \param[out] GeomData the corresponding geometry data, where computations are done.
  /// \param[in] pair_id The collsion pair index in the GeometryModel.
  ///
  /// \return Return true is the collision objects are colliding.
  /// \note The complete collision result is also available in geom_data.collisionResults[pair_id]
  ///
  bool computeCollision(const GeometryModel & geom_model,
                        GeometryData & geom_data,
                        const PairIndex pair_id);

  ///
  /// \brief Calls computeCollision for every active pairs of GeometryData.
  /// This function assumes that \ref updateGeometryPlacements has been called first.
  ///
  /// \param[in] geom_model: geometry model (const)
  /// \param[out] geom_data: corresponding geometry data (nonconst) where collisions are computed
  /// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
  ///
  /// \warning if stopAtFirstcollision = true, then the collisions vector will
  /// not be entirely fulfilled (of course).
  ///
  bool computeCollisions(const GeometryModel & geom_model,
                         GeometryData & geom_data,
                         const bool stopAtFirstCollision = false);

#endif // PINOCCHIO_WITH_HPP_FCL


} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/collision.hxx"

#endif // ifndef __pinocchio_algo_collision_hpp__
