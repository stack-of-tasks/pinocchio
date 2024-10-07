//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_collision_collision_hpp__
#define __pinocchio_collision_collision_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/collision/config.hpp"

#include <hpp/fcl/collision_data.h>

namespace pinocchio
{

  ///
  /// \brief Compute the collision status between a *SINGLE* collision pair.
  /// The result is store in the collisionResults vector.
  ///
  /// \param[in] GeomModel the geometry model (const)
  /// \param[out] GeomData the corresponding geometry data, where computations are done.
  /// \param[in] pair_id The collsion pair index in the GeometryModel.
  /// \param[in] collision_request The collision request associated to the collision pair.
  ///
  /// \return Return true is the collision objects are colliding.
  /// \note The complete collision result is also available in geom_data.collisionResults[pair_id]
  ///
  bool computeCollision(
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const PairIndex pair_id,
    fcl::CollisionRequest & collision_request);

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
  bool computeCollision(
    const GeometryModel & geom_model, GeometryData & geom_data, const PairIndex pair_id);

  ///
  /// \brief Calls computeCollision for every active pairs of GeometryData.
  /// This function assumes that \ref updateGeometryPlacements has been called first.
  ///
  /// \param[in] geom_model: geometry model (const)
  /// \param[out] geom_data: corresponding geometry data (nonconst) where collisions are computed
  /// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first
  /// collision is detected.
  ///
  /// \warning if stopAtFirstcollision = true, then the collisions vector will
  /// not be entirely fulfilled (of course).
  ///
  bool computeCollisions(
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const bool stopAtFirstCollision = false);

  ///
  /// Compute the forward kinematics, update the geometry placements and
  /// calls computeCollision for every active pairs of GeometryData.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model robot model (const)
  /// \param[out] data corresponding data (nonconst) where the forward kinematics results are stored
  /// \param[in] geom_model geometry model (const)
  /// \param[out] geom_data corresponding geometry data (nonconst) where distances are computed
  /// \param[in] q robot configuration.
  /// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first
  /// collision is detected.
  ///
  /// \warning if stopAtFirstcollision = true, then the collisions vector will
  /// not be entirely fulfilled (of course).
  /// \note A similar function is available without model, data and q, not recomputing the forward
  /// kinematics.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  bool computeCollisions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const bool stopAtFirstCollision = false);

  ///
  /// Compute the radius of the geometry volumes attached to every joints.
  ///
  /// \param[in] model Kinematic model of the system
  /// \param[in] geom_model Geometry model of the system
  /// \param[out] geom_data Geometry data of the system
  ///
  /// \sa GeometryData::radius
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void computeBodyRadius(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const GeometryModel & geom_model,
    GeometryData & geom_data);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/collision/collision.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/collision/collision.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_collision_collision_hpp__
