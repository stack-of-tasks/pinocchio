//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_collision_distance_hpp__
#define __pinocchio_collision_distance_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/collision/config.hpp"

#include <hpp/fcl/collision_data.h>

namespace pinocchio
{

  ///
  /// Update the geometry placements and
  /// calls computeDistance for every active pairs of GeometryData.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model: robot model (const)
  /// \param[in] data: corresponding data (nonconst) where FK results are stored
  /// \param[in] geom_model: geometry model (const)
  /// \param[out] geom_data: corresponding geometry data (nonconst) where distances are computed
  ///
  /// \note A similar function is available without model, data and q, not recomputing the FK.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  std::size_t computeDistances(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data);

  ///
  /// Compute the forward kinematics, update the geometry placements and
  /// calls computeDistance for every active pairs of GeometryData.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model: robot model (const)
  /// \param[in] data: corresponding data (nonconst) where FK results are stored
  /// \param[in] geom_model: geometry model (const)
  /// \param[out] geom_data: corresponding geometry data (nonconst) where distances are computed
  /// \param[in] q: robot configuration.
  ///
  /// \note A similar function is available without model, data and q, not recomputing the FK.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  std::size_t computeDistances(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const Eigen::MatrixBase<ConfigVectorType> & q);

  ///
  /// \brief Compute the minimal distance between collision objects of a *SINGLE* collison pair
  ///
  /// \param[in] GeomModel the geometry model (const)
  /// \param[out] GeomData the corresponding geometry data, where computations are done.
  /// \param[in] pair_id The index of the collision pair in geom model.
  ///
  /// \return A reference on fcl struct containing the distance result, referring an element
  /// of vector geom_data::distanceResults.
  /// \note The complete distance result is also available in geom_data.distanceResults[pair_id]
  ///
  fcl::DistanceResult & computeDistance(
    const GeometryModel & geom_model, GeometryData & geom_data, const PairIndex pair_id);

  ///
  /// \brief Compute the minimal distance between collision objects of a *ALL* collison pair
  ///
  /// \param[in] GeomModel the geometry model (const)
  /// \param[out] GeomData the corresponding geometry data, where computations are done.
  /// \return Index of the minimal pair distance in geom_data.DistanceResult
  ///
  /// \note The complete distance result is available by pair in geom_data.distanceResults
  ///
  std::size_t computeDistances(const GeometryModel & geom_model, GeometryData & geom_data);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/collision/distance.hxx"
#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/collision/distance.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_collision_distance_hpp__
