//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_distance_hpp__
#define __pinocchio_algo_distance_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{

#ifdef PINOCCHIO_WITH_HPP_FCL

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
  fcl::DistanceResult & computeDistance(const GeometryModel & geom_model,
                                        GeometryData & geom_data,
                                        const PairIndex pair_id);
  
  ///
  /// \brief Update the geometry placements and
  ///        calls computeDistance for every active pairs of GeometryData.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model: robot model (const)
  /// \param[out] data: corresponding data (const)
  /// \param[in] geom_model: geometry model (const)
  /// \param[out] geom_data: corresponding geometry data (nonconst) where distances are computed
  ///
  /// \note A similar function is available without model, data and q, not recomputing the FK.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline std::size_t computeDistances(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const GeometryModel & geom_model,
                                      GeometryData & geom_data);


#endif // PINOCCHIO_WITH_HPP_FCL


} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/distance.hxx"

#endif // ifndef __pinocchio_algo_distance_hpp__
