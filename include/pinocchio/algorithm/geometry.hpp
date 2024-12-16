//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifndef __pinocchio_algo_geometry_hpp__
#define __pinocchio_algo_geometry_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio
{

  ///
  /// \brief Apply a forward kinematics and update the placement of the geometry objects.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom_model The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects.
  /// See oMg field in GeometryData. \param[in] q The joint configuration vector (dim model.nq).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  inline void updateGeometryPlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const Eigen::MatrixBase<ConfigVectorType> & q);

  ///
  /// \brief Update the placement of the geometry objects according to the current joint placements
  /// contained in data.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom_model The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects.
  /// See oMg field in GeometryData.
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void updateGeometryPlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data);

  ///
  /// Append geom_model2 to geom_model1
  ///
  /// The steps for appending are:
  /// \li add GeometryObject of geom_model2 to geom_model1,
  /// \li add the collision pairs of geom_model2 into geom_model1 (indexes are updated)
  /// \li add all the collision pairs between geometry objects of geom_model1 and geom_model2.
  /// It is possible to ommit both data (an additional function signature is available which makes
  /// them optional), then inner/outer objects are not updated.
  ///
  /// \param[out] geom_model1   geometry model where the data is added
  /// \param[in]  geom_model2   geometry model from which new geometries are taken
  ///
  /// \note Of course, the geom_data corresponding to geom_model1 will not be valid anymore,
  /// and should be updated (or more simply, re-created from the new setting of geom_model1).
  /// \todo This function is not asserted in unittest.
  ///
  inline void appendGeometryModel(GeometryModel & geom_model1, const GeometryModel & geom_model2);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/geometry.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/geometry.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

// In Pinocchio 2, this header declare computeCollision and computeDistances
#if defined(PINOCCHIO_WITH_HPP_FCL) && defined(PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2)
  #include "pinocchio/collision/collision.hpp"
  #include "pinocchio/collision/distance.hpp"
#endif // PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2

#endif // ifndef __pinocchio_algo_geometry_hpp__
