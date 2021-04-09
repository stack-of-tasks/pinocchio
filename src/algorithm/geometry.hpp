//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_geometry_hpp__
#define __pinocchio_algo_geometry_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/geometry.hpp"

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
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void updateGeometryPlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const GeometryModel & geom_model,
                                       GeometryData & geom_data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q);
  
  ///
  /// \brief Update the placement of the geometry objects according to the current joint placements contained in data.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom_model The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void updateGeometryPlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const GeometryModel & geom_model,
                                       GeometryData & geom_data);

  ///
  /// \brief     Set a mesh scaling vector to each GeometryObject contained in the the GeometryModel.
  ///
  /// param[in]  geom_model The geometry model containing the collision objects.
  /// param[in]  meshScale The scale to be applied to each GeometryObject
  ///
  /// \deprecated This function is now deprecated without replacement.
  ///
  template<typename Vector3Like>
  PINOCCHIO_DEPRECATED
  inline void setGeometryMeshScales(GeometryModel & geom_model, const Eigen::MatrixBase<Vector3Like> & meshScale)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
    for(GeomIndex index=0; index<geom_model.ngeoms; index++)
      geom_model.geometryObjects[index].meshScale = meshScale;
  }

  ///
  /// \brief     Set an isotropic mesh scaling to each GeometryObject contained in the the GeometryModel.
  ///
  /// param[in]  geom_model The geometry model containing the collision objects.
  /// param[in]  meshScale The scale, to be applied to each GeometryObject, equally in all directions
  ///
  /// \deprecated This function is now deprecated without replacement.
  ///
  PINOCCHIO_DEPRECATED
  inline void setGeometryMeshScales(GeometryModel & geom_model, const double meshScale)
  {
    for(GeomIndex index=0; index<geom_model.ngeoms; index++)
      geom_model.geometryObjects[index].meshScale.setConstant(meshScale);
  }

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
  inline bool computeCollisions(const GeometryModel & geom_model,
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
  /// \param[in] stopAtFirstCollision if true, stop the loop over the collision pairs when the first collision is detected.
  ///
  /// \warning if stopAtFirstcollision = true, then the collisions vector will
  /// not be entirely fulfilled (of course).
  /// \note A similar function is available without model, data and q, not recomputing the forward kinematics.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline bool computeCollisions(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const GeometryModel & geom_model,
                                GeometryData & geom_data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const bool stopAtFirstCollision = false);

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
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline std::size_t computeDistances(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const GeometryModel & geom_model,
                                      GeometryData & geom_data,
                                      const Eigen::MatrixBase<ConfigVectorType> & q);
  
  ///
  /// Update the geometry placements and
  /// calls computeDistance for every active pairs of GeometryData.
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

  ///
  /// Compute the radius of the geometry volumes attached to every joints.
  ///
  /// \param[in] model Kinematic model of the system
  /// \param[in] geom_model Geometry model of the system
  /// \param[out] geom_data Geometry data of the system
  ///
  /// \sa GeometryData::radius
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void computeBodyRadius(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const GeometryModel & geom_model,
                                GeometryData & geom_data);
#endif // PINOCCHIO_WITH_HPP_FCL

  ///
  /// Append geom_model2 to geom_model1
  ///
  /// The steps for appending are:
  /// \li add GeometryObject of geom_model2 to geom_model1,
  /// \li add the collision pairs of geom_model2 into geom_model1 (indexes are updated)
  /// \li add all the collision pairs between geometry objects of geom_model1 and geom_model2.
  /// It is possible to ommit both data (an additional function signature is available which makes
  /// them optionnal), then inner/outer objects are not updated.
  ///
  /// \param[out] geom_model1   geometry model where the data is added
  /// \param[in]  geom_model2   geometry model from which new geometries are taken
  ///
  /// \note Of course, the geom_data corresponding to geom_model1 will not be valid anymore,
  /// and should be updated (or more simply, re-created from the new setting of geom_model1).
  /// \todo This function is not asserted in unittest.
  ///
  inline void appendGeometryModel(GeometryModel & geom_model1,
                                  const GeometryModel & geom_model2);

} // namespace pinocchio 

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/geometry.hxx"

#endif // ifndef __pinocchio_algo_geometry_hpp__
