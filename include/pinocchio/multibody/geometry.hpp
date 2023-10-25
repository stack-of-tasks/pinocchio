//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_multibody_geometry_hpp__
#define __pinocchio_multibody_geometry_hpp__

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/serialization/serializable.hpp"

#include <map>
#include <list>
#include <utility>
#include <assert.h>

namespace pinocchio
{
  
  struct GeometryModel
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef double Scalar;
    enum { Options = 0 };
    
    typedef SE3Tpl<Scalar,Options> SE3;
    
    typedef ::pinocchio::GeometryObject GeometryObject;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(GeometryObject) GeometryObjectVector;
    typedef std::vector<CollisionPair> CollisionPairVector;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXb;
    
    typedef pinocchio::GeomIndex GeomIndex;
  
    GeometryModel()
    : ngeoms(0)
    , geometryObjects()
    , collisionPairs()
    {}
    
    ~GeometryModel() {};
    
    /**
     * @brief      Add a geometry object to a GeometryModel and set its parent joint.
     *
     * @param[in]  object     Object
     * @param[in]  model      Corresponding model, used to assert the attributes of object.
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     * @note object is a nonconst copy to ease the insertion code.
     */
    template<typename S2, int O2, template<typename,int> class _JointCollectionTpl>
    GeomIndex addGeometryObject(const GeometryObject & object,
                                const ModelTpl<S2,O2,_JointCollectionTpl> & model);
    
    /**
     * @brief      Add a geometry object to a GeometryModel.
     *
     * @param[in]  object     Object
     *
     * @return     The index of the new added GeometryObject in geometryObjects.
     */
    GeomIndex addGeometryObject(const GeometryObject & object);

    /**
     * @brief     Remove a GeometryObject
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @node Remove also the collision pairs that contain the object.
     */
    void removeGeometryObject(const std::string& name);

    /**
     * @brief      Return the index of a GeometryObject given by its name.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     Index of the corresponding GeometryObject
     */
    GeomIndex getGeometryId(const std::string & name) const;

    
    /**
     * @brief      Check if a GeometryObject  given by its name exists.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     True if the GeometryObject exists in the geometryObjects.
     */
    bool existGeometryName(const std::string & name) const;

    ///
    /// \brief Add a collision pair into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] pair The CollisionPair to add.
    ///
    void addCollisionPair(const CollisionPair & pair);
    
    ///
    /// \brief Add all possible collision pairs.
    ///
    /// \note Collision pairs between geometries having the same parent joint are not added.
    ///
    void addAllCollisionPairs();
    
    ///
    /// \brief Set the collision pairs from a given input array.
    ///        Each entry of the input matrix defines the activation of a given collision pair
    ///        (map[i,j] == true means that the pair (i,j) is active).
    ///
    /// \param[in] collision_map Associative array.
    /// \param[in] upper Wheter the collision_map is an upper or lower triangular filled array. 
    ///
    void setCollisionPairs(const MatrixXb & collision_map,
                           const bool upper = true);
   
    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair(const CollisionPair& pair);
    
    ///
    /// \brief Remove all collision pairs from collisionPairs. Same as collisionPairs.clear().
    ///
    void removeAllCollisionPairs();
   
    ///
    /// \brief Check if a collision pair exists in collisionPairs.
    ///        See also findCollisitionPair(const CollisionPair & pair).
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair(const CollisionPair & pair) const;
    
    ///
    /// \brief Return the index of a given collision pair in collisionPairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collisionPairs.
    ///
    PairIndex findCollisionPair(const CollisionPair & pair) const;

    ///
    /// \brief Returns true if *this and other are equal.
    ///
    bool operator==(const GeometryModel & other) const
    {
      return
         ngeoms == other.ngeoms
      && geometryObjects == other.geometryObjects
      && collisionPairs == other.collisionPairs
      ;
    }
    
    ///
    /// \brief Returns true if *this and other are not equal.
    ///
    bool operator!=(const GeometryModel & other) const
    {
      return !(*this == other);
    }

    friend std::ostream& operator<<(std::ostream & os,
                                    const GeometryModel & model_geom);
    
    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    GeometryObjectVector geometryObjects;
    
    /// \brief Vector of collision pairs.
    CollisionPairVector collisionPairs;
    
  }; // struct GeometryModel

  struct GeometryData
  : serialization::Serializable<GeometryData>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef double Scalar;
    enum { Options = 0 };
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef std::vector<GeomIndex> GeomIndexList;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXb;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    typedef ::pinocchio::ComputeCollision ComputeCollision;
    typedef ::pinocchio::ComputeDistance ComputeDistance;
#endif
    
    ///
    /// \brief Vector gathering the SE3 placements of the geometry objects relative to the world.
    ///        See updateGeometryPlacements to update the placements.
    ///
    /// oMg is used for pinocchio (kinematics) computation but is translated to fcl type
    /// for fcl (collision) computation. The copy is done in collisionObjects[i]->setTransform(.)
    ///
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) oMg;

    ///
    /// \brief Vector of collision pairs.
    ///
    std::vector<bool> activeCollisionPairs;

#ifdef PINOCCHIO_WITH_HPP_FCL

    ///
    /// \brief Defines what information should be computed by distance computation.
    /// There is one request per pair of geometries.
    std::vector<fcl::DistanceRequest> distanceRequests;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector<fcl::DistanceResult> distanceResults;
    
    ///
    /// \brief Defines what information should be computed by collision test.
    /// There is one request per pair of geometries.
    std::vector<fcl::CollisionRequest> collisionRequests;

    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector<fcl::CollisionResult> collisionResults;

    ///
    /// \brief Radius of the bodies, i.e. distance of the further point of the geometry model
    /// attached to the body from the joint center.
    ///
    std::vector<Scalar> radius;

    ///
    /// \brief Index of the collision pair
    ///
    /// It is used by some method to return additional information. For instance,
    /// the algo computeCollisions() sets it to the first colliding pair.
    ///
    PairIndex collisionPairIndex;
    
    /// \brief Functor associated to the computation of collisions.
    PINOCCHIO_ALIGNED_STD_VECTOR(ComputeCollision) collision_functors;
    
    /// \brief Functor associated to the computation of distances.
    PINOCCHIO_ALIGNED_STD_VECTOR(ComputeDistance) distance_functors;
    
#endif // PINOCCHIO_WITH_HPP_FCL

    /// \brief Map over vector GeomModel::geometryObjects, indexed by joints.
    ///
    /// The map lists the collision GeometryObjects associated to a given joint Id.
    ///  Inner objects can be seen as geometry objects that directly move when the associated joint moves
    std::map<JointIndex,GeomIndexList>  innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///
    /// Outer objects can be seen as geometry objects that may often be
    /// obstacles to the Inner objects of a given joint
    std::map<JointIndex,GeomIndexList>  outerObjects;

    ///
    /// \brief Default constructor from a GeometryModel
    ///
    /// \param[in] geom_model GeometryModel associated to the new GeometryData
    ///
    explicit GeometryData(const GeometryModel & geom_model);
   
    ///
    /// \brief Copy constructor
    ///
    /// \param[in] other GeometryData to copy
    ///
    GeometryData(const GeometryData & other);

    ///
    /// \brief Copy operator
    ///
    /// \param[in] other GeometryData to copy
    ///
    GeometryData& operator=(const GeometryData & other);
    
    /// \brief Empty constructor
    GeometryData() {};
    
    /// \brief Destructor
    ~GeometryData();

    /// Fill both innerObjects and outerObjects maps, from vectors collisionObjects and 
    /// collisionPairs. 
    ///
    /// This simply corresponds to storing in a re-arranged manner the information stored
    /// in geomModel.geometryObjects and geomModel.collisionPairs.
    /// \param[in] geomModel the geometry model (const)
    ///
    /// \warning Outer objects are not duplicated (i.e. if a is in outerObjects[b], then
    /// b is not in outerObjects[a]).
    void fillInnerOuterObjectMaps(const GeometryModel & geomModel);
    
    ///
    /// Activate a collision pair, for which collisions and distances would now be computed.
    ///
    /// The collision (resp distance) between two geometries of GeomModel::geometryObjects
    /// is computed *iff* the corresponding pair has been added in GeomModel::collisionPairs *AND*
    /// it is active, i.e. the corresponding boolean in GeomData::activePairs is true. The second
    /// condition can be used to temporarily remove a pair without touching the model, in a versatile
    /// manner.
    ///
    /// \param[in] pair_id the index of the pair in GeomModel::collisionPairs vector.
    ///
    /// \sa GeomData
    ///
    void activateCollisionPair(const PairIndex pair_id);
    
    ///
    /// \brief Activate all collision pairs.
    ///
    /// \sa GeomData::deactivateAllCollisionPairs, GeomData::activateCollisionPair, GeomData::deactivateCollisionPair
    ///
    void activateAllCollisionPairs();
    
    ///
    /// \brief Set the collision pair association from a given input array.
    ///        Each entry of the input matrix defines the activation of a given collision pair.
    ///
    /// \param[in] geom_model Geometry model associated to the data.
    /// \param[in] collision_map Associative array.
    /// \param[in] upper Wheter the collision_map is an upper or lower triangular filled array.
    ///
    void setActiveCollisionPairs(const GeometryModel & geom_model,
                                 const MatrixXb & collision_map,
                                 const bool upper = true);
    
    ///
    /// \brief Enable or disable collision for the given geometry given by its geometry id with all the other geometries registered in the list of collision pairs.
    ///
    /// \param[in] geom_model Geometry model associated to the data.
    /// \param[in] geom_id Index of the geometry.
    /// \param[in] enable_collision If true, the collision will be enable, otherwise disable.
    ///
    void setGeometryCollisionStatus(const GeometryModel & geom_model,
                                    const GeomIndex geom_id,
                                    bool enable_collision);

    ///
    /// Deactivate a collision pair.
    ///
    /// Calls indeed GeomData::activateCollisionPair(pair_id)
    ///
    /// \param[in] pair_id the index of the pair in GeomModel::collisionPairs vector.
    ///
    /// \sa GeomData::activateCollisionPair
    ///
    void deactivateCollisionPair(const PairIndex pair_id);
    
    ///
    /// \brief Deactivate all collision pairs.
    ///
    /// \sa GeomData::activateAllCollisionPairs, GeomData::activateCollisionPair, GeomData::deactivateCollisionPair
    ///
    void deactivateAllCollisionPairs();
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    ///
    /// \brief Set the security margin of all the collision request in a row, according to the values stored in the associative map.
    ///
    /// \param[in] geom_model Geometry model associated to the data.
    /// \param[in] security_margin_map Associative map related the security margin of a given input collision pair (i,j).
    /// \param[in] upper Wheter the security_margin_map is an upper or lower triangular filled array.
    ///
    void setSecurityMargins(const GeometryModel & geom_model,
                            const MatrixXs & security_margin_map,
                            const bool upper = true);
#endif // ifdef PINOCCHIO_WITH_HPP_FCL

    friend std::ostream & operator<<(std::ostream & os, const GeometryData & geomData);
    
    ///
    /// \brief Returns true if *this and other are equal.
    ///
    bool operator==(const GeometryData & other) const
    {
      return
         oMg                  == other.oMg
      && activeCollisionPairs == other.activeCollisionPairs
#ifdef PINOCCHIO_WITH_HPP_FCL
      && distanceRequests     == other.distanceRequests
      && distanceResults      == other.distanceResults
      && collisionRequests    == other.collisionRequests
      && collisionResults     == other.collisionResults
      && radius               == other.radius
      && collisionPairIndex   == other.collisionPairIndex
#endif
      && innerObjects         == other.innerObjects
      && outerObjects         == other.outerObjects
      ;
    }
    
    ///
    /// \brief Returns true if *this and other are not equal.
    ///
    bool operator!=(const GeometryData & other) const
    {
      return !(*this == other);
    }
    
  }; // struct GeometryData

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry.hxx"

#endif // ifndef __pinocchio_multibody_geometry_hpp__
