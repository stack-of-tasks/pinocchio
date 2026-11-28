//
// Copyright (c) 2015-2023 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/geometry.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/geometry.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  struct CollisionPair : public std::pair<GeomIndex, GeomIndex>
  {

    typedef std::pair<GeomIndex, GeomIndex> Base;

    /// \brief Empty constructor
    CollisionPair();

    ///
    /// \brief Default constructor of a collision pair from two collision object indexes.
    /// \remarks The two indexes must be different, otherwise the constructor throws.
    ///
    /// \param[in] co1 Index of the first collision object.
    /// \param[in] co2 Index of the second collision object.
    ///
    CollisionPair(const GeomIndex co1, const GeomIndex co2);
    bool operator==(const CollisionPair & rhs) const;
    bool operator!=(const CollisionPair & rhs) const;
    void disp(std::ostream & os) const;
    friend std::ostream & operator<<(std::ostream & os, const CollisionPair & X);

  }; // struct CollisionPair

  template<>
  struct traits<GeometryModel>
  {
    typedef double Scalar;
    static constexpr int Options = 0;
  };

  struct GeometryModel
  : NumericalBase<GeometryModel>
  , serialization::Serializable<GeometryModel>
  {

    typedef typename traits<GeometryModel>::Scalar Scalar;
    static constexpr int Options = traits<GeometryModel>::Options;

    typedef SE3Tpl<Scalar, Options> SE3;

    typedef ::pinocchio::GeometryObject GeometryObject;
    typedef std::vector<GeometryObject> GeometryObjectVector;
    typedef std::vector<CollisionPair> CollisionPairVector;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXb;
    typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXi;

    typedef pinocchio::GeomIndex GeomIndex;

    GeometryModel()
    : ngeoms(0)
    , geometryObjects()
    , collisionPairs()
    {
    }

    GeometryModel(const GeometryModel & other) = default;

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
    template<typename S2, int O2, template<typename, int> class _JointCollectionTpl>
    GeomIndex addGeometryObject(
      const GeometryObject & object, const ModelTpl<S2, O2, _JointCollectionTpl> & model);

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
    void removeGeometryObject(const std::string & name);

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
    void setCollisionPairs(const MatrixXb & collision_map, const bool upper = true);

    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair(const CollisionPair & pair);

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
    /// \brief Create a deep copy of *this.
    ///
    GeometryModel clone() const;

    ///
    /// \brief Returns true if *this and other are equal.
    ///
    bool operator==(const GeometryModel & other) const
    {
      return ngeoms == other.ngeoms && geometryObjects == other.geometryObjects
             && collisionPairs == other.collisionPairs
             && collisionPairMapping == other.collisionPairMapping;
    }

    ///
    /// \brief Returns true if *this and other are not equal.
    ///
    bool operator!=(const GeometryModel & other) const
    {
      return !(*this == other);
    }

    friend std::ostream & operator<<(std::ostream & os, const GeometryModel & model_geom);

    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    GeometryObjectVector geometryObjects;

    /// \brief Vector of collision pairs.
    CollisionPairVector collisionPairs;

    /// \brief Matrix relating the collision pair ID to a pair of two GeometryObject indexes
    MatrixXi collisionPairMapping;

  }; // struct GeometryModel

  template<>
  struct traits<GeometryData>
  {
    typedef double Scalar;
    static constexpr int Options = 0;
  };

  struct GeometryData
  : NumericalBase<GeometryData>
  , serialization::Serializable<GeometryData>
  {

    typedef typename traits<GeometryData>::Scalar Scalar;
    static constexpr int Options = traits<GeometryData>::Options;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef std::vector<GeomIndex> GeomIndexList;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXb;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;

#ifdef PINOCCHIO_WITH_COLLISION
    typedef ::pinocchio::ComputeCollision ComputeCollision;
    typedef ::pinocchio::ComputeContactPatch ComputeContactPatch;
    typedef ::pinocchio::ComputeDistance ComputeDistance;
#endif

    ///
    /// \brief Vector gathering the SE3 placements of the geometry objects relative to the world.
    ///        See updateGeometryPlacements to update the placements.
    ///
    /// oMg is used for pinocchio (kinematics) computation but is translated to coal type
    /// for coal (collision) computation. The copy is done in collisionObjects[i]->setTransform(.)
    ///
    std::vector<SE3> oMg;

    ///
    /// \brief Vector of collision pairs.
    ///
    std::vector<bool> activeCollisionPairs;

#ifdef PINOCCHIO_WITH_COLLISION

    ///
    /// \brief Defines what information should be computed by distance computation.
    /// There is one request per pair of geometries.
    std::vector<coal::DistanceRequest> distanceRequests;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector<coal::DistanceResult> distanceResults;

    ///
    /// \brief Defines what information should be computed by collision test.
    /// There is one request per pair of geometries.
    std::vector<coal::CollisionRequest> collisionRequests;

    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector<coal::CollisionResult> collisionResults;

    ///
    /// \brief Defines what information should be computed by contact patch test.
    /// There is one request per pair of geometries.
    std::vector<coal::ContactPatchRequest> contactPatchRequests;

    ///
    /// \brief Vector gathering the result of the contact patch computation for all the collision
    /// pairs.
    ///
    std::vector<coal::ContactPatchResult> contactPatchResults;

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

    ///  \brief Functor associated to the computation of collisions.
    std::vector<ComputeCollision> collision_functors;

    ///  \brief Functor associated to the computation of contact patches.
    std::vector<ComputeContactPatch> contact_patch_functors;

    ///  \brief Functor associated to the computation of distances.
    std::vector<ComputeDistance> distance_functors;

#endif // PINOCCHIO_WITH_COLLISION

    /// \brief Map over vector GeomModel::geometryObjects, indexed by joints.
    ///
    /// The map lists the collision GeometryObjects associated to a given joint Id.
    ///  Inner objects can be seen as geometry objects that directly move when the associated joint
    ///  moves
    std::map<JointIndex, GeomIndexList> innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///
    /// Outer objects can be seen as geometry objects that may often be
    /// obstacles to the Inner objects of a given joint
    std::map<JointIndex, GeomIndexList> outerObjects;

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
    GeometryData & operator=(const GeometryData & other);

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
    /// condition can be used to temporarily remove a pair without touching the model, in a
    /// versatile manner.
    ///
    /// \param[in] pair_id the index of the pair in GeomModel::collisionPairs vector.
    ///
    ///  \sa GeomData
    ///
    void activateCollisionPair(const PairIndex pair_id);

    ///
    /// \brief Activate all collision pairs.
    ///
    /// \sa GeomData::deactivateAllCollisionPairs, GeomData::activateCollisionPair,
    /// GeomData::deactivateCollisionPair
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
    void setActiveCollisionPairs(
      const GeometryModel & geom_model, const MatrixXb & collision_map, const bool upper = true);

    ///
    /// \brief Enable or disable collision for the given geometry given by its geometry id with all
    /// the other geometries registered in the list of collision pairs.
    ///
    /// \param[in] geom_model Geometry model associated to the data.
    /// \param[in] geom_id Index of the geometry.
    /// \param[in] enable_collision If true, the collision will be enable, otherwise disable.
    ///
    void setGeometryCollisionStatus(
      const GeometryModel & geom_model, const GeomIndex geom_id, bool enable_collision);

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
    /// \sa GeomData::activateAllCollisionPairs, GeomData::activateCollisionPair,
    /// GeomData::deactivateCollisionPair
    ///
    void deactivateAllCollisionPairs();

#ifdef PINOCCHIO_WITH_COLLISION
    ///
    /// \brief Set the security margin of all the collision request in a row, according to the
    /// values stored in the associative map.
    ///
    /// \param[in] geom_model Geometry model associated to the data.
    /// \param[in] security_margin_map Associative map related the security margin of a given input
    /// collision pair (i,j). \param[in] upper Wheter the security_margin_map is an upper or lower
    /// triangular filled array. \param[in] sync_distance_upper_bound Wheter distance_upper_bound
    /// have fields to be updated with the security margin value.
    ///
    void setSecurityMargins(
      const GeometryModel & geom_model,
      const MatrixXs & security_margin_map,
      const bool upper = true,
      const bool sync_distance_upper_bound = false);

#endif // ifdef PINOCCHIO_WITH_COLLISION

    friend std::ostream & operator<<(std::ostream & os, const GeometryData & geomData);

    ///
    /// \brief Returns true if *this and other are equal.
    ///
    bool operator==(const GeometryData & other) const
    {
      return oMg == other.oMg && activeCollisionPairs == other.activeCollisionPairs
#ifdef PINOCCHIO_WITH_COLLISION
             && distanceRequests == other.distanceRequests
             && distanceResults == other.distanceResults
             && collisionRequests == other.collisionRequests
             && collisionResults == other.collisionResults
             && contactPatchRequests == other.contactPatchRequests
             && contactPatchResults == other.contactPatchResults && radius == other.radius
             && collisionPairIndex == other.collisionPairIndex
#endif
             && innerObjects == other.innerObjects && outerObjects == other.outerObjects;
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

namespace pinocchio
{

  inline CollisionPair::CollisionPair()
  : Base((std::numeric_limits<GeomIndex>::max)(), (std::numeric_limits<GeomIndex>::max)())
  {
  }

  ///
  /// \brief Default constructor of a collision pair from two collision object indexes.
  /// \remarks The two indexes must be different, otherwise the constructor throws.
  ///
  /// \param[in] co1 Index of the first collision object.
  /// \param[in] co2 Index of the second collision object.
  ///
  inline CollisionPair::CollisionPair(const GeomIndex co1, const GeomIndex co2)
  : Base(co1, co2)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(co1 != co2, "The index of collision objects must not be equal.");
  }

  inline bool CollisionPair::operator==(const CollisionPair & rhs) const
  {
    return (first == rhs.first && second == rhs.second)
           || (first == rhs.second && second == rhs.first);
  }

  inline bool CollisionPair::operator!=(const CollisionPair & other) const
  {
    return !(*this == other);
  }

  inline void CollisionPair::disp(std::ostream & os) const
  {
    os << "collision pair (" << first << "," << second << ")\n";
  }

  inline std::ostream & operator<<(std::ostream & os, const CollisionPair & X)
  {
    X.disp(os);
    return os;
  }

  inline GeometryModel GeometryModel::clone() const
  {
    GeometryModel res;
    res.ngeoms = ngeoms;
    res.collisionPairs = collisionPairs;
    res.collisionPairMapping = collisionPairMapping;

    res.geometryObjects.reserve(geometryObjects.size());
    for (const GeometryObject & geometry_object : geometryObjects)
    {
      res.geometryObjects.push_back(geometry_object.clone());
    }

    return res;
  }

  inline GeometryData::GeometryData(const GeometryModel & geom_model)
  : oMg(geom_model.ngeoms)
  , activeCollisionPairs(geom_model.collisionPairs.size(), true)
#ifdef PINOCCHIO_WITH_COLLISION
  , distanceRequests(geom_model.collisionPairs.size(), coal::DistanceRequest(true))
  , distanceResults(geom_model.collisionPairs.size())
  , collisionRequests(
      geom_model.collisionPairs.size(), coal::CollisionRequest(::coal::NO_REQUEST, 1))
  , collisionResults(geom_model.collisionPairs.size())
  , contactPatchRequests(geom_model.collisionPairs.size()) // use default constructor
  , contactPatchResults(geom_model.collisionPairs.size())  // use default constructor
  , radius()
  , collisionPairIndex(0)
#endif // PINOCCHIO_WITH_COLLISION
  , innerObjects()
  , outerObjects()
  {
#ifdef PINOCCHIO_WITH_COLLISION
    BOOST_FOREACH (coal::CollisionRequest & creq, collisionRequests)
    {
      creq.enable_cached_gjk_guess = true;
    }
    BOOST_FOREACH (coal::DistanceRequest & dreq, distanceRequests)
    {
      dreq.enable_cached_gjk_guess = true;
    }
    collision_functors.reserve(geom_model.collisionPairs.size());
    contact_patch_functors.reserve(geom_model.collisionPairs.size());
    distance_functors.reserve(geom_model.collisionPairs.size());

    for (size_t cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
      const CollisionPair & cp = geom_model.collisionPairs[cp_index];
      const GeometryObject & obj_1 = geom_model.geometryObjects[cp.first];
      const GeometryObject & obj_2 = geom_model.geometryObjects[cp.second];

      collision_functors.push_back(ComputeCollision(obj_1, obj_2));
      contact_patch_functors.push_back(ComputeContactPatch(obj_1, obj_2));
      distance_functors.push_back(ComputeDistance(obj_1, obj_2));
    }
#endif
    fillInnerOuterObjectMaps(geom_model);
  }

  inline GeometryData::GeometryData(const GeometryData & other)
  : oMg(other.oMg)
  , activeCollisionPairs(other.activeCollisionPairs)
#ifdef PINOCCHIO_WITH_COLLISION
  , distanceRequests(other.distanceRequests)
  , distanceResults(other.distanceResults)
  , collisionRequests(other.collisionRequests)
  , collisionResults(other.collisionResults)
  , contactPatchRequests(other.contactPatchRequests)
  , contactPatchResults(other.contactPatchResults)
  , radius(other.radius)
  , collisionPairIndex(other.collisionPairIndex)
  , collision_functors(other.collision_functors)
  , contact_patch_functors(other.contact_patch_functors)
  , distance_functors(other.distance_functors)
#endif // PINOCCHIO_WITH_COLLISION
  , innerObjects(other.innerObjects)
  , outerObjects(other.outerObjects)
  {
  }

  inline GeometryData & GeometryData::operator=(const GeometryData & other)
  {
    if (this != &other)
    {
      oMg = other.oMg;
      activeCollisionPairs = other.activeCollisionPairs;
#ifdef PINOCCHIO_WITH_COLLISION
      distanceRequests = other.distanceRequests;
      distanceResults = other.distanceResults;
      collisionRequests = other.collisionRequests;
      collisionResults = other.collisionResults;
      contactPatchRequests = other.contactPatchRequests;
      contactPatchResults = other.contactPatchResults;
      radius = other.radius;
      collisionPairIndex = other.collisionPairIndex;
      collision_functors = other.collision_functors;
      contact_patch_functors = other.contact_patch_functors;
      distance_functors = other.distance_functors;
#endif // PINOCCHIO_WITH_COLLISION
      innerObjects = other.innerObjects;
      outerObjects = other.outerObjects;
    }
    return *this;
  }

  inline GeometryData::~GeometryData()
  {
  }

  template<typename S2, int O2, template<typename, int> class JointCollectionTpl>
  GeomIndex GeometryModel::addGeometryObject(
    const GeometryObject & object, const ModelTpl<S2, O2, JointCollectionTpl> & model)
  {
    if (object.parentFrame < (FrameIndex)model.nframes)
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        model.frames[object.parentFrame].parentJoint == object.parentJoint,
        "The object joint parent and its frame joint parent do not match.");

    Eigen::Index idx = (Eigen::Index)(ngeoms++);
    geometryObjects.push_back(object);
    geometryObjects.back().parentJoint = model.frames[object.parentFrame].parentJoint;

    collisionPairMapping.conservativeResize(idx + 1, idx + 1);
    collisionPairMapping.col(idx).fill(-1);
    collisionPairMapping.row(idx).fill(-1);
    assert(collisionPairMapping.cols() == (Eigen::Index)ngeoms);

    return (GeomIndex)idx;
  }

  inline GeomIndex GeometryModel::addGeometryObject(const GeometryObject & object)
  {
    Eigen::Index idx = (Eigen::Index)(ngeoms++);
    geometryObjects.push_back(object);

    collisionPairMapping.conservativeResize(idx + 1, idx + 1);
    collisionPairMapping.col(idx).fill(-1);
    collisionPairMapping.row(idx).fill(-1);
    assert(collisionPairMapping.cols() == (Eigen::Index)ngeoms);

    return (GeomIndex)idx;
  }

  inline void GeometryModel::removeGeometryObject(const std::string & name)
  {
    GeomIndex i = 0;
    GeometryObjectVector::iterator it;
    for (it = geometryObjects.begin(); it != geometryObjects.end(); ++it, ++i)
    {
      if (it->name == name)
      {
        break;
      }
    }
    PINOCCHIO_THROW_IF(
      (it == geometryObjects.end()), std::invalid_argument,
      (std::string("Object ") + name + std::string(" does not belong to model")).c_str());

    // Sized for the object about to be removed, and filled in lockstep with collisionPairs
    // below so surviving pairs land at their post-removal index without a second pass.
    MatrixXi new_collision_pair_mapping =
      MatrixXi::Constant((Eigen::Index)ngeoms - 1, (Eigen::Index)ngeoms - 1, -1);
    int new_pair_index = 0;

    // Remove all collision pairs that contain i as first or second index,
    for (CollisionPairVector::iterator itCol = collisionPairs.begin();
         itCol != collisionPairs.end(); ++itCol)
    {
      if ((itCol->first == i) || (itCol->second == i))
      {
        itCol = collisionPairs.erase(itCol);
        itCol--;
      }
      else
      {
        // Indices of objects after the one that is removed should be decreased by one.
        if (itCol->first > i)
          itCol->first--;
        if (itCol->second > i)
          itCol->second--;

        new_collision_pair_mapping((Eigen::Index)itCol->second, (Eigen::Index)itCol->first) =
          new_collision_pair_mapping((Eigen::Index)itCol->first, (Eigen::Index)itCol->second) =
            new_pair_index++;
      }
    }
    collisionPairMapping = new_collision_pair_mapping;

    geometryObjects.erase(it);
    ngeoms--;
  }

  inline GeomIndex GeometryModel::getGeometryId(const std::string & name) const
  {
    using namespace boost::placeholders;
    GeometryObjectVector::const_iterator it = std::find_if(
      geometryObjects.begin(), geometryObjects.end(),
      boost::bind(&GeometryObject::name, _1) == name);
    return GeomIndex(it - geometryObjects.begin());
  }

  inline bool GeometryModel::existGeometryName(const std::string & name) const
  {
    using namespace boost::placeholders;
    return std::find_if(
             geometryObjects.begin(), geometryObjects.end(),
             boost::bind(&GeometryObject::name, _1) == name)
           != geometryObjects.end();
  }

  inline void GeometryData::fillInnerOuterObjectMaps(const GeometryModel & geomModel)
  {
    innerObjects.clear();
    outerObjects.clear();

    for (GeomIndex gid = 0; gid < geomModel.geometryObjects.size(); gid++)
      innerObjects[geomModel.geometryObjects[gid].parentJoint].push_back(gid);

    BOOST_FOREACH (const CollisionPair & pair, geomModel.collisionPairs)
    {
      outerObjects[geomModel.geometryObjects[pair.first].parentJoint].push_back(pair.second);
    }
  }

  inline std::ostream & operator<<(std::ostream & os, const GeometryModel & geomModel)
  {
    os << "Nb geometry objects = " << geomModel.ngeoms << std::endl;

    for (GeomIndex i = 0; i < (GeomIndex)(geomModel.ngeoms); ++i)
    {
      os << geomModel.geometryObjects[i] << std::endl;
    }

    return os;
  }

  inline std::ostream & operator<<(std::ostream & os, const GeometryData & geomData)
  {
#ifdef PINOCCHIO_WITH_COLLISION
    os << "Number of collision pairs = " << geomData.activeCollisionPairs.size() << std::endl;

    for (PairIndex i = 0; i < (PairIndex)(geomData.activeCollisionPairs.size()); ++i)
    {
      os << "Pairs " << i << (geomData.activeCollisionPairs[i] ? " active" : " inactive")
         << std::endl;
    }
#else
    os << "WARNING** Without coal library, no collision checking or distance computations are "
          "possible. Only geometry placements can be computed."
       << std::endl;
#endif
    os << "Number of geometry objects = " << geomData.oMg.size() << std::endl;

    return os;
  }

  inline void GeometryModel::addCollisionPair(const CollisionPair & pair)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair.first < ngeoms, "The input pair.first is larger than the number of geometries contained "
                           "in the GeometryModel");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair.second < ngeoms, "The input pair.second is larger than the number of geometries "
                            "contained in the GeometryModel");
    if (!existCollisionPair(pair))
    {
      collisionPairs.push_back(pair);

      collisionPairMapping((Eigen::Index)pair.second, (Eigen::Index)pair.first) =
        (int)(collisionPairs.size() - 1);
      collisionPairMapping((Eigen::Index)pair.first, (Eigen::Index)pair.second) =
        collisionPairMapping(
          (Eigen::Index)pair.second,
          (Eigen::Index)pair.first); // make symmetric
    }
  }

  inline void GeometryModel::setCollisionPairs(const MatrixXb & map, const bool upper)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      map.rows(), (Eigen::Index)ngeoms, "Input map does not have the correct number of rows.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      map.cols(), (Eigen::Index)ngeoms, "Input map does not have the correct number of columns.");
    removeAllCollisionPairs();

    for (Eigen::Index i = 0; i < (Eigen::Index)ngeoms; ++i)
    {
      for (Eigen::Index j = i + 1; j < (Eigen::Index)ngeoms; ++j)
      {
        if (upper)
        {
          if (map(i, j))
            addCollisionPair(CollisionPair((std::size_t)i, (std::size_t)j));
        }
        else
        {
          if (map(j, i))
            addCollisionPair(CollisionPair((std::size_t)i, (std::size_t)j));
        }
      }
    }
  }

  inline void GeometryModel::addAllCollisionPairs()
  {
    removeAllCollisionPairs();
    for (GeomIndex i = 0; i < ngeoms; ++i)
    {
      const JointIndex joint_i = geometryObjects[i].parentJoint;
      for (GeomIndex j = i + 1; j < ngeoms; ++j)
      {
        const JointIndex joint_j = geometryObjects[j].parentJoint;
        if (joint_i != joint_j)
          addCollisionPair(CollisionPair(i, j));
      }
    }
  }

  inline void GeometryModel::removeCollisionPair(const CollisionPair & pair)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair.first < ngeoms, "The input pair.first is larger than the number of geometries contained "
                           "in the GeometryModel");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair.second < ngeoms, "The input pair.second is larger than the number of geometries "
                            "contained in the GeometryModel");

    const long index = (long)findCollisionPair(pair);

    if (index != (long)collisionPairs.size())
    {
      collisionPairMapping((Eigen::Index)pair.first, (Eigen::Index)pair.second) =
        collisionPairMapping((Eigen::Index)pair.second, (Eigen::Index)pair.first) = -1;
      collisionPairs.erase(collisionPairs.begin() + index);

      for (Eigen::Index i = 0; i < collisionPairMapping.cols(); ++i)
      {
        for (Eigen::Index j = i + 1; j < collisionPairMapping.cols(); ++j)
        {
          if (collisionPairMapping(i, j) > index)
          {
            collisionPairMapping(i, j)--;
            collisionPairMapping(j, i) = collisionPairMapping(i, j);
          }
        }
      }
    }
  }

  inline void GeometryModel::removeAllCollisionPairs()
  {
    collisionPairs.clear();
    collisionPairMapping.fill(-1);
  }

  inline bool GeometryModel::existCollisionPair(const CollisionPair & pair) const
  {
    return collisionPairMapping((Eigen::Index)pair.first, (Eigen::Index)pair.second) != -1;
  }

  inline PairIndex GeometryModel::findCollisionPair(const CollisionPair & pair) const
  {
    int res = collisionPairMapping((Eigen::Index)pair.first, (Eigen::Index)pair.second);
    if (res == -1)
      return collisionPairs.size();
    else
      return (PairIndex)res;
  }

  inline void GeometryData::activateCollisionPair(const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair_id < activeCollisionPairs.size(),
      "The input argument pair_id is larger than the number of collision pairs contained in "
      "activeCollisionPairs.");
    activeCollisionPairs[pair_id] = true;
  }

  inline void GeometryData::activateAllCollisionPairs()
  {
    std::fill(activeCollisionPairs.begin(), activeCollisionPairs.end(), true);
  }

  inline void GeometryData::setActiveCollisionPairs(
    const GeometryModel & geom_model, const MatrixXb & map, const bool upper)
  {
    const Eigen::Index ngeoms = (Eigen::Index)geom_model.ngeoms;
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      map.rows(), ngeoms, "Input map does not have the correct number of rows.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      map.cols(), ngeoms, "Input map does not have the correct number of columns.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      geom_model.collisionPairs.size(), activeCollisionPairs.size(),
      "Current geometry data and the input geometry model are not conistent.");

    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];

      Eigen::Index i, j;
      if (upper)
      {
        j = (Eigen::Index)std::max(cp.first, cp.second);
        i = (Eigen::Index)std::min(cp.first, cp.second);
      }
      else
      {
        i = (Eigen::Index)std::max(cp.first, cp.second);
        j = (Eigen::Index)std::min(cp.first, cp.second);
      }

      activeCollisionPairs[k] = map(i, j);
    }
  }

  inline void GeometryData::setGeometryCollisionStatus(
    const GeometryModel & geom_model, const GeomIndex geom_id, const bool enable_collision)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      geom_id < geom_model.ngeoms, "The index of the geometry is not valid");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      geom_model.collisionPairs.size(), activeCollisionPairs.size(),
      "Current geometry data and the input geometry model are not conistent.");

    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];
      if (cp.first == geom_id || cp.second == geom_id)
      {
        activeCollisionPairs[k] = enable_collision;
      }
    }
  }

#ifdef PINOCCHIO_WITH_COLLISION
  inline void GeometryData::setSecurityMargins(
    const GeometryModel & geom_model,
    const MatrixXs & security_margin_map,
    const bool upper,
    const bool sync_distance_upper_bound)
  {
    const Eigen::Index ngeoms = (Eigen::Index)geom_model.ngeoms;
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      security_margin_map.rows(), ngeoms, "Input map does not have the correct number of rows.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      security_margin_map.cols(), ngeoms, "Input map does not have the correct number of columns.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      geom_model.collisionPairs.size(), collisionRequests.size(),
      "Current geometry data and the input geometry model are not consistent.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      geom_model.collisionPairs.size(), contactPatchRequests.size(),
      "Current geometry data and the input geometry model are not consistent.");

    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
      const CollisionPair & cp = geom_model.collisionPairs[k];

      Eigen::Index i, j;
      if (upper)
      {
        j = (Eigen::Index)std::max(cp.first, cp.second);
        i = (Eigen::Index)std::min(cp.first, cp.second);
      }
      else
      {
        i = (Eigen::Index)std::max(cp.first, cp.second);
        j = (Eigen::Index)std::min(cp.first, cp.second);
      }

      collisionRequests[k].security_margin = security_margin_map(i, j);
      if (sync_distance_upper_bound)
        collisionRequests[k].distance_upper_bound = collisionRequests[k].security_margin;
    }
  }
#endif // ifdef PINOCCHIO_WITH_COLLISION

  inline void GeometryData::deactivateCollisionPair(const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      pair_id < activeCollisionPairs.size(),
      "The input argument pair_id is larger than the number of collision pairs contained in "
      "activeCollisionPairs.");
    activeCollisionPairs[pair_id] = false;
  }

  inline void GeometryData::deactivateAllCollisionPairs()
  {
    std::fill(activeCollisionPairs.begin(), activeCollisionPairs.end(), false);
  }

} // namespace pinocchio
