//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_geom_hpp__
#define __se3_geom_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include <iostream>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <map>
#include <list>
#include <utility>
#include <assert.h>


namespace se3
{
  
  struct CollisionPair: public std::pair<Model::GeomIndex, Model::GeomIndex>
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;
    typedef std::pair<Model::GeomIndex, Model::GeomIndex> Base;
   
    ///
    /// \brief Default constructor of a collision pair from two collision object indexes.
    ///        The indexes must be ordered such that co1 < co2. If not, the constructor reverts the indexes.
    ///
    /// \param[in] co1 Index of the first collision object
    /// \param[in] co2 Index of the second collision object
    ///
    CollisionPair(const GeomIndex co1, const GeomIndex co2) : Base(co1,co2)
    {
      assert(co1 != co2 && "The index of collision objects must not be equal.");
      if (co1 > co2)
      {
        first = co2; second = co1;
      }
    }
    
    void disp(std::ostream & os) const { os << "collision pair (" << first << "," << second << ")\n"; }
    friend std::ostream & operator << (std::ostream & os, const CollisionPair & X)
    {
      X.disp(os); return os;
    }
  }; // struct CollisionPair

  
  /**
   * @brief      Result of distance computation between two CollisionObjects
   */
  struct DistanceResult
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;

    DistanceResult(fcl::DistanceResult dist_fcl, const GeomIndex co1, const GeomIndex co2)
    : fcl_distance_result(dist_fcl), object1(co1), object2(co2)
    {}


    ///
    /// @brief Return the minimal distance between two geometry objects
    ///
    double distance () const { return fcl_distance_result.min_distance; }

    ///
    /// \brief Return the witness point on the inner object expressed in global frame.
    ///
    Eigen::Vector3d closestPointInner () const { return toVector3d(fcl_distance_result.nearest_points [0]); }
    
    ///
    /// \brief Return the witness point on the outer object expressed in global frame.
    ///
    Eigen::Vector3d closestPointOuter () const { return toVector3d(fcl_distance_result.nearest_points [1]); }
    
    bool operator == (const DistanceResult & other) const
    {
      return (distance() == other.distance()
        && closestPointInner() == other.closestPointInner()
        && closestPointOuter() == other.closestPointOuter()
        && object1 == other.object1
        && object2 == other.object2);
    }
    
    /// \brief The FCL result of the distance computation
    fcl::DistanceResult fcl_distance_result;
    
    /// \brief Index of the first colision object
    GeomIndex object1;

    /// \brief Index of the second colision object
    GeomIndex object2;
    
  }; // struct DistanceResult 
  

  /**
   * @brief      Result of collision computation between two CollisionObjects
   */
  struct CollisionResult
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;

    /**
     * @brief      Default constrcutor of a CollisionResult
     *
     * @param[in]  coll_fcl  The FCL collision result
     * @param[in]  co1       Index of the first geometry object involved in the computation
     * @param[in]  co2       Index of the second geometry object involved in the computation
     */
    CollisionResult(fcl::CollisionResult coll_fcl, const GeomIndex co1, const GeomIndex co2)
    : fcl_collision_result(coll_fcl), object1(co1), object2(co2)
    {}

    bool operator == (const CollisionResult & other) const
    {
      return (fcl_collision_result == other.fcl_collision_result
              && object1 == other.object1
              && object2 == other.object2);
    }

    /// \brief The FCL result of the collision computation
    fcl::CollisionResult fcl_collision_result;

    /// \brief Index of the first collision object
    GeomIndex object1;

    /// \brief Index of the second collision object
    GeomIndex object2;

  }; // struct CollisionResult

/// \brief Return true if the intrinsic geometry of the two CollisionObject is the same
inline bool operator == (const fcl::CollisionObject & lhs, const fcl::CollisionObject & rhs)
{
  return lhs.collisionGeometry() == rhs.collisionGeometry()
          && lhs.getAABB().min_ == rhs.getAABB().min_
          && lhs.getAABB().max_ == rhs.getAABB().max_;
}
enum GeometryType
{
  VISUAL,
  COLLISION,
  NONE
};

struct GeometryObject
{
  typedef Model::Index Index;
  typedef Model::JointIndex JointIndex;
  typedef Model::GeomIndex GeomIndex;


  /// \brief Name of the geometry object
  std::string name;

  /// \brief Index of the parent joint
  JointIndex parent;

  /// \brief The actual cloud of points representing the collision mesh of the object
  fcl::CollisionObject collision_object;

  /// \brief Position of geometry object in parent joint's frame
  SE3 placement;

  /// \brief Absolute path to the mesh file
  std::string mesh_path;


  GeometryObject(const std::string & name, const JointIndex parent, const fcl::CollisionObject & collision,
                 const SE3 & placement, const std::string & mesh_path)
                : name(name)
                , parent(parent)
                , collision_object(collision)
                , placement(placement)
                , mesh_path(mesh_path)
  {}

  GeometryObject & operator=(const GeometryObject & other)
  {
    name = other.name;
    parent = other.parent;
    collision_object = other.collision_object;
    placement = other.placement;
    mesh_path = other.mesh_path;
    return *this;
  }

};
  
  inline bool operator==(const GeometryObject & lhs, const GeometryObject & rhs)
  {
    return ( lhs.name == rhs.name
            && lhs.parent == rhs.parent
            && lhs.collision_object == rhs.collision_object
            && lhs.placement == rhs.placement
            && lhs.mesh_path ==  rhs.mesh_path
            );
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object)
  {
    os  << "Name: \t \n" << geom_object.name << "\n"
        << "Parent ID: \t \n" << geom_object.parent << "\n"
        // << "collision object: \t \n" << geom_object.collision_object << "\n"
        << "Position in parent frame: \t \n" << geom_object.placement << "\n"
        << "Absolute path to mesh file: \t \n" << geom_object.mesh_path << "\n"
        << std::endl;
    return os;
  }

  struct GeometryModel
  {
    typedef Model::Index Index;
    typedef Model::JointIndex JointIndex;
    typedef Model::GeomIndex GeomIndex;
    
    typedef std::vector<GeomIndex> GeomIndexList;

    /// \brief A const reference to the reference model.
    const se3::Model & model;

    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    std::vector<GeometryObject> geometryObjects;
    
    /// \brief A list of associated collision GeometryObjects to a given joint Id.
    ///        Inner objects can be seen as geometry objects that directly move when the associated joint moves
    std::map < JointIndex, GeomIndexList >  innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///        Outer objects can be seen as geometry objects that may often be obstacles to the Inner objects of given joint
    std::map < JointIndex, GeomIndexList >  outerObjects;

    GeometryModel(const se3::Model & model)
      : model(model)
      , ngeoms(0)
      , geometryObjects()
      , innerObjects()
      , outerObjects()
    {}

    ~GeometryModel() {};

    /**
     * @brief      Add a geometry object of a given type to a GeometryModel
     *
     * @param[in]  parent     Index of the parent joint
     * @param[in]  co         The actual fcl CollisionObject
     * @param[in]  placement  The relative placement regarding to the parent frame
     * @param[in]  geom_name  The name of the Geometry Object
     * @param[in]  mesh_path  The absolute path to the mesh
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     */
    inline GeomIndex addGeometryObject(const JointIndex parent, const fcl::CollisionObject & co,
                                       const SE3 & placement, const std::string & geom_name = "",
                                       const std::string & mesh_path = "") throw(std::invalid_argument);



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


    /**
     * @brief      Get the name of a GeometryObject given by its index.
     *
     * @param[in]  index  Index of the GeometryObject
     *
     * @return     Name of the GeometryObject
     */
    const std::string & getGeometryName(const GeomIndex index) const;


    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's inner objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an inner object
     */
    void addInnerObject(const JointIndex joint, const GeomIndex inner_object);
    
    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's outer objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an outer object
     */
    void addOutterObject(const JointIndex joint, const GeomIndex outer_object);

    friend std::ostream& operator<<(std::ostream & os, const GeometryModel & model_geom);
  }; // struct GeometryModel

  struct GeometryData
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;
    typedef CollisionPair CollisionPair_t;
    typedef std::vector<CollisionPair_t> CollisionPairsVector_t;

    ///
    /// \brief A const reference to the data associated to the robot model.
    ///        See class Data.
    ///
    const Data & data_ref;
    
    ///
    /// \brief A const reference to the model storing all the geometries.
    ///        See class GeometryModel.
    ///
    const GeometryModel & model_geom;

    ///
    /// \brief Vector gathering the SE3 placements of the geometry objects relative to the world.
    ///        See updateGeometryPlacements to update the placements.
    ///
    std::vector<se3::SE3> oMg;

    ///
    /// \brief Same as oMg but using fcl::Transform3f to store placement.
    ///        This pre-allocation avoids dynamic allocation during collision checking or distance computations.
    ///
    std::vector<fcl::Transform3f> oMg_fcl;
    ///
    /// \brief Vector of collision pairs.
    ///        See addCollisionPair, removeCollisionPair to fill or remove elements in the vector.
    ///
    CollisionPairsVector_t collision_pairs;
    
    ///
    /// \brief Number of collision pairs stored in collision_pairs.
    ///
    Index nCollisionPairs;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector <DistanceResult> distance_results;
    
    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector <CollisionResult> collision_results;

    GeometryData(const Data & data, const GeometryModel & model_geom)
        : data_ref(data)
        , model_geom(model_geom)
        , oMg(model_geom.ngeoms)
        , oMg_fcl(model_geom.ngeoms)
        , collision_pairs()
        , nCollisionPairs(0)
        , distance_results()
        , collision_results()
    {
      const std::size_t num_max_collision_pairs = (model_geom.ngeoms * (model_geom.ngeoms-1))/2;
      collision_pairs.reserve(num_max_collision_pairs);
      distance_results.reserve(num_max_collision_pairs);
      collision_results.reserve(num_max_collision_pairs);
    }

    ~GeometryData() {};

    ///
    /// \brief Add a collision pair given by the index of the two colliding geometries into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] co1 Index of the first colliding geometry.
    /// \param[in] co2 Index of the second colliding geometry.
    ///
    void addCollisionPair (const GeomIndex co1, const GeomIndex co2);
    
    ///
    /// \brief Add a collision pair into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] pair The CollisionPair to add.
    ///
    void addCollisionPair (const CollisionPair_t & pair);
    
    ///
    /// \brief Add all possible collision pairs.
    ///
    void addAllCollisionPairs();
   
    ///
    /// \brief Remove if exists the collision pair given by the index of the two colliding geometries from the vector of collision_pairs.
    ///
    /// \param[in] co1 Index of the first colliding geometry.
    /// \param[in] co2 Index of the second colliding geometry.
    ///
    void removeCollisionPair (const GeomIndex co1, const GeomIndex co2);
    
    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair (const CollisionPair_t& pair);
    
    ///
    /// \brief Remove all collision pairs from collision_pairs. Same as collision_pairs.clear().
    void removeAllCollisionPairs ();
   
    ///
    /// \brief Check if a collision pair given by the index of the two colliding geometries exists in collision_pairs.
    ///        See also findCollisitionPair(const GeomIndex co1, const GeomIndex co2).
    ///
    /// \param[in] co1 Index of the first colliding geometry.
    /// \param[in] co2 Index of the second colliding geometry.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair (const GeomIndex co1, const GeomIndex co2) const ;
    
    ///
    /// \brief Check if a collision pair exists in collision_pairs.
    ///        See also findCollisitionPair(const CollisionPair_t & pair).
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair (const CollisionPair_t & pair) const;
    
    ///
    /// \brief Return the index in collision_pairs of a CollisionPair given by the index of the two colliding geometries.
    ///
    /// \param[in] co1 Index of the first colliding geometry.
    /// \param[in] co2 Index of the second colliding geometry.
    ///
    /// \return The index of the collision pair in collision_pairs.
    ///
    Index findCollisionPair (const GeomIndex co1, const GeomIndex co2) const;
    
    ///
    /// \brief Return the index of a given collision pair in collision_pairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collision_pairs.
    ///
    Index findCollisionPair (const CollisionPair_t & pair) const;
    
    void initializeListOfCollisionPairs();
    

    ///
    /// \brief Compute the collision status between two collision objects given by their indexes.
    ///
    /// \param[in] co1 Index of the first collision object.
    /// \param[in] co2 Index of the second collision object.
    ///
    /// \return Return true is the collision objects are colliding.
    ///
    CollisionResult computeCollision(const GeomIndex co1, const GeomIndex co2) const;
    
    ///
    /// \brief Compute the collision status between two collision objects of a given collision pair.
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return Return true is the collision objects are colliding.
    ///
    CollisionResult computeCollision(const CollisionPair_t & pair) const;
    
    ///
    /// \brief Compute the collision result of all the collision pairs according to
    ///        the current placements of the geometires stored in GeometryData::oMg.
    ///        The results are stored in the vector GeometryData::collision_results.
    ///
    void computeAllCollisions();
    
    ///
    /// \brief Check if at least one of the collision pairs has its two collision objects in collision.
    ///
    bool isColliding() const;

    ///
    /// \brief Compute the minimal distance between two collision objects given by their indexes.
    ///
    /// \param[in] co1 Index of the first collision object.
    /// \param[in] co2 Index of the second collision object.
    ///
    /// \return An fcl struct containing the distance result.
    ///
    DistanceResult computeDistance(const GeomIndex co1, const GeomIndex co2) const;
    
    ///
    /// \brief Compute the minimal distance between collision objects of a collison pair
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return An fcl struct containing the distance result.
    ///
    DistanceResult computeDistance(const CollisionPair_t & pair) const;
    
    ///
    /// \brief Compute the distance result for all collision pairs according to
    ///        the current placements of the geometries stored in GeometryData::oMg.
    ///        The results are stored in the vector GeometryData::distance_results.
    ///
    void computeAllDistances();
    
    void resetDistances();

    void displayCollisionPairs() const
    {
      for (std::vector<CollisionPair_t>::const_iterator it = collision_pairs.begin(); it != collision_pairs.end(); ++it)
      {
        std::cout << it-> first << "\t" << it->second << std::endl;
      }
    }
    friend std::ostream & operator<<(std::ostream & os, const GeometryData & data_geom);
    
  }; // struct GeometryData

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry.hxx"

#endif // ifndef __se3_geom_hpp__
