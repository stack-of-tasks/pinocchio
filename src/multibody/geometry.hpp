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
    }

    bool operator== (const CollisionPair& rhs) const
    {
      return (first == rhs.first && second == rhs.second)
        || (first == rhs.second && second == rhs.first);
    }
    
    void disp(std::ostream & os) const { os << "collision pair (" << first << "," << second << ")\n"; }
    friend std::ostream & operator << (std::ostream & os, const CollisionPair & X)
    {
      X.disp(os); return os;
    }
  }; // struct CollisionPair
  typedef std::vector<CollisionPair> CollisionPairsVector_t;
  
  /**
   * @brief      Result of distance computation between two CollisionObjects
   */
  struct DistanceResult
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;

    DistanceResult() : fcl_distance_result(), object1(0), object2(0) {}
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
    CollisionResult() : fcl_collision_result(), object1(0), object2(0) {}

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
  boost::shared_ptr<fcl::CollisionGeometry> collision_geometry;

  /// \brief Position of geometry object in parent joint's frame
  SE3 placement;

  /// \brief Absolute path to the mesh file
  std::string mesh_path;


  GeometryObject(const std::string & name, const JointIndex parent, const boost::shared_ptr<fcl::CollisionGeometry> & collision,
                 const SE3 & placement, const std::string & mesh_path)
                : name(name)
                , parent(parent)
                , collision_geometry(collision)
                , placement(placement)
                , mesh_path(mesh_path)
  {}

  GeometryObject & operator=(const GeometryObject & other)
  {
    name = other.name;
    parent = other.parent;
    collision_geometry = other.collision_geometry;
    placement = other.placement;
    mesh_path = other.mesh_path;
    return *this;
  }

};
  
  inline bool operator==(const GeometryObject & lhs, const GeometryObject & rhs)
  {
    return ( lhs.name == rhs.name
            && lhs.parent == rhs.parent
            && lhs.collision_geometry == rhs.collision_geometry
            && lhs.placement == rhs.placement
            && lhs.mesh_path ==  rhs.mesh_path
            );
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object)
  {
    os  << "Name: \t \n" << geom_object.name << "\n"
        << "Parent ID: \t \n" << geom_object.parent << "\n"
        // << "collision object: \t \n" << geom_object.collision_geometry << "\n"
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

    
    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    std::vector<GeometryObject> geometryObjects;
    ///
    /// \brief Vector of collision pairs.
    ///
    CollisionPairsVector_t collisionPairs;
  
    /// \brief A list of associated collision GeometryObjects to a given joint Id.
    ///        Inner objects can be seen as geometry objects that directly move when the associated joint moves
    std::map < JointIndex, GeomIndexList >  innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///        Outer objects can be seen as geometry objects that may often be obstacles to the Inner objects of given joint
    std::map < JointIndex, GeomIndexList >  outerObjects;

    GeometryModel()
      : ngeoms(0)
      , geometryObjects()
      , collisionPairs()
      , innerObjects()
      , outerObjects()
    { 
      const std::size_t num_max_collision_pairs = (ngeoms * (ngeoms-1))/2;
      collisionPairs.reserve(num_max_collision_pairs);
    }

    ~GeometryModel() {};

    /**
     * @brief      Add a geometry object to a GeometryModel
     *
     * @param[in]  parent     Index of the parent joint
     * @param[in]  co         The actual fcl CollisionGeometry
     * @param[in]  placement  The relative placement regarding to the parent frame
     * @param[in]  geom_name  The name of the Geometry Object
     * @param[in]  mesh_path  The absolute path to the mesh
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     */
    inline GeomIndex addGeometryObject(const JointIndex parent, const boost::shared_ptr<fcl::CollisionGeometry> & co,
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


    ///
    /// \brief Add a collision pair into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] pair The CollisionPair to add.
    ///
    void addCollisionPair (const CollisionPair & pair);
    
    ///
    /// \brief Add all possible collision pairs.
    ///
    void addAllCollisionPairs();
   
    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair (const CollisionPair& pair);
    
    ///
    /// \brief Remove all collision pairs from collisionPairs. Same as collisionPairs.clear().
    void removeAllCollisionPairs ();
   
    ///
    /// \brief Check if a collision pair exists in collisionPairs.
    ///        See also findCollisitionPair(const CollisionPair & pair).
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair (const CollisionPair & pair) const;
    
    ///
    /// \brief Return the index of a given collision pair in collisionPairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collisionPairs.
    ///
    Index findCollisionPair (const CollisionPair & pair) const;
    
    /// \brief Display on std::cout the list of pairs (is it really useful?).
    void displayCollisionPairs() const;

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
    ///
    std::vector<bool> activeCollisionPairs;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector <DistanceResult> distance_results;
    
    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector <CollisionResult> collision_results;

    ///
    /// \brief Radius of the bodies, i.e. distance of the further point of the geometry model
    /// attached to the body from the joint center.
    ///
    std::vector<double> radius;
    
    GeometryData(const GeometryModel & modelGeom)
        : model_geom(modelGeom)
        , oMg(model_geom.ngeoms)
        , oMg_fcl(model_geom.ngeoms)
        , activeCollisionPairs()
        , distance_results()
        , collision_results()
        , radius()
         
    {
      activeCollisionPairs.resize(modelGeom.collisionPairs.size());
      distance_results.resize(modelGeom.collisionPairs.size());
      collision_results.resize(modelGeom.collisionPairs.size());
    }

    ~GeometryData() {};

    void activateCollisionPair(const Index pairId,const bool flag=true);
    void deactivateCollisionPair(const Index pairId);

    ///
    /// \brief Compute the collision status between two collision objects of a given collision pair.
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return Return true is the collision objects are colliding.
    ///
    CollisionResult computeCollision(const CollisionPair & pair) const;
    
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
    /// \brief Compute the minimal distance between collision objects of a collison pair
    ///
    /// \param[in] pair The collsion pair.
    ///
    /// \return An fcl struct containing the distance result.
    ///
    DistanceResult computeDistance(const CollisionPair & pair) const;
    
    ///
    /// \brief Compute the distance result for all collision pairs according to
    ///        the current placements of the geometries stored in GeometryData::oMg.
    ///        The results are stored in the vector GeometryData::distance_results.
    ///
    void computeAllDistances();
    
    void resetDistances();

    friend std::ostream & operator<<(std::ostream & os, const GeometryData & data_geom);
    
  }; // struct GeometryData

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry.hxx"

#endif // ifndef __se3_geom_hpp__
