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

  // Result of distance computation between two CollisionObjects.
  struct DistanceResult
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;

    DistanceResult(fcl::DistanceResult dist_fcl, const GeomIndex co1, const GeomIndex co2)
    : fcl_distance_result(dist_fcl), object1(co1), object2(co2)
    {}

    // Get distance between objects
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
    
    fcl::DistanceResult fcl_distance_result;
    
    /// Index of the first colision object
    GeomIndex object1;
    /// Index of the second colision object
    GeomIndex object2;
    
  }; // struct DistanceResult 
  
  struct CollisionResult
  {
    typedef Model::Index Index;
    typedef Model::GeomIndex GeomIndex;

    CollisionResult(fcl::CollisionResult coll_fcl, const GeomIndex co1, const GeomIndex co2)
    : fcl_collision_result(coll_fcl), object1(co1), object2(co2)
    {}

    bool operator == (const CollisionResult & other) const
    {
      return (fcl_collision_result == other.fcl_collision_result
              && object1 == other.object1
              && object2 == other.object2);
    }
    fcl::CollisionResult fcl_collision_result;

    // Index of the first collision object
    GeomIndex object1;
    //Index of the second collision object
    GeomIndex object2;

  }; // struct CollisionResult
  

  struct GeometryModel
  {
    typedef Model::Index Index;
    typedef Model::JointIndex JointIndex;
    typedef Model::GeomIndex GeomIndex;

    Index ngeom;
    std::vector<fcl::CollisionObject> collision_objects;
    std::vector<std::string> geom_names;
    std::vector<JointIndex> geom_parents;                          // Joint parent of body <i>, denoted <li> (li==parents[i])
    std::vector<SE3> geometryPlacement;                       // Position of geometry object in parent joint's frame
    
    std::map < JointIndex, std::list<GeomIndex> >  innerObjects;       // Associate a list of CollisionObjects to a given joint Id 
    std::map < JointIndex, std::list<GeomIndex> >  outerObjects;       // Associate a list of CollisionObjects to a given joint Id 

    GeometryModel()
      : ngeom(0)
      , collision_objects()
      , geom_names(0)
      , geom_parents(0)
      , geometryPlacement(0)
      , innerObjects()
      , outerObjects()
    {
    }

    ~GeometryModel() {};

    GeomIndex addGeomObject(const JointIndex parent, const fcl::CollisionObject & co, const SE3 & placement, const std::string & geoName = "");
    GeomIndex getGeomId(const std::string & name) const;
    bool existGeomName(const std::string & name) const;
    const std::string & getGeomName(const GeomIndex index) const;

    void addInnerObject(const JointIndex joint, const GeomIndex inner_object);
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
    /// \brief Vector gathering the SE3 placements of the geometries relative to the world.
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
        , oMg(model_geom.ngeom)
        , oMg_fcl(model_geom.ngeom)
        , collision_pairs()
        , nCollisionPairs(0)
        , distance_results()
        , collision_results()
    {
      const std::size_t num_max_collision_pairs = (model_geom.ngeom * (model_geom.ngeom-1))/2;
      collision_pairs.reserve(num_max_collision_pairs);
      distance_results.resize(num_max_collision_pairs, DistanceResult( fcl::DistanceResult(), 0, 0) );
      collision_results.resize(num_max_collision_pairs, CollisionResult( fcl::CollisionResult(), 0, 0));
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
    
    void desactivateCollisionPairs();
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
