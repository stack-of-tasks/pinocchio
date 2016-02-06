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


namespace se3
{

  // Result of distance computation between two CollisionObjects.
  struct DistanceResult
  {
    typedef Model::Index Index;

    DistanceResult(fcl::DistanceResult dist_fcl, const Index co1, const Index co2)
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
    Index object1;
    /// Index of the second colision object
    Index object2;
    
  }; // struct DistanceResult 
  
  struct GeometryModel
  {
    typedef Model::Index Index;

    Index ngeom;
    std::vector<fcl::CollisionObject> collision_objects;
    std::vector<std::string> geom_names;
    std::vector<Index> geom_parents;                          // Joint parent of body <i>, denoted <li> (li==parents[i])
    std::vector<SE3> geometryPlacement;                       // Position of geometry object in parent joint's frame
    
    std::map < Index, std::list<Index> >  innerObjects;       // Associate a list of CollisionObjects to a given joint Id 
    std::map < Index, std::list<Index> >  outerObjects;       // Associate a list of CollisionObjects to a given joint Id 

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

    Index addGeomObject(const Index parent, const fcl::CollisionObject & co, const SE3 & placement, const std::string & geoName = "");
    Index getGeomId(const std::string & name) const;
    bool existGeomName(const std::string & name) const;
    const std::string & getGeomName(const Index index) const;

    void addInnerObject(const Index joint, const Index inner_object);
    void addOutterObject(const Index joint, const Index outer_object);

    friend std::ostream& operator<<(std::ostream & os, const GeometryModel & model_geom);
  }; // struct GeometryModel

  struct GeometryData
  {
    typedef Model::Index Index;
    typedef std::pair<Index,Index> CollisionPair_t;
    typedef std::vector<CollisionPair_t> CollisionPairsVector_t;

    const Data & data_ref;
    const GeometryModel & model_geom;

    std::vector<se3::SE3> oMg;
    std::vector<fcl::Transform3f> oMg_fcl;

    CollisionPairsVector_t collision_pairs;
    Index nCollisionPairs;

    std::vector <DistanceResult> distances;
    std::vector <bool> collisions;

    GeometryData(const Data & data, const GeometryModel & model_geom)
        : data_ref(data)
        , model_geom(model_geom)
        , oMg(model_geom.ngeom)
        , oMg_fcl(model_geom.ngeom)
        , collision_pairs()
        , nCollisionPairs(0)
        , distances()
        , collisions()
    {
      initializeListOfCollisionPairs();
      distances.resize(nCollisionPairs, DistanceResult( fcl::DistanceResult(), 0, 0) );
      collisions.resize(nCollisionPairs, false );
    }

    ~GeometryData() {};

    void addCollisionPair (const Index co1, const Index co2);
    void addCollisionPair (const CollisionPair_t & pair);
    
    void removeCollisionPair (const Index co1, const Index co2);
    void removeCollisionPair (const CollisionPair_t& pair);
   
    ///
    /// \brief Check if a CollisionPair given by the index of the two colliding geometries exists in collision_pairs.
    ///        See also findCollisitionPair(const Index co1, const Index co2).
    ///
    /// \param[in] co1 Index of the first colliding geometry.
    /// \param[in] co2 Index of the second colliding geometry.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair (const Index co1, const Index co2) const ;
    
    ///
    /// \brief Check if a CollisionPair given by the index of the two colliding geometries exists in collision_pairs.
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
    Index findCollisionPair (const Index co1, const Index co2) const;
    
    ///
    /// \brief Return the index of a given CollisionPair in collision_pairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collision_pairs.
    ///
    Index findCollisionPair (const CollisionPair_t & pair) const;
    
    void fillAllPairsAsCollisions();
    void desactivateCollisionPairs();
    void initializeListOfCollisionPairs();

    bool collide(const Index co1, const Index co2) const;

    fcl::DistanceResult computeDistance(const Index co1, const Index co2) const;
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
