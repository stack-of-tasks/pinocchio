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

#ifndef __se3_geometry_hxx__
#define __se3_geometry_hxx__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include <iostream>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <map>
#include <list>
#include <utility>

/// @cond DEV

namespace se3
{

  inline GeometryModel::GeomIndex GeometryModel::addGeometryObject(const JointIndex parent,
                                                                   const fcl::CollisionObject & co,
                                                                   const SE3 & placement,
                                                                   const std::string & geom_name,
                                                                   const std::string & mesh_path) throw(std::invalid_argument)
  {

    Index idx = (Index) (ngeoms ++);

    geometry_objects.push_back(GeometryObject( geom_name, parent, co,
                                               placement, mesh_path));
    addInnerObject(parent, idx);
    return idx;
  }


  inline GeometryModel::GeomIndex GeometryModel::getGeometryId(const std::string & name) const
  {

    std::vector<GeometryObject>::const_iterator it = std::find_if(geometry_objects.begin(),
                                                                  geometry_objects.end(),
                                                                  boost::bind(&GeometryObject::name, _1) == name
                                                                  );
    return GeometryModel::GeomIndex(it - geometry_objects.begin());
  }



  inline bool GeometryModel::existGeometryName(const std::string & name) const
  {
    return std::find_if(geometry_objects.begin(),
                        geometry_objects.end(),
                        boost::bind(&GeometryObject::name, _1) == name) != geometry_objects.end();
  }


  inline const std::string& GeometryModel::getGeometryName(const GeomIndex index) const
  {
    assert( index < (GeomIndex)geometry_objects.size() );
    return geometry_objects[index].name;
  }


  inline void GeometryModel::addInnerObject(const JointIndex joint_id, const GeomIndex inner_object)
  {
    if (std::find(innerObjects[joint_id].begin(),
                  innerObjects[joint_id].end(),
                  inner_object) == innerObjects[joint_id].end())
      innerObjects[joint_id].push_back(inner_object);
    else
      std::cout << "inner object already added" << std::endl;
  }

  inline void GeometryModel::addOutterObject (const JointIndex joint, const GeomIndex outer_object)
  {
    if (std::find(outerObjects[joint].begin(),
                  outerObjects[joint].end(),
                  outer_object) == outerObjects[joint].end())
      outerObjects[joint].push_back(outer_object);
    else
      std::cout << "outer object already added" << std::endl;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryModel & model_geom)
  {
    os << "Nb geometry objects = " << model_geom.ngeoms << std::endl;
    
    for(GeometryModel::Index i=0;i<(GeometryModel::Index)(model_geom.ngeoms);++i)
    {
      os  << model_geom.geometry_objects[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryData & data_geom)
  {
    os << "Nb collision pairs = " << data_geom.nCollisionPairs << std::endl;
    
    for(GeometryData::Index i=0;i<(GeometryData::Index)(data_geom.nCollisionPairs);++i)
    {
      os << "collision object in position " << data_geom.collision_pairs[i] << std::endl;
    }

    return os;
  }

  inline void GeometryData::addCollisionPair (const GeomIndex co1, const GeomIndex co2)
  {
    assert ( co1 != co2);
    assert ( co2 < model_geom.ngeoms);
    CollisionPair_t pair(co1, co2);
    
    addCollisionPair(pair);
  }

  inline void GeometryData::addCollisionPair (const CollisionPair_t & pair)
  {
    assert(pair.second < model_geom.ngeoms);
    
    if (!existCollisionPair(pair))
    {
      collision_pairs.push_back(pair);
      nCollisionPairs++;
    }
  }
  
  inline void GeometryData::addAllCollisionPairs()
  {
    removeAllCollisionPairs();
    collision_pairs.reserve((model_geom.ngeoms * (model_geom.ngeoms-1))/2);
    for (Index i = 0; i < model_geom.ngeoms; ++i)
      for (Index j = i+1; j < model_geom.ngeoms; ++j)
        addCollisionPair(i,j);
  }
  
  inline void GeometryData::removeCollisionPair (const GeomIndex co1, const GeomIndex co2)
  {
    assert(co1 < co2);
    assert(co2 < model_geom.ngeoms);
    assert(existCollisionPair(co1,co2));

    removeCollisionPair (CollisionPair_t(co1,co2));
  }

  inline void GeometryData::removeCollisionPair (const CollisionPair_t & pair)
  {
    assert(pair.second < model_geom.ngeoms);

    CollisionPairsVector_t::iterator it = std::find(collision_pairs.begin(),
                                                    collision_pairs.end(),
                                                    pair);
    if (it != collision_pairs.end())
    {
      collision_pairs.erase(it);
      nCollisionPairs--;
    }
  }
  
  inline void GeometryData::removeAllCollisionPairs ()
  {
    collision_pairs.clear();
    nCollisionPairs = 0;
  }

  inline bool GeometryData::existCollisionPair (const GeomIndex co1, const GeomIndex co2) const
  {
    return existCollisionPair(CollisionPair_t(co1,co2));
  }

  inline bool GeometryData::existCollisionPair (const CollisionPair_t & pair) const
  {
    return (std::find(collision_pairs.begin(),
                      collision_pairs.end(),
                      pair) != collision_pairs.end());
  }
  
  inline GeometryData::Index GeometryData::findCollisionPair (const GeomIndex co1, const GeomIndex co2) const
  {
    return findCollisionPair(CollisionPair_t(co1,co2));
  }
  
  inline GeometryData::Index GeometryData::findCollisionPair (const CollisionPair_t & pair) const
  {
    CollisionPairsVector_t::const_iterator it = std::find(collision_pairs.begin(),
                                                          collision_pairs.end(),
                                                          pair);
    
    return (Index) distance(collision_pairs.begin(), it);
  }
  
//  std::vector<Index> GeometryData::findCollisionPairsSupportedBy(const JointIndex joint_id) const
//  {
////    std::vector<Index> collision_pairs;
////    for(CollisionPairsVector_t::const_iterator it = collision_pairs.begin();
////        it != collision_pairs.end(); ++it)
////    {
////      if (geom_model.it->first )
////    }
//  }

  // TODO :  give a srdf file as argument, read it, and remove corresponding
  // pairs from list collision_pairs
  inline void GeometryData::desactivateCollisionPairs()
  {

  }

  inline void GeometryData::initializeListOfCollisionPairs()
  {
    addAllCollisionPairs();
    desactivateCollisionPairs();
    assert(nCollisionPairs == collision_pairs.size());
  }

  inline CollisionResult GeometryData::computeCollision(const GeomIndex co1, const GeomIndex co2) const
  {
    return computeCollision(CollisionPair_t(co1,co2));
  }
  
  inline CollisionResult GeometryData::computeCollision(const CollisionPair_t & pair) const
  {
    const Index & co1 = pair.first;
    const Index & co2 = pair.second;
    
    fcl::CollisionRequest collisionRequest (1, false, false, 1, false, true, fcl::GST_INDEP);
    fcl::CollisionResult collisionResult;

    fcl::collide (model_geom.geometry_objects[co1].collision_object.collisionGeometry().get(), oMg_fcl[co1],
                  model_geom.geometry_objects[co2].collision_object.collisionGeometry().get(), oMg_fcl[co2],
                  collisionRequest, collisionResult);

    return CollisionResult (collisionResult, co1, co2);
  }
  
  inline void GeometryData::computeAllCollisions()
  {
    for(size_t i = 0; i<nCollisionPairs; ++i)
    {
      const CollisionPair_t & pair = collision_pairs[i];
      collision_results[i] = computeCollision(pair.first, pair.second);
    }
  }
  
  inline bool GeometryData::isColliding() const
  {
    for(CollisionPairsVector_t::const_iterator it = collision_pairs.begin(); it != collision_pairs.end(); ++it)
    {
      if (computeCollision(it->first, it->second).fcl_collision_result.isCollision())
        return true;
    }
    return false;
  }

  inline DistanceResult GeometryData::computeDistance(const GeomIndex co1, const GeomIndex co2) const
  {
    return computeDistance(CollisionPair_t(co1,co2));
  }
  
  inline DistanceResult GeometryData::computeDistance(const CollisionPair_t & pair) const
  {
    const Index & co1 = pair.first;
    const Index & co2 = pair.second;
    
    fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
    fcl::DistanceResult result;
    fcl::distance ( model_geom.geometry_objects[co1].collision_object.collisionGeometry().get(), oMg_fcl[co1],
                    model_geom.geometry_objects[co2].collision_object.collisionGeometry().get(), oMg_fcl[co2],
                    distanceRequest, result);
    
    return DistanceResult (result, co1, co2);
  }
  
  inline void GeometryData::computeAllDistances ()
  {
    for(size_t i = 0; i<nCollisionPairs; ++i)
    {
      const CollisionPair_t & pair = collision_pairs[i];
      distance_results[i] = computeDistance(pair.first, pair.second);
    }
  }

  inline void GeometryData::resetDistances()
  {
    std::fill(distance_results.begin(), distance_results.end(), DistanceResult( fcl::DistanceResult(), 0, 0) );
  }
  
  inline void GeometryData::addCollisionPairsFromSrdf(const std::string & filename,
                                               const bool verbose) throw (std::invalid_argument)
  {
    // Add all collision pairs
    addAllCollisionPairs();
    
    // Read SRDF file
    srdf::removeCollisionPairsFromSrdf(model_geom, *this, filename, verbose);
  }


} // namespace se3

/// @endcond

#endif // ifndef __se3_geometry_hxx__
