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

#ifndef __se3_geom_hxx__
#define __se3_geom_hxx__


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

  inline GeometryModel::Index GeometryModel::addGeomObject (const Index parent,
                                                            const fcl::CollisionObject & co,
                                                            const SE3 & placement,
                                                            const std::string & geoName)
  {

    Index idx = (Index) (ngeom ++);


    collision_objects    .push_back(co);
    geom_parents         .push_back(parent);
    geometryPlacement    .push_back(placement);
    geom_names           .push_back( (geoName!="")?geoName:random(8));

    return idx;
  }

  inline GeometryModel::Index GeometryModel::getGeomId (const std::string & name) const
  {
    std::vector<std::string>::iterator::difference_type
      res = std::find(geom_names.begin(),geom_names.end(),name) - geom_names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<(long)collision_objects.size())&&"The joint name you asked does not exist" );
    return Index(res);
  }

  inline bool GeometryModel::existGeomName (const std::string & name) const
  {
    return (geom_names.end() != std::find(geom_names.begin(),geom_names.end(),name));
  }
  
  inline const std::string& GeometryModel::getGeomName (const Index index) const
  {
    assert( index < (Index)collision_objects.size() );
    return geom_names[index];
  }

  inline void GeometryModel::addInnerObject (const Index joint, const Index inner_object)
  {
    if (std::find(innerObjects[joint].begin(), innerObjects[joint].end(),inner_object)==innerObjects[joint].end())
      innerObjects[joint].push_back(inner_object);
    else
      std::cout << "inner object already added" << std::endl;
  }

  inline void GeometryModel::addOutterObject (const Index joint, const Index outer_object)
  {
    if (std::find(outerObjects[joint].begin(), outerObjects[joint].end(),outer_object)==outerObjects[joint].end())
      outerObjects[joint].push_back(outer_object);
    else
      std::cout << "outer object already added" << std::endl;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryModel & model_geom)
  {
    os << "Nb collision objects = " << model_geom.ngeom << std::endl;
    
    for(GeometryModel::Index i=0;i<(GeometryModel::Index)(model_geom.ngeom);++i)
    {
      os  << "Collision object " << i << " : " << model_geom.geom_names[i] << ": attached to joint = " << model_geom.geom_parents[i]
          << "\nwith offset \n" << model_geom.geometryPlacement[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryData & data_geom)
  {

    for(GeometryData::Index i=0;i<(GeometryData::Index)(data_geom.model_geom.ngeom);++i)
    {
      os << "collision object in position " << data_geom.oMg[i] << std::endl;
    }

    return os;
  }

  inline void GeometryData::addCollisionPair (const Index co1, const Index co2)
  {
    assert ( co1 != co2);
    assert ( co2 < model_geom.ngeom);
    CollisionPair_t pair(co1, co2);
    
    addCollisionPair(pair);
  }

  inline void GeometryData::addCollisionPair (const CollisionPair_t & pair)
  {
    assert(pair.first < pair.second);
    assert(pair.second < model_geom.ngeom);
    
    if (!existCollisionPair(pair))
    {
      collision_pairs.push_back(pair);
      nCollisionPairs++;
    }
  }
  
  inline void GeometryData::addAllCollisionPairs()
  {
    removeAllCollisionPairs();
    collision_pairs.reserve((model_geom.ngeom * (model_geom.ngeom-1))/2);
    for (Index i = 0; i < model_geom.ngeom; ++i)
      for (Index j = i+1; j < model_geom.ngeom; ++j)
        addCollisionPair(i,j);
  }
  
  inline void GeometryData::removeCollisionPair (const Index co1, const Index co2)
  {
    assert(co1 < co2);
    assert(co2 < model_geom.ngeom);
    assert(existCollisionPair(co1,co2));

    removeCollisionPair (CollisionPair_t(co1,co2));
  }

  inline void GeometryData::removeCollisionPair (const CollisionPair_t & pair)
  {
    assert(pair.second < model_geom.ngeom);

    CollisionPairsVector_t::const_iterator it = std::find(collision_pairs.begin(), collision_pairs.end(), pair);
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

  inline bool GeometryData::existCollisionPair (const Index co1, const Index co2) const
  {
    return existCollisionPair(CollisionPair_t(co1,co2));
  }

  inline bool GeometryData::existCollisionPair (const CollisionPair_t & pair) const
  {
    return (std::find(collision_pairs.begin(), collision_pairs.end(), pair)
            != collision_pairs.end());
  }
  
  inline GeometryData::Index GeometryData::findCollisionPair (const Index co1, const Index co2) const
  {
    return findCollisionPair(CollisionPair_t(co1,co2));
  }
  
  inline GeometryData::Index GeometryData::findCollisionPair (const CollisionPair_t & pair) const
  {
    CollisionPairsVector_t::const_iterator it = std::find(collision_pairs.begin(), collision_pairs.end(), pair);
    
    return (Index) distance(collision_pairs.begin(), it);
  }

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

  inline bool GeometryData::collide(const Index co1, const Index co2) const
  {
    fcl::CollisionRequest collisionRequest (1, false, false, 1, false, true, fcl::GST_INDEP);
    fcl::CollisionResult collisionResult;

    if (fcl::collide (model_geom.collision_objects[co1].collisionGeometry().get(), oMg_fcl[co1],
                      model_geom.collision_objects[co2].collisionGeometry().get(), oMg_fcl[co2],
                      collisionRequest, collisionResult) != 0)
    {
      return true;
    }
    return false;
  }


  inline fcl::DistanceResult GeometryData::computeDistance(const Index co1, const Index co2) const
  {
    fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
    fcl::DistanceResult result;
    fcl::distance ( model_geom.collision_objects[co1].collisionGeometry().get(), oMg_fcl[co1],
                    model_geom.collision_objects[co2].collisionGeometry().get(), oMg_fcl[co2],
                    distanceRequest, result);
    return result;
  }

  inline void GeometryData::resetDistances()
  {
    std::fill(distance_results.begin(), distance_results.end(), DistanceResult( fcl::DistanceResult(), 0, 0) );
  }


} // namespace se3

#endif // ifndef __se3_geom_hxx__
