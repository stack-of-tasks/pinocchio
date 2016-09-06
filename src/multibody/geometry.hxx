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



#include <iostream>


#include <map>
#include <list>
#include <utility>

/// @cond DEV

namespace se3
{

  inline GeomIndex GeometryModel::addGeometryObject(const GeometryObject& object)
  {
    Index idx = (Index) (ngeoms ++);
    geometryObjects.push_back(object);
    addInnerObject(object.parentJoint, idx);
    return idx;
  }

  inline GeomIndex GeometryModel::addGeometryObject(const Model& model,
                                                    const FrameIndex parent,
                                                    const boost::shared_ptr<fcl::CollisionGeometry> & co,
                                                    const SE3 & placement,
                                                    const std::string & geom_name,
                                                    const std::string & mesh_path) throw(std::invalid_argument)
  {
    assert (model.frames[parent].type == se3::BODY);
    JointIndex parentJoint = model.frames[parent].parent;
    GeometryObject object( geom_name, parent, parentJoint, co,
                           placement, mesh_path);
    return addGeometryObject (object);
  }


  inline GeomIndex GeometryModel::getGeometryId(const std::string & name) const
  {

    std::vector<GeometryObject>::const_iterator it = std::find_if(geometryObjects.begin(),
                                                                  geometryObjects.end(),
                                                                  boost::bind(&GeometryObject::name, _1) == name
                                                                  );
    return GeomIndex(it - geometryObjects.begin());
  }



  inline bool GeometryModel::existGeometryName(const std::string & name) const
  {
    return std::find_if(geometryObjects.begin(),
                        geometryObjects.end(),
                        boost::bind(&GeometryObject::name, _1) == name) != geometryObjects.end();
  }


  inline const std::string& GeometryModel::getGeometryName(const GeomIndex index) const
  {
    assert( index < (GeomIndex)geometryObjects.size() );
    return geometryObjects[index].name;
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
    
    for(Index i=0;i<(Index)(model_geom.ngeoms);++i)
    {
      os  << model_geom.geometryObjects[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryData & data_geom)
  {
#ifdef WITH_HPP_FCL
    os << "Nb collision pairs = " << data_geom.activeCollisionPairs.size() << std::endl;
    
    for(Index i=0;i<(Index)(data_geom.activeCollisionPairs.size());++i)
    {
      os << "collision object in position " << data_geom.model_geom.collisionPairs[i] << std::endl;
    }
#else
    os << "WARNING** Without fcl, no collision computations are possible. Only Positions can be computed" << std::endl;
    os << "Nb of geometry objects = " << data_geom.oMg.size() << std::endl;
#endif

    return os;
  }

#ifdef WITH_HPP_FCL
  
  inline void GeometryModel::addCollisionPair (const CollisionPair & pair)
  {
    assert( (pair.first < ngeoms) && (pair.second < ngeoms) );
    if (!existCollisionPair(pair)) { collisionPairs.push_back(pair); }
  }
  
  inline void GeometryModel::addAllCollisionPairs()
  {
    removeAllCollisionPairs();
    for (Index i = 0; i < ngeoms; ++i)
    {
      const JointIndex& joint_i = geometryObjects[i].parentJoint;
      for (Index j = i+1; j < ngeoms; ++j)
      {
        const JointIndex& joint_j = geometryObjects[j].parentJoint;
        if (joint_i != joint_j)
          addCollisionPair(CollisionPair(i,j));
      }
    }
  }
  
  inline void GeometryModel::removeCollisionPair (const CollisionPair & pair)
  {
    assert( (pair.first < ngeoms) && (pair.second < ngeoms) );

    CollisionPairsVector_t::iterator it = std::find(collisionPairs.begin(),
                                                    collisionPairs.end(),
                                                    pair);
    if (it != collisionPairs.end()) { collisionPairs.erase(it); }
  }
  
  inline void GeometryModel::removeAllCollisionPairs () { collisionPairs.clear(); }

  inline bool GeometryModel::existCollisionPair (const CollisionPair & pair) const
  {
    return (std::find(collisionPairs.begin(),
                      collisionPairs.end(),
                      pair) != collisionPairs.end());
  }
  
  inline Index GeometryModel::findCollisionPair (const CollisionPair & pair) const
  {
    CollisionPairsVector_t::const_iterator it = std::find(collisionPairs.begin(),
                                                          collisionPairs.end(),
                                                          pair);
    
    return (Index) std::distance(collisionPairs.begin(), it);
  }

  inline void GeometryModel::displayCollisionPairs() const
  {
    for (CollisionPairsVector_t::const_iterator it = collisionPairs.begin(); 
         it != collisionPairs.end(); ++it)
      {
        std::cout << it-> first << "\t" << it->second << std::endl;
      }
  }

  inline void GeometryData::activateCollisionPair(const Index pairId,const bool flag)
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = flag;
  }

  inline void GeometryData::deactivateCollisionPair(const Index pairId)
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = false;
  }

  inline bool GeometryData::computeCollision(const Index& pairId)
  {
    const CollisionPair & pair = model_geom.collisionPairs[pairId];
    fcl::CollisionResult& collisionResult = collision_results[pairId];

    const Index & co1 = pair.first;     assert(co1<collisionObjects.size());
    const Index & co2 = pair.second;    assert(co2<collisionObjects.size());

    collisionResult.clear();
    fcl::collide (&collisionObjects[co1],&collisionObjects[co2],
                  collisionRequest, collisionResult);

    return collisionResult.isCollision();
  }
  
  inline void GeometryData::computeAllCollisions()
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    assert( collision_results   .size() == model_geom.collisionPairs.size() );
    for(size_t i = 0; i<model_geom.collisionPairs.size(); ++i)
    {
      if(activeCollisionPairs[i])
        computeCollision(i);
    }
  }
  
  inline bool GeometryData::isColliding()
  {
    Index& i = collisionPairIndex;
    for(i = 0; i<model_geom.collisionPairs.size(); ++i)
    {
      if (activeCollisionPairs[i] && computeCollision(i))
        return true;
    }
    return false;
  }

  inline fcl::DistanceResult & GeometryData::computeDistance(const Index & pairId )
  {
    assert( pairId < model_geom.collisionPairs.size() );
    const CollisionPair & pair = model_geom.collisionPairs[pairId];

    assert( pair < distance_results.size() );
    assert( pair.first  < collisionObjects.size() );
    assert( pair.second < collisionObjects.size() );
    
    fcl::distance ( &collisionObjects[pair.first],&collisionObjects[pair.second],
                    distanceRequest, distance_results[pairId]);

    return distance_results[pairId];
  }
  
  inline void GeometryData::computeAllDistances ()
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    for(size_t i = 0; i<activeCollisionPairs.size(); ++i)
    {
      if (activeCollisionPairs[i]) computeDistance(i);
    }
  }

  inline void GeometryData::resetDistances()
  {
    std::fill(distance_results.begin(), distance_results.end(), fcl::DistanceResult() );
  }
#endif //WITH_HPP_FCL
} // namespace se3

/// @endcond

#endif // ifndef __se3_geometry_hxx__
