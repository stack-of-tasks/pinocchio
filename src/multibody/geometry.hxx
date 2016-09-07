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
  inline GeometryData::GeometryData(const GeometryModel & modelGeom)
    : model_geom(modelGeom)
    , oMg(model_geom.ngeoms)

#ifdef WITH_HPP_FCL   
    , activeCollisionPairs(modelGeom.collisionPairs.size(), true)
    , distanceRequest (true, 0, 0, fcl::GST_INDEP)
    , distance_results(modelGeom.collisionPairs.size())
    , collisionRequest (1, false, false, 1, false, true, fcl::GST_INDEP)
    , collision_results(modelGeom.collisionPairs.size())
    , radius()
    , collisionPairIndex(-1)
    , innerObjects()
    , outerObjects()
  {
    collisionObjects.reserve(modelGeom.geometryObjects.size());
    BOOST_FOREACH( const GeometryObject & geom, modelGeom.geometryObjects)
      { collisionObjects.push_back
          (fcl::CollisionObject(geom.collision_geometry)); }
    fillInnerOuterObjectMaps();
  }
#else
  {}
#endif // WITH_HPP_FCL   

  inline GeomIndex GeometryModel::addGeometryObject(const GeometryObject& object)
  {
    GeomIndex idx = (GeomIndex) (ngeoms ++);
    geometryObjects.push_back(object);
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


    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's inner objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an inner object
     */
  // inline void GeometryModel::addInnerObject(const JointIndex joint_id, const GeomIndex inner_object)
  // {
  //   if (std::find(innerObjects[joint_id].begin(),
  //                 innerObjects[joint_id].end(),
  //                 inner_object) == innerObjects[joint_id].end())
  //     innerObjects[joint_id].push_back(inner_object);
  //   else
  //     std::cout << "inner object already added" << std::endl;
  // }

    /**
     * @brief      Associate a GeometryObject of type COLLISION to a joint's outer objects list
     *
     * @param[in]  joint         Index of the joint
     * @param[in]  inner_object  Index of the GeometryObject that will be an outer object
     */
  // inline void GeometryModel::addOutterObject (const JointIndex joint, const GeomIndex outer_object)
  // {
  //   if (std::find(outerObjects[joint].begin(),
  //                 outerObjects[joint].end(),
  //                 outer_object) == outerObjects[joint].end())
  //     outerObjects[joint].push_back(outer_object);
  //   else
  //     std::cout << "outer object already added" << std::endl;
  // }

  inline void GeometryData::fillInnerOuterObjectMaps()
  {
    innerObjects.clear();
    outerObjects.clear();

    for( GeomIndex gid = 0; gid<model_geom.geometryObjects.size(); gid++)
      innerObjects[model_geom.geometryObjects[gid].parentJoint].push_back(gid);

    BOOST_FOREACH( const CollisionPair & pair, model_geom.collisionPairs )
      {
        outerObjects[model_geom.geometryObjects[pair.first].parentJoint].push_back(pair.second);
      }
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryModel & model_geom)
  {
    os << "Nb geometry objects = " << model_geom.ngeoms << std::endl;
    
    for(GeomIndex i=0;i<(GeomIndex)(model_geom.ngeoms);++i)
    {
      os  << model_geom.geometryObjects[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryData & data_geom)
  {
#ifdef WITH_HPP_FCL
    os << "Nb collision pairs = " << data_geom.activeCollisionPairs.size() << std::endl;
    
    for(PairIndex i=0;i<(PairIndex)(data_geom.activeCollisionPairs.size());++i)
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
    for (GeomIndex i = 0; i < ngeoms; ++i)
    {
      const JointIndex& joint_i = geometryObjects[i].parentJoint;
      for (GeomIndex j = i+1; j < ngeoms; ++j)
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
  
  inline PairIndex GeometryModel::findCollisionPair (const CollisionPair & pair) const
  {
    CollisionPairsVector_t::const_iterator it = std::find(collisionPairs.begin(),
                                                          collisionPairs.end(),
                                                          pair);
    
    return (PairIndex) std::distance(collisionPairs.begin(), it);
  }

  inline void GeometryData::activateCollisionPair(const PairIndex pairId,const bool flag)
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = flag;
  }

  inline void GeometryData::deactivateCollisionPair(const PairIndex pairId)
  {
    assert( activeCollisionPairs.size() == model_geom.collisionPairs.size() );
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = false;
  }

  inline void GeometryData::resetDistances()
  {
    std::fill(distance_results.begin(), distance_results.end(), fcl::DistanceResult() );
  }
#endif //WITH_HPP_FCL
} // namespace se3

/// @endcond

#endif // ifndef __se3_geometry_hxx__
