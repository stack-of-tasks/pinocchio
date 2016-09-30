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
    : oMg(modelGeom.ngeoms)

#ifdef WITH_HPP_FCL   
    , activeCollisionPairs(modelGeom.collisionPairs.size(), true)
    , distanceRequest (true, 0, 0, fcl::GST_INDEP)
    , distanceResults(modelGeom.collisionPairs.size())
    , collisionRequest (1, false, false, 1, false, true, fcl::GST_INDEP)
    , collisionResults(modelGeom.collisionPairs.size())
    , radius()
    , collisionPairIndex(0)
    , innerObjects()
    , outerObjects()
  {
    collisionObjects.reserve(modelGeom.geometryObjects.size());
    BOOST_FOREACH( const GeometryObject & geom, modelGeom.geometryObjects)
      { collisionObjects.push_back
          (fcl::CollisionObject(geom.fcl)); }
    fillInnerOuterObjectMaps(modelGeom);
  }
#else
  {}
#endif // WITH_HPP_FCL   

  inline GeomIndex GeometryModel::addGeometryObject(GeometryObject object,
                                                    const Model & model,
                                                    const bool autofillJointParent)
  {
    // TODO reenable when relevant: assert( (object.parentFrame != -1) || (object.parentJoint != -1) );

    if( autofillJointParent )
      // TODO: this might be automatically done for some default value of parentJoint (eg ==-1)
      object.parentJoint = model.frames[object.parentFrame].parent; 

    assert( //TODO: reenable when relevant (object.parentFrame == -1) ||
           (model.frames[object.parentFrame].type == se3::BODY)  );
    assert( //TODO: reenable when relevant (object.parentFrame == -1) ||
           (model.frames[object.parentFrame].parent == object.parentJoint) );

    GeomIndex idx = (GeomIndex) (ngeoms ++);
    geometryObjects.push_back(object);
    return idx;
  }

  inline GeomIndex GeometryModel::getGeometryId(const std::string & name) const
  {

    container::aligned_vector<GeometryObject>::const_iterator it = std::find_if(geometryObjects.begin(),
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

#ifdef WITH_HPP_FCL
  inline void GeometryData::fillInnerOuterObjectMaps(const GeometryModel & geomModel)
  {
    innerObjects.clear();
    outerObjects.clear();

    for( GeomIndex gid = 0; gid<geomModel.geometryObjects.size(); gid++)
      innerObjects[geomModel.geometryObjects[gid].parentJoint].push_back(gid);

    BOOST_FOREACH( const CollisionPair & pair, geomModel.collisionPairs )
      {
        outerObjects[geomModel.geometryObjects[pair.first].parentJoint].push_back(pair.second);
      }
  }
#endif

  inline std::ostream & operator<< (std::ostream & os, const GeometryModel & geomModel)
  {
    os << "Nb geometry objects = " << geomModel.ngeoms << std::endl;
    
    for(GeomIndex i=0;i<(GeomIndex)(geomModel.ngeoms);++i)
    {
      os  << geomModel.geometryObjects[i] <<std::endl;
    }

    return os;
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryData & geomData)
  {
#ifdef WITH_HPP_FCL
    os << "Nb collision pairs = " << geomData.activeCollisionPairs.size() << std::endl;
    
    for(PairIndex i=0;i<(PairIndex)(geomData.activeCollisionPairs.size());++i)
    {
      os << "Pairs " << i << (geomData.activeCollisionPairs[i]?"active":"unactive") << std::endl;
    }
#else
    os << "WARNING** Without fcl, no collision computations are possible. Only Positions can be computed" << std::endl;
    os << "Nb of geometry objects = " << geomData.oMg.size() << std::endl;
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
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = flag;
  }

  inline void GeometryData::deactivateCollisionPair(const PairIndex pairId)
  {
    assert( pairId < activeCollisionPairs.size() );
    activeCollisionPairs[pairId] = false;
  }

#endif //WITH_HPP_FCL
} // namespace se3

/// @endcond

#endif // ifndef __se3_geometry_hxx__
