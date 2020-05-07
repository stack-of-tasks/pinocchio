//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_geometry_hxx__
#define __pinocchio_multibody_geometry_hxx__

#include "pinocchio/multibody/model.hpp"

/// @cond DEV

namespace pinocchio
{
// Avoid deprecated warning of collisionObjects
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  inline GeometryData::GeometryData(const GeometryModel & geom_model)
  : oMg(geom_model.ngeoms)
  , activeCollisionPairs(geom_model.collisionPairs.size(), true)
#ifdef PINOCCHIO_WITH_HPP_FCL
  , distanceRequest(true)
  , distanceRequests(geom_model.collisionPairs.size(), hpp::fcl::DistanceRequest(true))
  , distanceResults(geom_model.collisionPairs.size())
  , collisionRequest(::hpp::fcl::NO_REQUEST,1)
  , collisionRequests(geom_model.collisionPairs.size(), hpp::fcl::CollisionRequest(::hpp::fcl::NO_REQUEST,1))
  , collisionResults(geom_model.collisionPairs.size())
  , radius()
  , collisionPairIndex(0)
#endif // PINOCCHIO_WITH_HPP_FCL
  , innerObjects()
  , outerObjects()
  {
#ifdef PINOCCHIO_WITH_HPP_FCL
    collisionObjects.reserve(geom_model.geometryObjects.size());
    BOOST_FOREACH(const GeometryObject & geom_object, geom_model.geometryObjects)
    {
      collisionObjects.push_back(fcl::CollisionObject(geom_object.geometry));
    }
    BOOST_FOREACH(hpp::fcl::CollisionRequest & creq, collisionRequests)
    {
      creq.enable_cached_gjk_guess = true;
    }
#if HPP_FCL_VERSION_AT_LEAST(1, 4, 5)
    BOOST_FOREACH(hpp::fcl::DistanceRequest & dreq, distanceRequests)
    {
      dreq.enable_cached_gjk_guess = true;
    }
#endif
#endif
    fillInnerOuterObjectMaps(geom_model);
  }

  inline GeometryData::GeometryData(const GeometryData & other)
  : oMg (other.oMg)
  , activeCollisionPairs (other.activeCollisionPairs)
#ifdef PINOCCHIO_WITH_HPP_FCL
  , collisionObjects (other.collisionObjects)
  , distanceRequest (other.distanceRequest)
  , distanceRequests (other.distanceRequests)
  , distanceResults (other.distanceResults)
  , collisionRequest (other.collisionRequest)
  , collisionRequests (other.collisionRequests)
  , collisionResults (other.collisionResults)
  , radius (other.radius)
  , collisionPairIndex (other.collisionPairIndex)
#endif // PINOCCHIO_WITH_HPP_FCL
  , innerObjects (other.innerObjects)
  , outerObjects (other.outerObjects)
  {}

  inline GeometryData::~GeometryData() {}
#pragma GCC diagnostic pop

  template<typename S2, int O2, template<typename,int> class JointCollectionTpl>
  GeomIndex GeometryModel::addGeometryObject(const GeometryObject & object,
                                             const ModelTpl<S2,O2,JointCollectionTpl> & model)
  {
    if(object.parentFrame < (FrameIndex)model.nframes)
      PINOCCHIO_CHECK_INPUT_ARGUMENT(model.frames[object.parentFrame].parent == object.parentJoint,
                                     "The object joint parent and its frame joint parent do not match.");
    
    GeomIndex idx = (GeomIndex) (ngeoms ++);
    geometryObjects.push_back(object);
    geometryObjects.back().parentJoint = model.frames[object.parentFrame].parent;
    return idx;
  }
  
  inline GeomIndex GeometryModel::addGeometryObject(const GeometryObject & object)
  {
    GeomIndex idx = (GeomIndex) (ngeoms ++);
    geometryObjects.push_back(object);
    return idx;
  }

  inline GeomIndex GeometryModel::getGeometryId(const std::string & name) const
  {

    GeometryObjectVector::const_iterator it
    = std::find_if(geometryObjects.begin(),
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

  inline void GeometryData::fillInnerOuterObjectMaps(const GeometryModel & geomModel)
  {
    innerObjects.clear();
    outerObjects.clear();

    for( GeomIndex gid = 0; gid<geomModel.geometryObjects.size(); gid++)
      innerObjects[geomModel.geometryObjects[gid].parentJoint].push_back(gid);

    BOOST_FOREACH(const CollisionPair & pair, geomModel.collisionPairs)
    {
      outerObjects[geomModel.geometryObjects[pair.first].parentJoint].push_back(pair.second);
    }
  }

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
#ifdef PINOCCHIO_WITH_HPP_FCL
    os << "Number of collision pairs = " << geomData.activeCollisionPairs.size() << std::endl;
    
    for(PairIndex i=0;i<(PairIndex)(geomData.activeCollisionPairs.size());++i)
    {
      os << "Pairs " << i << (geomData.activeCollisionPairs[i]?" active":" inactive") << std::endl;
    }
#else
    os << "WARNING** Without fcl library, no collision checking or distance computations are possible. Only geometry placements can be computed." << std::endl;
    os << "Number of geometry objects = " << geomData.oMg.size() << std::endl;
#endif

    return os;
  }

  inline void GeometryModel::addCollisionPair(const CollisionPair & pair)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.first < ngeoms,
                                   "The input pair.first is larger than the number of geometries contained in the GeometryModel");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.second < ngeoms,
                                   "The input pair.second is larger than the number of geometries contained in the GeometryModel");
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
  
  inline void GeometryModel::removeCollisionPair(const CollisionPair & pair)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.first < ngeoms,
                                   "The input pair.first is larger than the number of geometries contained in the GeometryModel");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pair.second < ngeoms,
                                   "The input pair.second is larger than the number of geometries contained in the GeometryModel");

    CollisionPairVector::iterator it = std::find(collisionPairs.begin(),
                                                 collisionPairs.end(),
                                                 pair);
    if (it != collisionPairs.end()) { collisionPairs.erase(it); }
  }
  
  inline void GeometryModel::removeAllCollisionPairs () { collisionPairs.clear(); }

  inline bool GeometryModel::existCollisionPair(const CollisionPair & pair) const
  {
    return (std::find(collisionPairs.begin(),
                      collisionPairs.end(),
                      pair) != collisionPairs.end());
  }
  
  inline PairIndex GeometryModel::findCollisionPair(const CollisionPair & pair) const
  {
    CollisionPairVector::const_iterator it = std::find(collisionPairs.begin(),
                                                       collisionPairs.end(),
                                                       pair);
    
    return (PairIndex) std::distance(collisionPairs.begin(), it);
  }
  
  inline void GeometryData::activateCollisionPair(const PairIndex pairId)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pairId < activeCollisionPairs.size(),
                                   "The input argument pairId is larger than the number of collision pairs contained in activeCollisionPairs.");
    activeCollisionPairs[pairId] = true;
  }

  inline void GeometryData::deactivateCollisionPair(const PairIndex pairId)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(pairId < activeCollisionPairs.size(),
                                   "The input argument pairId is larger than the number of collision pairs contained in activeCollisionPairs.");
    activeCollisionPairs[pairId] = false;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_multibody_geometry_hxx__
