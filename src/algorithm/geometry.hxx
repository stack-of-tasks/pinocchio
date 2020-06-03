//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_algo_geometry_hxx__
#define __pinocchio_algo_geometry_hxx__

#include <boost/foreach.hpp>

namespace pinocchio 
{
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void updateGeometryPlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const GeometryModel & geom_model,
                                       GeometryData & geom_data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    forwardKinematics(model, data, q);
    updateGeometryPlacements(model, data, geom_model, geom_data);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void updateGeometryPlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const GeometryModel & geom_model,
                                       GeometryData & geom_data)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    assert(model.check(data) && "data is not consistent with model.");
    
    for (GeomIndex i=0; i < (GeomIndex) geom_model.ngeoms; ++i)
    {
      const Model::JointIndex & joint = geom_model.geometryObjects[i].parentJoint;
      if (joint>0) geom_data.oMg[i] =  (data.oMi[joint] * geom_model.geometryObjects[i].placement);
      else         geom_data.oMg[i] =  geom_model.geometryObjects[i].placement;
#ifdef PINOCCHIO_WITH_HPP_FCL  
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      geom_data.collisionObjects[i].setTransform( toFclTransform3f(geom_data.oMg[i]) );
#pragma GCC diagnostic pop
#endif // PINOCCHIO_WITH_HPP_FCL
    }
  }
#ifdef PINOCCHIO_WITH_HPP_FCL  

  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */

  inline bool computeCollision(const GeometryModel & geom_model,
                               GeometryData & geom_data,
                               const PairIndex & pairId)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pairId < geom_model.collisionPairs.size() );
    const CollisionPair & pair = geom_model.collisionPairs[pairId];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pairId      < geom_data.collisionResults.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );

    fcl::CollisionResult& collisionResult = geom_data.collisionResults[pairId];
    collisionResult.clear();

    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));

    fcl::collide (geom_model.geometryObjects[pair.first ].geometry.get(), oM1,
                  geom_model.geometryObjects[pair.second].geometry.get(), oM2,
                  geom_data.collisionRequests[pairId],
                  collisionResult);

    return collisionResult.isCollision();
  }
  
  inline bool computeCollisions(const GeometryModel & geom_model,
                                GeometryData & geom_data,
                                const bool stopAtFirstCollision)
  {
    bool isColliding = false;
    
    for (std::size_t cpt = 0; cpt < geom_model.collisionPairs.size(); ++cpt)
    {
      if(geom_data.activeCollisionPairs[cpt])
      {
        computeCollision(geom_model,geom_data,cpt);
        if(!isColliding && geom_data.collisionResults[cpt].isCollision())
        {
          isColliding = true;
          geom_data.collisionPairIndex = cpt; // first pair to be in collision
          if(stopAtFirstCollision)
            return true;
        }
      }
    }
    
    return isColliding;
  }
  
  // WARNING, if stopAtFirstcollision = true, then the collisions vector will not be fulfilled.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline bool computeCollisions(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const GeometryModel & geom_model,
                                GeometryData & geom_data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const bool stopAtFirstCollision)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    updateGeometryPlacements(model, data, geom_model, geom_data, q);
    
    return computeCollisions(geom_model,geom_data, stopAtFirstCollision);
  }

  /* --- DISTANCES ----------------------------------------------------------------- */
  /* --- DISTANCES ----------------------------------------------------------------- */
  /* --- DISTANCES ----------------------------------------------------------------- */

  inline fcl::DistanceResult & computeDistance(const GeometryModel & geom_model,
                                               GeometryData & geom_data,
                                               const PairIndex & pairId)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pairId < geom_model.collisionPairs.size() );
    const CollisionPair & pair = geom_model.collisionPairs[pairId];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pairId      < geom_data.distanceResults.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );

    geom_data.distanceResults[pairId].clear();
    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));
    fcl::distance ( geom_model.geometryObjects[pair.first ].geometry.get(), oM1,
                    geom_model.geometryObjects[pair.second].geometry.get(), oM2,
                    geom_data.distanceRequests[pairId],
                    geom_data.distanceResults[pairId]);

    return geom_data.distanceResults[pairId];
  }
  
  inline std::size_t computeDistances(const GeometryModel & geom_model,
                                      GeometryData & geom_data)
  {
    std::size_t min_index = geom_model.collisionPairs.size();
    double min_dist = std::numeric_limits<double>::infinity();
    for (std::size_t cpt = 0; cpt < geom_model.collisionPairs.size(); ++cpt)
    {
      if(geom_data.activeCollisionPairs[cpt])
      {
        computeDistance(geom_model,geom_data,cpt);
        if(geom_data.distanceResults[cpt].min_distance < min_dist)
        {
          min_index = cpt;
          min_dist = geom_data.distanceResults[cpt].min_distance;
        }
      }
    }
    return min_index;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline std::size_t computeDistances(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const GeometryModel & geom_model,
                                      GeometryData & geom_data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    updateGeometryPlacements(model,data,geom_model,geom_data);
    return computeDistances(geom_model,geom_data);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline std::size_t computeDistances(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const GeometryModel & geom_model,
                                      GeometryData & geom_data,
                                      const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    updateGeometryPlacements(model, data, geom_model, geom_data, q);
    return computeDistances(geom_model,geom_data);
  }

  /* --- RADIUS -------------------------------------------------------------------- */
  /* --- RADIUS -------------------------------------------------------------------- */
  /* --- RADIUS -------------------------------------------------------------------- */

  /// Given p1..3 being either "min" or "max", return one of the corners of the 
  /// AABB cub of the FCL object.
#define PINOCCHIO_GEOM_AABB(FCL,p1,p2,p3)                                     \
  SE3::Vector3(                                                         \
    FCL->aabb_local.p1##_ [0],                                          \
    FCL->aabb_local.p2##_ [1],                                          \
    FCL->aabb_local.p3##_ [2])

  /// For all bodies of the model, compute the point of the geometry model
  /// that is the further from the center of the joint. This quantity is used 
  /// in some continuous collision test.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void computeBodyRadius(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const GeometryModel & geom_model,
                                GeometryData & geom_data)
  {
    geom_data.radius.resize(model.joints.size(),0);
    BOOST_FOREACH(const GeometryObject & geom_object,geom_model.geometryObjects)
    {
      const GeometryObject::CollisionGeometryPtr & geometry
        = geom_object.geometry;
      const GeometryModel::SE3 & jMb = geom_object.placement; // placement in joint.
      const Model::JointIndex & i = geom_object.parentJoint;
      assert (i<geom_data.radius.size());

      double radius = geom_data.radius[i] * geom_data.radius[i];

      // The radius is simply the one of the 8 corners of the AABB cube, expressed 
      // in the joint frame, whose norm is the highest.
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,min,min,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,min,min,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,min,max,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,min,max,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,max,min,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,max,min,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,max,max,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(PINOCCHIO_GEOM_AABB(geometry,max,max,max)).squaredNorm(),radius);

      // Don't forget to sqroot the squared norm before storing it.
      geom_data.radius[i] = sqrt(radius);
    }
  }

#undef PINOCCHIO_GEOM_AABB
#endif // PINOCCHIO_WITH_HPP_FCL

  /* --- APPEND GEOMETRY MODEL ----------------------------------------------------------- */

  inline void appendGeometryModel(GeometryModel & geom_model1,
                                  const GeometryModel & geom_model2)
  {
    assert (geom_model1.ngeoms == geom_model1.geometryObjects.size());
    Index nGeom1 = geom_model1.geometryObjects.size();
    Index nColPairs1 = geom_model1.collisionPairs.size();
    assert (geom_model2.ngeoms == geom_model2.geometryObjects.size());
    Index nGeom2 = geom_model2.geometryObjects.size();
    Index nColPairs2 = geom_model2.collisionPairs.size();

    /// Append the geometry objects and geometry positions
    geom_model1.geometryObjects.insert(geom_model1.geometryObjects.end(),
        geom_model2.geometryObjects.begin(), geom_model2.geometryObjects.end());
    geom_model1.ngeoms += nGeom2;

    /// 1. copy the collision pairs and update geom_data1 accordingly.
    geom_model1.collisionPairs.reserve(nColPairs1 + nColPairs2 + nGeom1 * nGeom2);
    for (Index i = 0; i < nColPairs2; ++i)
    {
      const CollisionPair& cp = geom_model2.collisionPairs[i];
      geom_model1.collisionPairs.push_back(
          CollisionPair (cp.first + nGeom1, cp.second + nGeom1)
          );
    }

    /// 2. add the collision pairs between geom_model1 and geom_model2.
    for (Index i = 0; i < nGeom1; ++i) {
      for (Index j = 0; j < nGeom2; ++j) {
        geom_model1.collisionPairs.push_back(CollisionPair(i, nGeom1 + j));
      }
    }
  }

} // namespace pinocchio

#endif // ifnded __pinocchio_algo_geometry_hxx__
