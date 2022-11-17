//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_geometry_hxx__
#define __pinocchio_algo_geometry_hxx__

#include <boost/foreach.hpp>
#include <sstream>

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
    
    for(GeomIndex i=0; i < (GeomIndex) geom_model.ngeoms; ++i)
    {
      const Model::JointIndex joint_id = geom_model.geometryObjects[i].parentJoint;
      if (joint_id>0) geom_data.oMg[i] =  (data.oMi[joint_id] * geom_model.geometryObjects[i].placement);
      else            geom_data.oMg[i] =  geom_model.geometryObjects[i].placement;
    }
  }
#ifdef PINOCCHIO_WITH_HPP_FCL  

  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */
  /* --- COLLISIONS ----------------------------------------------------------------- */

  inline bool computeCollision(const GeometryModel & geom_model,
                               GeometryData & geom_data,
                               const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( geom_model.collisionPairs.size() == geom_data.collisionResults.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair_id < geom_model.collisionPairs.size() );
    const CollisionPair & pair = geom_model.collisionPairs[pair_id];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );

    fcl::CollisionRequest & collision_request = geom_data.collisionRequests[pair_id];
    collision_request.distance_upper_bound = collision_request.security_margin + 1e-6; // TODO: change the margin
    
    fcl::CollisionResult & collision_result = geom_data.collisionResults[pair_id];
    collision_result.clear();

    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));

    try
    {
      GeometryData::ComputeCollision & do_computations = geom_data.collision_functors[pair_id];
      do_computations(oM1, oM2, collision_request, collision_result);
    }
    catch(std::invalid_argument & e)
    {
      std::stringstream ss;
      ss << "Problem when trying to check the collision of collision pair #" << pair_id << " (" << pair.first << "," << pair.second << ")" << std::endl;
      ss << "hpp-fcl original error:\n" << e.what() << std::endl;
      throw std::invalid_argument(ss.str());
    }
    

    return collision_result.isCollision();
  }
  
  inline bool computeCollisions(const GeometryModel & geom_model,
                                GeometryData & geom_data,
                                const bool stopAtFirstCollision)
  {
    bool isColliding = false;
    
    for (std::size_t cp_index = 0;
         cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
      const CollisionPair & cp = geom_model.collisionPairs[cp_index];
      
      if(geom_data.activeCollisionPairs[cp_index]
         && !(geom_model.geometryObjects[cp.first].disableCollision || geom_model.geometryObjects[cp.second].disableCollision))
      {
        bool res = computeCollision(geom_model,geom_data,cp_index);
        if(!isColliding && res)
        {
          isColliding = true;
          geom_data.collisionPairIndex = cp_index; // first pair to be in collision
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
    updateGeometryPlacements(model, data, geom_model, geom_data, q);
    return computeCollisions(geom_model,geom_data, stopAtFirstCollision);
  }

  /* --- DISTANCES ----------------------------------------------------------------- */
  /* --- DISTANCES ----------------------------------------------------------------- */
  /* --- DISTANCES ----------------------------------------------------------------- */

  inline fcl::DistanceResult & computeDistance(const GeometryModel & geom_model,
                                               GeometryData & geom_data,
                                               const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair_id < geom_model.collisionPairs.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( geom_model.collisionPairs.size() == geom_data.collisionResults.size() );
    const CollisionPair & pair = geom_model.collisionPairs[pair_id];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );

    fcl::DistanceRequest & distance_request = geom_data.distanceRequests[pair_id];
    fcl::DistanceResult & distance_result = geom_data.distanceResults[pair_id];
    distance_result.clear();
    
    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));
    
    try
    {
      GeometryData::ComputeDistance & do_computations = geom_data.distance_functors[pair_id];
      do_computations(oM1, oM2, distance_request, distance_result);
    }
    catch(std::invalid_argument & e)
    {
      std::stringstream ss;
      ss << "Problem when trying to compute the distance of collision pair #" << pair_id << " (" << pair.first << "," << pair.second << ")" << std::endl;
      ss << "hpp-fcl original error:\n" << e.what() << std::endl;
      throw std::invalid_argument(ss.str());
    }

    return geom_data.distanceResults[pair_id];
  }
  
  inline std::size_t computeDistances(const GeometryModel & geom_model,
                                      GeometryData & geom_data)
  {
    std::size_t min_index = geom_model.collisionPairs.size();
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (std::size_t cp_index = 0;
         cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
      const CollisionPair & cp = geom_model.collisionPairs[cp_index];
      
      if(   geom_data.activeCollisionPairs[cp_index]
         && !(geom_model.geometryObjects[cp.first].disableCollision || geom_model.geometryObjects[cp.second].disableCollision))
      {
        computeDistance(geom_model,geom_data,cp_index);
        if(geom_data.distanceResults[cp_index].min_distance < min_dist)
        {
          min_index = cp_index;
          min_dist = geom_data.distanceResults[cp_index].min_distance;
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
      
      // Force computation of the Local AABB
      // TODO: change for a more elegant solution
      const_cast<hpp::fcl::CollisionGeometry&>(*geometry).computeLocalAABB();
      
      const GeometryModel::SE3 & jMb = geom_object.placement; // placement in joint.
      const Model::JointIndex i = geom_object.parentJoint;
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
      Index parent_i = geom_model1.geometryObjects[i].parentJoint;
      for (Index j = nGeom1; j < nGeom1 + nGeom2; ++j) {
        if (parent_i != geom_model1.geometryObjects[j].parentJoint)
          geom_model1.collisionPairs.push_back(CollisionPair(i, j));
      }
    }
  }

} // namespace pinocchio

#endif // ifnded __pinocchio_algo_geometry_hxx__
