//
// Copyright (c) 2015-2022 CNRS INRIA
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
      for (Index j = 0; j < nGeom2; ++j) {
        geom_model1.collisionPairs.push_back(CollisionPair(i, nGeom1 + j));
      }
    }
  }

} // namespace pinocchio

#endif // ifnded __pinocchio_algo_geometry_hxx__
