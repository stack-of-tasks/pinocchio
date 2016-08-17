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

#ifndef __se3_algo_geometry_hxx__
#define __se3_algo_geometry_hxx__

#include <boost/foreach.hpp>

namespace se3 
{
  
  inline void updateGeometryPlacements(const Model & model,
                                      Data & data,
                                      const GeometryModel & model_geom,
                                      GeometryData & data_geom,
                                      const Eigen::VectorXd & q
                                      )
  {
    forwardKinematics(model, data, q);
    updateGeometryPlacements(model, data, model_geom, data_geom);
  }
  
  inline void  updateGeometryPlacements(const Model &,
                                       const Data & data,
                                       const GeometryModel & model_geom,
                                       GeometryData & data_geom
                                       )
  {
    for (GeomIndex i=0; i < (GeomIndex) data_geom.model_geom.ngeoms; ++i)
    {
      const Model::JointIndex & joint = model_geom.geometryObjects[i].parentJoint;
      if (joint>0) data_geom.oMg[i] =  (data.oMi[joint] * model_geom.geometryObjects[i].placement);
      else         data_geom.oMg[i] =  model_geom.geometryObjects[i].placement;
#ifdef WITH_HPP_FCL  
      data_geom.collisionObjects[i].setTransform( toFclTransform3f(data_geom.oMg[i]) );
#endif // WITH_HPP_FCL
    }
  }
#ifdef WITH_HPP_FCL  
  inline bool computeCollisions(GeometryData & data_geom,
                                const bool stopAtFirstCollision
                                )
  {
    bool isColliding = false;
    const GeometryModel & geomModel = data_geom.model_geom;
    
    for (std::size_t cpt = 0; cpt < geomModel.collisionPairs.size(); ++cpt)
    {
      if(data_geom.activeCollisionPairs[cpt])
        {
          data_geom.collision_results[cpt]
            = data_geom.computeCollision(geomModel.collisionPairs[cpt]);
          isColliding |= data_geom.collision_results[cpt].fcl_collision_result.isCollision();
          if(isColliding && stopAtFirstCollision)
            return true;
        }
    }
    
    return isColliding;
  }
  
  // WARNING, if stopAtFirstcollision = true, then the collisions vector will not be fulfilled.
  inline bool computeCollisions(const Model & model,
                                Data & data,
                                const GeometryModel & model_geom,
                                GeometryData & data_geom,
                                const Eigen::VectorXd & q,
                                const bool stopAtFirstCollision
                                )
  {
    updateGeometryPlacements (model, data, model_geom, data_geom, q);
    
    return computeCollisions(data_geom, stopAtFirstCollision);
  }

  // Required to have a default template argument on templated free function
  inline std::size_t computeDistances(GeometryData & data_geom)
  {
    return computeDistances<true>(data_geom);
  }
  
  template <bool COMPUTE_SHORTEST>
  inline std::size_t computeDistances(GeometryData & data_geom)
  {
    const GeometryModel & geomModel = data_geom.model_geom;
    std::size_t min_index = geomModel.collisionPairs.size();
    double min_dist = std::numeric_limits<double>::infinity();
    for (std::size_t cpt = 0; cpt < geomModel.collisionPairs.size(); ++cpt)
    {
      if(data_geom.activeCollisionPairs[cpt])
        {
          data_geom.distance_results[cpt] = data_geom.computeDistance(geomModel.collisionPairs[cpt]);
          if (COMPUTE_SHORTEST && data_geom.distance_results[cpt].distance() < min_dist)
            {
              min_index = cpt;
              min_dist = data_geom.distance_results[cpt].distance();
            }
        }
    }
    return min_index;
  }
  
  // Required to have a default template argument on templated free function
  inline std::size_t computeDistances(const Model & model,
                               Data & data,
                               const GeometryModel & model_geom,
                               GeometryData & data_geom,
                               const Eigen::VectorXd & q
                               )
  {
    return computeDistances<true>(model, data, model_geom, data_geom, q);
  }

  template <bool ComputeShortest>
  inline std::size_t computeDistances(const Model & model,
                               Data & data,
                               const GeometryModel & model_geom,
                               GeometryData & data_geom,
                               const Eigen::VectorXd & q
                               )
  {
    updateGeometryPlacements (model, data, model_geom, data_geom, q);
    return computeDistances<ComputeShortest>(data_geom);
  }

  /// Given p1..3 being either "min" or "max", return one of the corners of the 
  /// AABB cub of the FCL object.
#define SE3_GEOM_AABB(FCL,p1,p2,p3)                                     \
  SE3::Vector3(                                                         \
    FCL->aabb_local.p1##_ [0],                                          \
    FCL->aabb_local.p2##_ [1],                                          \
    FCL->aabb_local.p3##_ [2])

  /// For all bodies of the model, compute the point of the geometry model
  /// that is the further from the center of the joint. This quantity is used 
  /// in some continuous collision test.
  inline void computeBodyRadius(const Model &         model,
                                const GeometryModel & geomModel,
                                GeometryData &        geomData)
  {
    geomData.radius.resize(model.joints.size(),0);
    BOOST_FOREACH(const GeometryObject & geom,geomModel.geometryObjects)
    {
      const boost::shared_ptr<const fcl::CollisionGeometry> & fcl
        = geom.collision_geometry;
      const SE3 & jMb = geom.placement; // placement in joint.
      const Model::JointIndex & i = geom.parentJoint;
      assert (i<geomData.radius.size());

      double radius = geomData.radius[i];

      // The radius is simply the one of the 8 corners of the AABB cube, expressed 
      // in the joint frame, whose norm is the highest.
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,min,min,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,min,min,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,min,max,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,min,max,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,max,min,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,max,min,max)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,max,max,min)).squaredNorm(),radius);
      radius = std::max (jMb.act(SE3_GEOM_AABB(fcl,max,max,max)).squaredNorm(),radius);

      // Don't forget to sqroot the squared norm before storing it.
      geomData.radius[i] = sqrt(radius);
    }
  }

#undef SE3_GEOM_AABB
#endif // WITH_HPP_FCL

} // namespace se3

#endif // ifnded __se3_algo_geometry_hxx__
