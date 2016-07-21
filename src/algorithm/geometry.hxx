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
    for (GeometryData::GeomIndex i=0; i < (GeometryData::GeomIndex) data_geom.model_geom.ngeoms; ++i)
    {
      const Model::JointIndex & parent = model_geom.geometryObjects[i].parent;
      data_geom.oMg[i] =  (data.oMi[parent] * model_geom.geometryObjects[i].placement);
      data_geom.oMg_fcl[i] =  toFclTransform3f(data_geom.oMg[i]);
    }
  }
  
  inline bool computeCollisions(GeometryData & data_geom,
                                const bool stopAtFirstCollision
                                )
  {
    bool isColliding = false;
    
    for (std::size_t cpt = 0; cpt < data_geom.collision_pairs.size(); ++cpt)
    {
      data_geom.collision_results[cpt] = data_geom.computeCollision(data_geom.collision_pairs[cpt].first, data_geom.collision_pairs[cpt].second);
      isColliding |= data_geom.collision_results[cpt].fcl_collision_result.isCollision();
      if(isColliding && stopAtFirstCollision)
        return true;
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
  
  
  inline void computeDistances(GeometryData & data_geom)
  {
    for (std::size_t cpt = 0; cpt < data_geom.collision_pairs.size(); ++cpt)
      data_geom.distance_results[cpt] = data_geom.computeDistance(data_geom.collision_pairs[cpt].first,
                                                                  data_geom.collision_pairs[cpt].second);
  }
  
  inline void computeDistances(const Model & model,
                               Data & data,
                               const GeometryModel & model_geom,
                               GeometryData & data_geom,
                               const Eigen::VectorXd & q
                               )
  {
    updateGeometryPlacements (model, data, model_geom, data_geom, q);
    computeDistances(data_geom);
  }
  
} // namespace se3

#endif __se3_geometry_hxx__
