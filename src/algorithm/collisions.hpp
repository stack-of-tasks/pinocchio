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

#ifndef __se3_collisions_hpp__
#define __se3_collisions_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace se3
{


  ///
  /// \brief Apply a forward kinematics and update the placement of the geometry objects.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  inline void updateGeometryPlacements(const Model & model,
                                       Data & data,
                                       const GeometryModel & geom,
                                       GeometryData & geom_data,
                                       const Eigen::VectorXd & q
                                       );
  
  ///
  /// \brief Update the placement of the geometry objects according to the current joint placements contained in data.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] geom The geometry model containing the collision objects.
  /// \param[out] geom_data The geometry data containing the placements of the collision objects. See oMg field in GeometryData.
  ///
  inline void updateGeometryPlacements(const Model & model,
                                       const Data & data,
                                       const GeometryModel & geom,
                                       GeometryData & geom_data
                                       );

  inline bool computeCollisions(const Model & model,
                                Data & data,
                                const GeometryModel & model_geom,
                                GeometryData & data_geom,
                                const Eigen::VectorXd & q,
                                const bool stopAtFirstCollision = false
                                );

  inline bool computeCollisions(GeometryData & data_geom,
                                const bool stopAtFirstCollision = false
                                );

  inline void computeDistances(GeometryData & data_geom);

  inline void computeDistances(const Model & model,
                              Data & data,
                              const GeometryModel & model_geom,
                              GeometryData & data_geom,
                              const Eigen::VectorXd & q
                              );

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
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
      const Model::JointIndex & parent = model_geom.geometry_objects[i].parent;
      data_geom.oMg_geometries[i] =  (data.oMi[parent] * model_geom.geometry_objects[i].placement);
      data_geom.oMg_fcl_geometries[i] =  toFclTransform3f(data_geom.oMg_geometries[i]);
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

#endif // ifndef __se3_collisions_hpp__
