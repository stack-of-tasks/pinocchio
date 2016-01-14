//
// Copyright (c) 2015 CNRS
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


  inline void updateCollisionGeometry(const Model & model,
                                      Data & data,
                                      const GeometryModel& geom,
                                      GeometryData& data_geom,
                                      const Eigen::VectorXd & q,
                                      const bool computeGeometry = false
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
  
  
inline void  updateCollisionGeometry(const Model & model,
                                     Data & data,
                                     const GeometryModel & model_geom,
                                     GeometryData & data_geom,
                                     const Eigen::VectorXd & q,
                                     const bool computeGeometry
                                     )
{
  using namespace se3;

  if (computeGeometry) geometry(model, data, q);
  for (GeometryData::Index i=0; i < (GeometryData::Index) data_geom.model_geom.ngeom; ++i)
  {
    const Model::Index & parent = model_geom.geom_parents[i];
    data_geom.oMg[i] =  (data.oMi[parent] * model_geom.geometryPlacement[i]);
    data_geom.oMg_fcl[i] =  toFclTransform3f(data_geom.oMg[i]);
  }
}

inline bool computeCollisions(GeometryData & data_geom,
                              const bool stopAtFirstCollision
                              )
{
  using namespace se3;

  bool isColliding = false;

  for (std::size_t cpt = 0; cpt < data_geom.collision_pairs.size(); ++cpt)
  {
    data_geom.collisions[cpt] = data_geom.collide(data_geom.collision_pairs[cpt].first, data_geom.collision_pairs[cpt].second);
    isColliding = data_geom.collisions[cpt];
    if(isColliding && stopAtFirstCollision)
    {
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
  using namespace se3;


  updateCollisionGeometry (model, data, model_geom, data_geom, q, true);

  
  return computeCollisions(data_geom, stopAtFirstCollision);

}


inline void computeDistances(GeometryData & data_geom)
{
  for (std::size_t cpt = 0; cpt < data_geom.collision_pairs.size(); ++cpt)
  {
    data_geom.distances[cpt] = DistanceResult(data_geom.computeDistance(data_geom.collision_pairs[cpt].first, data_geom.collision_pairs[cpt].second),
                                              data_geom.collision_pairs[cpt].first,
                                              data_geom.collision_pairs[cpt].second
                                              );
  }
}

inline void computeDistances(const Model & model,
                            Data & data,
                            const GeometryModel & model_geom,
                            GeometryData & data_geom,
                            const Eigen::VectorXd & q
                            )
{
  updateCollisionGeometry (model, data, model_geom, data_geom, q, true);

  computeDistances(data_geom);
}
} // namespace se3

#endif // ifndef __se3_collisions_hpp__

