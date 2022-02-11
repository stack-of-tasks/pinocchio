//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_collision_hxx__
#define __pinocchio_algo_collision_hxx__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{

#ifdef PINOCCHIO_WITH_HPP_FCL

  inline bool computeCollision(const GeometryModel & geom_model,
                               GeometryData & geom_data,
                               const PairIndex pair_id)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( geom_model.collisionPairs.size() == geom_data.collisionResults.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair_id < geom_model.collisionPairs.size() );
    
    const CollisionPair & pair = geom_model.collisionPairs[pair_id];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT(geom_data.activeCollisionPairs[pair_id]);

    fcl::CollisionRequest & collision_request = geom_data.collisionRequests[pair_id];
    
    collision_request.distance_upper_bound = collision_request.security_margin + 1e-6; // TODO: change the margin
    
    fcl::CollisionResult & collision_result = geom_data.collisionResults[pair_id];
    collision_result.clear();

    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));
    
    try
    {
      GeometryData::ComputeCollision & do_computations = geom_data.collision_functors[pair_id];
      do_computations.run(oM1, oM2, collision_request, collision_result);
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

#endif // PINOCCHIO_WITH_HPP_FCL


} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/collision.hxx"

#endif // ifndef __pinocchio_algo_collision_hxx__
