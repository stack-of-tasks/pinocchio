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
    std::cout << "computeCollision in" << std::endl;
    std::cout << "geom_data: " << &geom_data << std::endl;
    std::cout << "geom_model: " << &geom_model << std::endl;
    PINOCCHIO_CHECK_INPUT_ARGUMENT( geom_model.collisionPairs.size() == geom_data.collisionResults.size() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair_id < geom_model.collisionPairs.size() );
    
    std::cout << "get collision pair" << std::endl;
    const CollisionPair & pair = geom_model.collisionPairs[pair_id];

    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.first  < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( pair.second < geom_model.ngeoms );
    PINOCCHIO_CHECK_INPUT_ARGUMENT(geom_data.activeCollisionPairs[pair_id]);

    std::cout << "get collision_request" << std::endl;
    std::cout << "pair.first: " << pair.first << std::endl;
    std::cout << "pair.second: " << pair.second << std::endl;
    fcl::CollisionRequest & collision_request = geom_data.collisionRequests[pair_id];
    
    std::cout << "set distance_upper_bound" << std::endl;
    collision_request.distance_upper_bound = collision_request.security_margin + 1e-6; // TODO: change the margin
    
    std::cout << "get collision_result" << std::endl;
    fcl::CollisionResult & collision_result = geom_data.collisionResults[pair_id];
    collision_result.clear();

    fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[pair.first ])),
                     oM2 (toFclTransform3f(geom_data.oMg[pair.second]));
    
    std::cout << "do_computations" << std::endl;
    try
    {
      GeometryData::ComputeCollision & do_computations = geom_data.collision_functors[pair_id];
      std::cout << "geom obj1 ptr: " << &geom_model.geometryObjects[pair.first] << std::endl;
      std::cout << "ComputeCollision::geom obj1 ptr: " << &do_computations.getGeometryObject1() << std::endl;
      std::cout << "collision object 1 ptr: " << geom_model.geometryObjects[pair.first].geometry.get() << std::endl;
      std::cout << "ComputeCollision:: collision obj1 ptr: " << do_computations.getGeometryObject1().geometry.get() << std::endl;
      std::cout << "geom obj2 ptr: " << &geom_model.geometryObjects[pair.second] << std::endl;
      std::cout << "ComputeCollision::geom obj2 ptr: " << &do_computations.getGeometryObject2() << std::endl;
      std::cout << "collision object 2 ptr: " << geom_model.geometryObjects[pair.second].geometry.get() << std::endl;
      std::cout << "ComputeCollision:: collision obj2 ptr: " << do_computations.getGeometryObject2().geometry.get() << std::endl;
      do_computations.run(oM1, oM2, collision_request, collision_result);
    }
    catch(std::invalid_argument & e)
    {
      std::stringstream ss;
      ss << "Problem when trying to check the collision of collision pair #" << pair_id << " (" << pair.first << "," << pair.second << ")" << std::endl;
      ss << "hpp-fcl original error:\n" << e.what() << std::endl;
      throw std::invalid_argument(ss.str());
    }
    
    std::cout << "computeCollision out" << std::endl;
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
