//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_algo_distance_hxx__
#define __pinocchio_algo_distance_hxx__

namespace pinocchio
{

#ifdef PINOCCHIO_WITH_HPP_FCL

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

#endif // PINOCCHIO_WITH_HPP_FCL


} // namespace pinocchio

#endif // ifndef __pinocchio_algo_distance_hxx__
