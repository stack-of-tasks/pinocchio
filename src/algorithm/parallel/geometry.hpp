//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_parallel_geometry_hpp__
#define __pinocchio_algorithm_parallel_geometry_hpp__

#include <omp.h>

#include "pinocchio/multibody/pool/geometry.hpp"
#include "pinocchio/algorithm/geometry.hpp"
  
namespace pinocchio
{

  inline bool computeCollisions(const int num_threads,
                                const GeometryModel & geom_model,
                                GeometryData & geom_data,
                                const bool stopAtFirstCollision = false)
  {
    bool is_colliding = false;
    
    omp_set_num_threads(num_threads);
    std::size_t cp_index = 0;
    
#pragma omp parallel for
    for(cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
      if(stopAtFirstCollision && is_colliding) continue;
      const CollisionPair & cp = geom_model.collisionPairs[cp_index];
      
      if(geom_data.activeCollisionPairs[cp_index]
         && !(   geom_model.geometryObjects[cp.first].disableCollision
              || geom_model.geometryObjects[cp.second].disableCollision))
      {
        fcl::CollisionRequest & collision_request = geom_data.collisionRequests[cp_index];
        collision_request.distance_upper_bound = collision_request.security_margin + 1e-6; // TODO: change the margin
        
        fcl::CollisionResult & collision_result = geom_data.collisionResults[cp_index];
        collision_result.clear();

        fcl::Transform3f oM1 (toFclTransform3f(geom_data.oMg[cp.first ])),
                         oM2 (toFclTransform3f(geom_data.oMg[cp.second]));
        
        const GeometryData::ComputeCollision & do_computations = geom_data.collision_functors[cp_index];
        std::size_t res = do_computations(oM1, oM2, collision_request, collision_result);
        
        if(!is_colliding && res)
        {
          is_colliding = true;
          geom_data.collisionPairIndex = cp_index; // first pair to be in collision
        }
      }
    }
    
    return is_colliding;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  bool computeCollisions(const int num_threads,
                         const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const GeometryModel & geom_model,
                         GeometryData & geom_data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const bool stopAtFirstCollision = false)
  {
    updateGeometryPlacements(model, data, geom_model, geom_data, q);
    return computeCollisions(num_threads, geom_model, geom_data, stopAtFirstCollision);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorPool, typename CollisionVectorResult>
  void computeCollisions(const int num_threads,
                         GeometryPoolTpl<Scalar,Options,JointCollectionTpl> & pool,
                         const Eigen::MatrixBase<ConfigVectorPool> & q,
                         const Eigen::MatrixBase<CollisionVectorResult> & res,
                         const bool stopAtFirstCollision = false)
  {
    typedef GeometryPoolTpl<Scalar,Options,JointCollectionTpl> Pool;
    typedef typename Pool::Model Model;
    typedef typename Pool::Data Data;
    typedef typename Pool::GeometryModel GeometryModel;
    typedef typename Pool::GeometryData GeometryData;
    typedef typename Pool::DataVector DataVector;
    typedef typename Pool::GeometryDataVector GeometryDataVector;
    
    const Model & model = pool.model();
    const GeometryModel & geometry_model = pool.geometry_model();
    DataVector & datas = pool.datas();
    GeometryDataVector & geometry_datas = pool.geometry_datas();
    CollisionVectorResult & res_ = res.const_cast_derived();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(num_threads <= pool.size(), "The pool is too small");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.rows(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), res.size());
    res_.fill(false);
    
    omp_set_num_threads(num_threads);
    const Eigen::DenseIndex batch_size = res.size();
    Eigen::DenseIndex i = 0;
    
#pragma omp parallel for
    for(i = 0; i < batch_size; i++)
    {
      const int thread_id = omp_get_thread_num();
      Data & data = datas[(size_t)thread_id];
      GeometryData & geometry_data = geometry_datas[(size_t)thread_id];
      res_[i] = computeCollisions(model,data,geometry_model,geometry_data,q.col(i),stopAtFirstCollision);
    }
  }
}

#endif // ifndef __pinocchio_algorithm_parallel_geometry_hpp__
