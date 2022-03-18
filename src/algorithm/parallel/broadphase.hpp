//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_parallel_broadphase_hpp__
#define __pinocchio_algorithm_parallel_broadphase_hpp__

#include "pinocchio/multibody/pool/broadphase-manager.hpp"
#include "pinocchio/algorithm/broadphase.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/parallel/omp.hpp"

namespace pinocchio
{

  template<typename BroadPhaseManagerDerived, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorPool, typename CollisionVectorResult>
  void computeCollisions(const size_t num_threads,
                         BroadPhaseManagerPoolBase<BroadPhaseManagerDerived,Scalar,Options,JointCollectionTpl> & pool,
                         const Eigen::MatrixBase<ConfigVectorPool> & q,
                         const Eigen::MatrixBase<CollisionVectorResult> & res,
                         const bool stopAtFirstCollisionInConfiguration = false,
                         const bool stopAtFirstCollisionInBatch = false)
  {
    typedef BroadPhaseManagerPoolBase<BroadPhaseManagerDerived,Scalar,Options,JointCollectionTpl> Pool;
    typedef typename Pool::Model Model;
    typedef typename Pool::Data Data;
    typedef typename Pool::ModelVector ModelVector;
    typedef typename Pool::DataVector DataVector;
    typedef typename Pool::BroadPhaseManager BroadPhaseManager;
    typedef typename Pool::BroadPhaseManagerVector BroadPhaseManagerVector;

    const ModelVector & models = pool.getModels();
    const Model & model_check = models[0];
    DataVector & datas = pool.getDatas();
    BroadPhaseManagerVector & broadphase_managers = pool.getBroadPhaseManagers();
    CollisionVectorResult & res_ = res.const_cast_derived();

    PINOCCHIO_CHECK_INPUT_ARGUMENT(num_threads <= pool.size(), "The pool is too small");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.rows(), model_check.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), res.size());
    res_.fill(false);

    set_default_omp_options(num_threads);
    const Eigen::DenseIndex batch_size = res.size();

    if(stopAtFirstCollisionInBatch)
    {
      bool is_colliding = false;
      Eigen::DenseIndex i = 0;
#pragma omp parallel for schedule(dynamic)
      for(i = 0; i < batch_size; i++)
      {
        if(is_colliding) continue;
        
        const int thread_id = omp_get_thread_num();
        const Model & model = models[(size_t)thread_id];
        Data & data = datas[(size_t)thread_id];
        BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];
        res_[i] = computeCollisions(model,data,manager,q.col(i),stopAtFirstCollisionInConfiguration);
        
        if(res_[i])
        {
          is_colliding = true;
        }
      }
    }
    else
    {
      Eigen::DenseIndex i = 0;
#pragma omp parallel for schedule(dynamic)
      for(i = 0; i < batch_size; i++)
      {
        const int thread_id = omp_get_thread_num();
        const Model & model = models[(size_t)thread_id];
        Data & data = datas[(size_t)thread_id];
        BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];
        res_[i] = computeCollisions(model,data,manager,q.col(i),stopAtFirstCollisionInConfiguration);
      }
    }
  }
}

#endif // ifndef __pinocchio_algorithm_parallel_broadphase_hpp__
