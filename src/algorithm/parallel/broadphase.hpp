//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_parallel_broadphase_hpp__
#define __pinocchio_algorithm_parallel_broadphase_hpp__

#include <omp.h>

#include "pinocchio/multibody/pool/broadphase-manager.hpp"
#include "pinocchio/algorithm/broadphase.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace pinocchio
{

  template<typename BroadPhaseManagerDerived, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorPool, typename CollisionVectorResult>
  void computeCollisions(const size_t num_threads,
                         BroadPhaseManagerPoolTpl<BroadPhaseManagerDerived,Scalar,Options,JointCollectionTpl> & pool,
                         const Eigen::MatrixBase<ConfigVectorPool> & q,
                         const Eigen::MatrixBase<CollisionVectorResult> & res,
                         const bool stopAtFirstCollision = false)
  {
    typedef BroadPhaseManagerPoolTpl<BroadPhaseManagerDerived,Scalar,Options,JointCollectionTpl> Pool;
    typedef typename Pool::Model Model;
    typedef typename Pool::Data Data;
    typedef typename Pool::DataVector DataVector;
    typedef typename Pool::BroadPhaseManager BroadPhaseManager;
    typedef typename Pool::BroadPhaseManagerVector BroadPhaseManagerVector;
    
    const Model & model = pool.getModel();
    DataVector & datas = pool.getDatas();
    BroadPhaseManagerVector & broadphase_managers = pool.getBroadPhaseManagers();
    CollisionVectorResult & res_ = res.const_cast_derived();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(num_threads <= pool.size(), "The pool is too small");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.rows(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), res.size());
    res_.fill(false);
    
    omp_set_num_threads((int)num_threads);
    const Eigen::DenseIndex batch_size = res.size();
    Eigen::DenseIndex i = 0;
    
#pragma omp parallel for
    for(i = 0; i < batch_size; i++)
    {
      const int thread_id = omp_get_thread_num();
      Data & data = datas[(size_t)thread_id];
      BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];
      const GeometryModel & geom_model = pool.getGeometryModel();
      GeometryData & geom_data = pool.getGeometryData((size_t)thread_id);
      res_[i] = computeCollisions(model,data,manager,q.col(i),stopAtFirstCollision);
    }
  }
}

#endif // ifndef __pinocchio_algorithm_parallel_broadphase_hpp__
