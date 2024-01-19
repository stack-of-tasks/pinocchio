//
// Copyright (c) 2022-2024 INRIA
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
  void computeCollisionsInParallel(const size_t num_threads,
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
#pragma omp parallel for schedule(static)
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
#pragma omp parallel for schedule(static)
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

///
/// \brief Evaluate the collision over a set of trajectories and return whether a trajectory contains a collision
///
template<typename BroadPhaseManagerDerived, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename CollisionVectorResult>
void computeCollisionsInParallel(const size_t num_threads,
                                 BroadPhaseManagerPoolBase<BroadPhaseManagerDerived,Scalar,Options,JointCollectionTpl> & pool,
                                 const std::vector<Eigen::MatrixXd> & trajectories,
                                 const Eigen::MatrixBase<CollisionVectorResult> & res)
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
  PINOCCHIO_CHECK_ARGUMENT_SIZE(Eigen::DenseIndex(trajectories.size()), res.size());
  res_.fill(false);

  set_default_omp_options(num_threads);
  const size_t batch_size = trajectories.size();

  size_t i = 0;
#pragma omp parallel for schedule(static)
  for(i = 0; i < batch_size; i++)
  {
    const int thread_id = omp_get_thread_num();
    const Model & model = models[(size_t)thread_id];
    Data & data = datas[(size_t)thread_id];
    const Eigen::MatrixXd & qs = trajectories[i];
    BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];

    PINOCCHIO_CHECK_ARGUMENT_SIZE(qs.rows(), model_check.nq);
    res_[Eigen::DenseIndex(i)] = false;
    for(Eigen::DenseIndex col_id = 0; col_id < qs.cols(); ++col_id)
    {
      const bool colliding = computeCollisions(model,data,manager,qs.col(col_id));
      if(colliding)
      {
        res_[Eigen::DenseIndex(i)] = true; break;
      }
    }

  }
}
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_parallel_broadphase_hpp__
