//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_collision_parallel_broadphase_hpp__
#define __pinocchio_collision_parallel_broadphase_hpp__

#include "pinocchio/collision/pool/broadphase-manager.hpp"
#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/parallel/omp.hpp"

#include <cstdint>

namespace pinocchio
{

  template<
    typename BroadPhaseManagerDerived,
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorPool,
    typename CollisionVectorResult>
  void computeCollisionsInParallel(
    const size_t num_threads,
    BroadPhaseManagerPoolBase<BroadPhaseManagerDerived, Scalar, Options, JointCollectionTpl> & pool,
    const Eigen::MatrixBase<ConfigVectorPool> & q,
    const Eigen::MatrixBase<CollisionVectorResult> & res,
    const bool stopAtFirstCollisionInConfiguration = false,
    const bool stopAtFirstCollisionInBatch = false)
  {
    typedef BroadPhaseManagerPoolBase<BroadPhaseManagerDerived, Scalar, Options, JointCollectionTpl>
      Pool;
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

    if (stopAtFirstCollisionInBatch)
    {
      bool is_colliding = false;
      Eigen::DenseIndex i = 0;
#pragma omp parallel for schedule(static)
      for (i = 0; i < batch_size; i++)
      {
        if (is_colliding)
          continue;

        const int thread_id = omp_get_thread_num();
        const Model & model = models[(size_t)thread_id];
        Data & data = datas[(size_t)thread_id];
        BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];
        res_[i] =
          computeCollisions(model, data, manager, q.col(i), stopAtFirstCollisionInConfiguration);

        if (res_[i])
        {
          is_colliding = true;
        }
      }
    }
    else
    {
      Eigen::DenseIndex i = 0;
#pragma omp parallel for schedule(static)
      for (i = 0; i < batch_size; i++)
      {
        const int thread_id = omp_get_thread_num();
        const Model & model = models[(size_t)thread_id];
        Data & data = datas[(size_t)thread_id];
        BroadPhaseManager & manager = broadphase_managers[(size_t)thread_id];
        res_[i] =
          computeCollisions(model, data, manager, q.col(i), stopAtFirstCollisionInConfiguration);
      }
    }
  }

  ///
  /// \brief Evaluate the collision over a set of trajectories and return whether a trajectory
  /// contains a collision
  ///
  template<
    typename BroadPhaseManagerDerived,
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl>
  void computeCollisionsInParallel(
    const size_t num_threads,
    BroadPhaseManagerPoolBase<BroadPhaseManagerDerived, Scalar, Options, JointCollectionTpl> & pool,
    const std::vector<Eigen::MatrixXd> & trajectories,
    std::vector<VectorXb> & res,
    const bool stopAtFirstCollisionInTrajectory = false)
  {
    typedef BroadPhaseManagerPoolBase<BroadPhaseManagerDerived, Scalar, Options, JointCollectionTpl>
      Pool;
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

    PINOCCHIO_CHECK_INPUT_ARGUMENT(num_threads <= pool.size(), "The pool is too small");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(trajectories.size(), res.size());

    for (size_t k = 0; k < trajectories.size(); ++k)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(trajectories[k].cols(), res[k].size());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(trajectories[k].rows(), model_check.nq);
    }

    set_default_omp_options(num_threads);
    const int64_t batch_size = static_cast<int64_t>(trajectories.size());

    int64_t omp_i = 0;
    // Visual studio support OpenMP 2 that only support signed indexes in for loops
    // See
    // https://stackoverflow.com/questions/2820621/why-arent-unsigned-openmp-index-variables-allowed
    // -openmp:llvm could solve this:
    // https://learn.microsoft.com/en-us/cpp/build/reference/openmp-enable-openmp-2-0-support?view=msvc-160
#pragma omp parallel for schedule(static)
    for (omp_i = 0; omp_i < batch_size; omp_i++)
    {
      size_t i = static_cast<size_t>(omp_i);
      const int thread_id = omp_get_thread_num();
      const Model & model = models[size_t(thread_id)];
      Data & data = datas[(size_t)thread_id];
      const Eigen::MatrixXd & current_traj = trajectories[i];
      VectorXb & res_current_traj = res[i];
      res_current_traj.fill(false);
      BroadPhaseManager & manager = broadphase_managers[size_t(thread_id)];

      for (Eigen::DenseIndex col_id = 0; col_id < current_traj.cols(); ++col_id)
      {
        res_current_traj[col_id] =
          computeCollisions(model, data, manager, current_traj.col(col_id), true);
        if (res_current_traj[col_id] && stopAtFirstCollisionInTrajectory)
          break;
      }
    }
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_collision_parallel_broadphase_hpp__
