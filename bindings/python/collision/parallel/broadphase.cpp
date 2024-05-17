//
// Copyright (c) 2021-2024 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/collision/parallel/broadphase.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/broadphase_SSaP.h>
#include <hpp/fcl/broadphase/broadphase_SaP.h>
#include <hpp/fcl/broadphase/broadphase_bruteforce.h>
#include <hpp/fcl/broadphase/broadphase_interval_tree.h>
#include <hpp/fcl/broadphase/broadphase_spatialhash.h>

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
    using namespace Eigen;

    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

    template<typename BroadPhaseManager>
    VectorXb computeCollisionsInParallel_1(
      const size_t num_threads,
      BroadPhaseManagerPoolBase<BroadPhaseManager, double> & pool,
      const Eigen::MatrixBase<Eigen::MatrixXd> & q,
      const bool stopAtFirstCollisionInConfiguration = false,
      const bool stopAtFirstCollisionInBatch = false)
    {
      VectorXb res(q.cols());
      computeCollisionsInParallel(
        num_threads, pool, q, res, stopAtFirstCollisionInConfiguration,
        stopAtFirstCollisionInBatch);
      return res;
    }

    template<typename BroadPhaseManager>
    std::vector<VectorXb> computeCollisionsInParallel_2(
      const size_t num_threads,
      BroadPhaseManagerPoolBase<BroadPhaseManager, double> & pool,
      const std::vector<Eigen::MatrixXd> & trajectories,
      const bool stopAtFirstCollisionInTrajectory = false)
    {
      std::vector<VectorXb> res(trajectories.size());
      for (size_t k = 0; k < trajectories.size(); ++k)
      {
        res[k].resize(trajectories[k].cols());
      }
      computeCollisionsInParallel(
        num_threads, pool, trajectories, res, stopAtFirstCollisionInTrajectory);
      return res;
    }

    template<typename BroadPhaseManager>
    void exposeCase()
    {
      bp::def(
        "computeCollisionsInParallel", computeCollisionsInParallel_1<BroadPhaseManager>,
        (bp::arg("num_thread"), bp::arg("pool"), bp::arg("q"),
         bp::arg("stop_at_first_collision_in_configuration") = false,
         bp::arg("stop_at_first_collision_in_batch") = false),
        "Evaluates in parallel the batch of configurations and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: the broadphase manager pool\n"
        "\tq: the batch of joint configurations\n"
        "\tstop_at_first_collision_in_configuration: if set to true, stops when encountering "
        "the first collision in a configuration\n"
        "\tstop_at_first_collision_in_batch: if set to true, stops when encountering the "
        "first collision in a batch.\n");

      bp::def(
        "computeCollisionsInParallel", computeCollisionsInParallel_2<BroadPhaseManager>,
        (bp::arg("num_thread"), bp::arg("pool"), bp::arg("trajectories"),
         bp::arg("stop_at_first_collision_in_trajectory") = false),
        "Evaluates in parallel the batch of trajectories and returns a vector of vector of Boolean "
        "containing the status for each configuration contained in a given trajectory.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: the broadphase manager pool\n"
        "\ttrajectories: the list of joint trajectories\n"
        "\tstop_at_first_collision_in_trajectory: if set to true, stops when encountering the "
        "first collision in a given trajectory.\n");
    }

    void exposeParallelBroadPhase()
    {
      exposeCase<BroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager>>();
      exposeCase<TreeBroadPhaseManagerTpl<hpp::fcl::DynamicAABBTreeCollisionManager>>();
    }

  } // namespace python
} // namespace pinocchio
