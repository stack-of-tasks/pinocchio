//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/parallel/rnea.hpp"

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {

    static void rnea_proxy_res(
      const int num_thread,
      ModelPool & pool,
      const Eigen::MatrixXd & q,
      const Eigen::MatrixXd & v,
      const Eigen::MatrixXd & a,
      Eigen::Ref<Eigen::MatrixXd> tau)
    {
      rneaInParallel(num_thread, pool, q, v, a, tau);
    }

    static Eigen::MatrixXd rnea_proxy(
      const int num_thread,
      ModelPool & pool,
      const Eigen::MatrixXd & q,
      const Eigen::MatrixXd & v,
      const Eigen::MatrixXd & a)
    {
      Eigen::MatrixXd tau(v.rows(), v.cols());
      rneaInParallel(num_thread, pool, q, v, a, tau);
      return tau;
    }

    void exposeParallelRNEA()
    {
      namespace bp = boost::python;

      using namespace Eigen;

      bp::def(
        "rneaInParallel", rnea_proxy, bp::args("num_thread", "pool", "q", "v", "a"),
        "Computes in parallel the RNEA and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: pool of model/data\n"
        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
        "\tv: the joint velocity vector (size model.nv x batch_size)\n"
        "\ta: the joint acceleration vector (size model.nv x batch_size)\n");

      bp::def(
        "rneaInParallel", rnea_proxy_res, bp::args("num_thread", "pool", "q", "v", "a", "tau"),
        "Computes in parallel the RNEA and stores the result in tau.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: pool of model/data\n"
        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
        "\tv: the joint velocity vector (size model.nv x batch_size)\n"
        "\ta: the joint acceleration vector (size model.nv x batch_size)\n"
        "\ttau: the resulting joint torque vectors (size model.nv x batch_size)\n");
    }

  } // namespace python
} // namespace pinocchio
