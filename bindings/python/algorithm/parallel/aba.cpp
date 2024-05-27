//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/parallel/aba.hpp"

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {

    static void aba_proxy_res(
      const int num_thread,
      ModelPool & pool,
      const Eigen::MatrixXd & q,
      const Eigen::MatrixXd & v,
      const Eigen::MatrixXd & tau,
      Eigen::Ref<Eigen::MatrixXd> a)
    {
      abaInParallel(num_thread, pool, q, v, tau, a);
    }

    static Eigen::MatrixXd aba_proxy(
      const int num_thread,
      ModelPool & pool,
      const Eigen::MatrixXd & q,
      const Eigen::MatrixXd & v,
      const Eigen::MatrixXd & tau)
    {
      Eigen::MatrixXd a(v.rows(), v.cols());
      abaInParallel(num_thread, pool, q, v, tau, a);
      return a;
    }

    void exposeParallelABA()
    {
      namespace bp = boost::python;

      using namespace Eigen;

      bp::def(
        "abaInParallel", aba_proxy, bp::args("num_thread", "pool", "q", "v", "tau"),
        "Computes in parallel the ABA and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: pool of model/data\n"
        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
        "\tv: the joint velocity vector (size model.nv x batch_size)\n"
        "\ttau: the joint torque vector (size model.nv x batch_size)\n");

      bp::def(
        "abaInParallel", aba_proxy_res, bp::args("num_thread", "pool", "q", "v", "tau", "a"),
        "Computes in parallel the ABA, store the result in a.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: pool of model/data\n"
        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
        "\tv: the joint velocity vector (size model.nv x batch_size)\n"
        "\ttau: the joint torque vector (size model.nv x batch_size)\n"
        "\ta: the resulting joint acceleration vectors (size model.nv x batch_size)\n");
    }

  } // namespace python
} // namespace pinocchio
