// Copyright (c) 2026 INRIA

#include <omp.h>

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/parallel/aba.hpp"
#include "pinocchio/algorithm/parallel/rnea.hpp"

// #include "pinocchio/utils/openmp.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
void exposeParallelAlgorithms(nb::module_ m)
{
  using namespace nb::literals;

  // doesn't work because OpenMPException doesn't implement `what()`
  // nb::exception<OpenMPException>(m, "OpenMPException", PyExc_Exception);

  // abaInParallel() -------------------------------------
  m.def(
    "abaInParallel",
    [](
      const size_t num_threads, ModelPool & pool, const MatrixXs & q, const MatrixXs & v,
      const MatrixXs & tau,
      nb::DRef<MatrixXs> a_out) { abaInParallel(num_threads, pool, q, v, tau, a_out); },
    "num_threads"_a, "pool"_a, "qs"_a, "vs"_a, "taus"_a,
    "as_out"_a
    "Computes in parallel the ABA and returns the result.\n\n"
    "Parameters:\n"
    "\tnum_threads: number of threads used for the computation\n"
    "\tpool: pool of model/data\n"
    "\tqs: the joint configuration vectors (size model.nq x batch_size)\n"
    "\tvs: the joint velocity vectors (size model.nv x batch_size)\n"
    "\ttaus: the joint torque vectors (size model.nv x batch_size)\n"
    "\ta_out: output joint acceleration vectors (size model.nv x batch_size)\n");

  m.def(
    "abaInParallel",
    [](
      const size_t num_threads, ModelPool & pool, const MatrixXs & q, const MatrixXs & v,
      const MatrixXs & tau) -> MatrixXs {
      MatrixXs a_out(v.rows(), v.cols());
      abaInParallel(num_threads, pool, q, v, tau, a_out);
      return a_out;
    },
    "num_threads"_a, "pool"_a, "qs"_a, "vs"_a, "taus"_a,
    "Computes in parallel the ABA and returns the result.\n\n"
    "Parameters:\n"
    "\tnum_threads: number of threads used for the computation\n"
    "\tpool: pool of model/data\n"
    "\tqs: the joint configuration vectors (size model.nq x batch_size)\n"
    "\tvs: the joint velocity vectors (size model.nv x batch_size)\n"
    "\ttaus: the joint torque vectors (size model.nv x batch_size)\n"
    "Returns:\n"
    "\ta_out: output joint acceleration vectors (size model.nv x batch_size)\n");

  // rneaaInParallel() ------------------------------------
  m.def(
    "rneaInParallel",
    [](
      const size_t num_threads, ModelPool & pool, const MatrixXs & q, const MatrixXs & v,
      const MatrixXs & a,
      nb::DRef<MatrixXs> tau_out) { rneaInParallel(num_threads, pool, q, v, a, tau_out); },
    "num_threads"_a, "pool"_a, "qs"_a, "vs"_a, "a"_a, "taus_out"_a,
    "Computes in parallel the RNEA and returns the result.\n\n"
    "Parameters:\n"
    "\tnum_threads: number of threads used for the computation\n"
    "\tpool: pool of model/data\n"
    "\tqs: the joint configuration vectors (size model.nq x batch_size)\n"
    "\tvs: the joint velocity vectors (size model.nv x batch_size)\n"
    "\ta: the joint acceleration vectors (size model.nv x batch_size)\n"
    "\ttaus_out: output joint torque vectors (size model.nv x batch_size)\n");

  m.def(
    "rneaInParallel",
    [](
      const size_t num_threads, ModelPool & pool, const MatrixXs & q, const MatrixXs & v,
      const MatrixXs & a) {
      MatrixXs tau(v.rows(), v.cols());
      rneaInParallel(num_threads, pool, q, v, a, tau);
      return tau;
    },
    "num_threads"_a, "pool"_a, "qs"_a, "vs"_a, "a"_a,
    "Computes in parallel the RNEA and stores the result in tau.\n\n"
    "Parameters:\n"
    "\tnum_threads: number of threads used for the computation\n"
    "\tpool: pool of model/data\n"
    "\tqs: the joint configuration vectors (size model.nq x batch_size)\n"
    "\tvs: the joint velocity vectors (size model.nv x batch_size)\n"
    "\ta: the joint acceleration vectors (size model.nv x batch_size)\n"
    "Returns:\n"
    "\tthe resulting joint torque vectors (size model.nv x batch_size)\n");

  // Probably should be deprecated in favor of functions in
  // <pinocchio/utils/openmp.hpp> ?
  m.def(
    "omp_get_max_threads", &omp_get_max_threads,
    "Returns an upper bound on the number of threads that can be used.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
