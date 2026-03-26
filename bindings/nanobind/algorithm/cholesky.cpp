// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/cholesky.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeCholesky(nb::module_ m_)
{
  using namespace pinocchio::cholesky;

  nb::module_ m = m_.def_submodule("cholesky", "Submodule for Cholesky decomposition algorithms.");

  m.def(
    "decompose",
    [](const Model & model, Data & data) -> const Data::MatrixXs & {
      return decompose(model, data);
    },
    "model"_a, "data"_a,
    "Computes the Cholesky decomposition of the joint space inertia matrix M contained in data.\n"
    "The upper triangular part of data.M should have been filled first by calling "
    "crba, or any related algorithms.");

  m.def(
    "solve",
    [](const Model & model, const Data & data, VectorXs v) -> VectorXs {
      solve(model, data, v);
      return v;
    },
    "model"_a, "data"_a, "v"_a,
    "Returns the solution x of M x = y using the Cholesky decomposition stored in data given the "
    "entry y.");

  m.def(
    "computeMinv",
    [](const Model & model, Data & data) -> const Data::RowMatrixXs & {
      return computeMinv(model, data);
    },
    "model"_a, "data"_a,
    "Returns the inverse of the joint space inertia matrix using the results of the Cholesky "
    "decomposition performed by cholesky.decompose. The result is stored in data.Minv.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
