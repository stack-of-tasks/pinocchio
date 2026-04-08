// Copyright (c) 2026 INRIA
//
// Port of the following file:
// include/pinocchio/bindings/python/algorithm/delassus-operator.hpp

#pragma once

#include "../fwd.hpp"

#include "pinocchio/algorithm/delassus-operator.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Visitor exposing the common interface of all Delassus operator types.
template<typename DelassusOperator>
struct DelassusOperatorBaseVisitor : nb::def_visitor<DelassusOperatorBaseVisitor<DelassusOperator>>
{
  using Self = DelassusOperator;
  using Scalar = typename Self::Scalar;
  using Vector = typename Self::Vector;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;

    cl
      // Matrix-multiply
      .def(
        "__mul__",
        [](const Self & self, const MatrixXs & other) -> MatrixXs { return self * other; },
        "other"_a, nb::is_operator(), "Matrix multiplication: Delassus * matrix.")
      .def(
        "__matmul__",
        [](const Self & self, const MatrixXs & other) -> MatrixXs { return self * other; },
        "other"_a, nb::is_operator(), "Matrix multiplication: Delassus @ matrix.")

      // solve
      .def(
        "solve",
        [](const Self & self, const MatrixXs & mat) -> MatrixXs { return self.solve(mat); },
        "mat"_a,
        "Returns the solution x of Delassus * x = mat using the current decomposition of the "
        "Delassus matrix.")

      // compliance
      .def(
        "updateCompliance", [](Self & self, Scalar mu) { self.updateCompliance(mu); }, "mu"_a,
        "Add a compliance term to the diagonal of the Delassus matrix. The compliance term "
        "should be positive.")
      .def(
        "updateCompliance", [](Self & self, const VectorXs & mus) { self.updateCompliance(mus); },
        "mus"_a,
        "Add compliance terms to the diagonal of the Delassus matrix. All terms should be "
        "positive.")
      .def(
        "getCompliance", [](const Self & self) -> VectorXs { return self.getCompliance(); },
        "Returns the compliance terms of the Delassus operator.")

      // damping
      .def(
        "updateDamping", [](Self & self, Scalar mu) { self.updateDamping(mu); }, "mu"_a,
        "Add a damping term to the diagonal of the Delassus matrix. The damping term should be "
        "positive.")
      .def(
        "updateDamping", [](Self & self, const VectorXs & mus) { self.updateDamping(mus); },
        "mus"_a,
        "Add damping terms to the diagonal of the Delassus matrix. All terms should be positive.")
      .def(
        "getDamping", [](const Self & self) -> MatrixXs { return self.getDamping().matrix(); },
        "Returns the damping terms of the Delassus operator as a dense matrix.")

      // misc
      .def(
        "inverse", &Self::inverse,
        "Returns the inverse of the Delassus expression as a dense matrix.")
      .def("size", &Self::size, "Returns the size of the decomposition.")
      .def("rows", &Self::rows, "Returns the number of rows.")
      .def("cols", &Self::cols, "Returns the number of columns.");
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
