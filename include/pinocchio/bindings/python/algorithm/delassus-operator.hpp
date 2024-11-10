//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_python_algorithm_delssus_operator_hpp__
#define __pinocchio_python_algorithm_delssus_operator_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/delassus-operator-base.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename DelassusOperator>
    struct DelassusOperatorBasePythonVisitor
    : public boost::python::def_visitor<DelassusOperatorBasePythonVisitor<DelassusOperator>>
    {
      typedef DelassusOperator Self;
      typedef typename DelassusOperator::Scalar Scalar;
      typedef context::MatrixXs Matrix;
      typedef typename DelassusOperator::Vector Vector;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::self * bp::other<Matrix>())
          .def(
            "__matmul__",
            +[](const DelassusOperator & self, const Matrix & other) -> Matrix {
              return self * other;
            },
            bp::args("self", "other"),
            "Matrix multiplication between self and another matrix. Returns the result of Delassus "
            "* matrix.")

          .def(
            "solve", &DelassusOperator::template solve<Matrix>, bp::args("self", "mat"),
            "Returns the solution x of Delassus * x = mat using the current decomposition of "
            "the Delassus matrix.")

          .def(
            "computeLargestEigenValue",
            (Scalar(DelassusOperator::*)(const bool, const int, const Scalar)
               const)&DelassusOperator::computeLargestEigenValue,
            (bp::arg("self"), bp::arg("reset") = true, bp::arg("max_it") = 10,
             bp::arg("prec") = Scalar(1e-8)),
            "Compute the largest eigenvalue associated to the underlying Delassus matrix.")
          .def(
            "computeLowestEigenValue",
            (Scalar(DelassusOperator::*)(const bool, const bool, const int, const Scalar)
               const)&DelassusOperator::computeLowestEigenValue,
            (bp::arg("self"), bp::arg("reset") = true, bp::arg("compute_largest") = true,
             bp::arg("max_it") = 10, bp::arg("prec") = Scalar(1e-8)),
            "Compute the lowest eigenvalue associated to the underlying Delassus matrix.")

          .def(
            "updateDamping",
            (void(DelassusOperator::*)(const Scalar &)) & DelassusOperator::updateDamping,
            bp::args("self", "mu"),
            "Add a damping term to the diagonal of the Delassus matrix. The damping term should "
            "be positive.")
          .def(
            "updateDamping", &DelassusOperator::template updateDamping<Vector>,
            bp::args("self", "mus"),
            "Add a damping term to the diagonal of the Delassus matrix. The damping terms "
            "should be all positive.")

          .def(
            "matrix", &DelassusOperator::matrix, bp::arg("self"),
            "Returns the Delassus expression as a dense matrix.")
          .def(
            "inverse", &DelassusOperator::inverse, bp::arg("self"),
            "Returns the inverse of the Delassus expression as a dense matrix.")

          .def(
            "size", &DelassusOperator::size, bp::arg("self"),
            "Returns the size of the decomposition.")
          .def("rows", &DelassusOperator::rows, bp::arg("self"), "Returns the number of rows.")
          .def("cols", &DelassusOperator::cols, bp::arg("self"), "Returns the number of columns.");
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_delssus_operator_hpp__
