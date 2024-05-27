//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/math/eigenvalues.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename MatrixType>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) inv(const Eigen::MatrixBase<MatrixType> & mat)
    {
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) res(mat.rows(), mat.cols());
      inverse(mat, res);
      return res;
    }

    void exposeLinalg()
    {
      using namespace Eigen;

      {
        // using the rpy scope
        bp::scope current_scope = getOrCreatePythonNamespace("linalg");

#ifndef PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED
        bp::def(
          "computeLargestEigenvector",
          (context::VectorXs(*)(const context::MatrixXs &, const int, const context::Scalar))
            & computeLargestEigenvector<context::MatrixXs>,
          (bp::arg("mat"), bp::arg("max_it") = 10, bp::arg("rel_tol") = 1e-8),
          "Compute the lagest eigenvector of a given matrix according to a given eigenvector "
          "estimate.");

        bp::def(
          "retrieveLargestEigenvalue", &retrieveLargestEigenvalue<context::MatrixXs>,
          bp::arg("eigenvector"),
          "Compute the lagest eigenvalue of a given matrix. This is taking the eigenvector "
          "computed by the function computeLargestEigenvector.");
#endif // PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED

        bp::def("inv", &inv<context::MatrixXs>, "Computes the inverse of a matrix.");
#ifdef PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE
        bp::def(
          "inv", &inv<Matrix<long double, Dynamic, Dynamic>>, "Computes the inverse of a matrix.");
#endif
      }
    }

  } // namespace python
} // namespace pinocchio
