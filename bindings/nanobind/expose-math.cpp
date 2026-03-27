// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/math.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

template<typename MatrixType>
typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) inv(const Eigen::MatrixBase<MatrixType> & mat)
{
  typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) res(mat.rows(), mat.cols());
  inverse(mat, res);
  return res;
}

static void exposeLinalg(nb::module_ m_)
{
  auto m = m_.def_submodule("linalg");

#ifndef PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED
  m.def(
    "computeLargestEigenvector", &computeLargestEigenvector<MatrixXs>, "mat"_a, "max_it"_a = 10,
    "rel_tol"_a = 1e-8,
    "Compute the lagest eigenvector of a given matrix according to a given eigenvector "
    "estimate.");

  m.def(
    "retrieveLargestEigenvalue", &retrieveLargestEigenvalue<MatrixXs>, "eigenvector"_a,
    "Compute the lagest eigenvalue of a given matrix. This is taking the eigenvector "
    "computed by the function computeLargestEigenvector.");
#endif

  m.def("inv", &inv<MatrixXs>, "Computes the inverse of a matrix.");
  m.def(
    "inv", &inv<Eigen::Matrix<long double, Dynamic, Dynamic>>, "Computes the inverse of a matrix.");
}

void exposeMathUtil(nb::module_ m)
{

  exposeLinalg(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
