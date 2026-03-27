// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/comparable.hpp"

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

static void exposeLanczosDecomposition(nb::module_ m)
{
  using LanczosDecomposition = LanczosDecompositionTpl<MatrixXs>;
  using TridiagonalMatrix = LanczosDecomposition::TridiagonalMatrix;
  using PlainMatrix = LanczosDecomposition::PlainMatrix;

  nb::class_<LanczosDecomposition>(m, "LanczosDecomposition", "Lanczos decomposition")
    .def(
      nb::init<const MatrixXs &, Eigen::Index>(), "matrix"_a, "decomposition_size"_a,
      "Default constructor from a given matrix and a given decomposition size.")
    .def(
      nb::init<Eigen::Index, Eigen::Index>(), "size"_a, "decomposition_size"_a,
      "Default constructor for the Lanczos decomposition from a given matrix size.")
    .def(
      "compute", &LanczosDecomposition::compute<MatrixXs>, "matrix"_a,
      "Computes the Lanczos decomposition for the given input matrix.")
    // decompositon data
    .def(
      "Ts", (TridiagonalMatrix & (LanczosDecomposition::*)()) & LanczosDecomposition::Ts,
      "Returns the tridiagonal matrix associated with the Lanczos decomposition.",
      nb::rv_policy::reference_internal)
    .def(
      "Qs", (PlainMatrix & (LanczosDecomposition::*)()) & LanczosDecomposition::Qs,
      "Returns the orthogonal basis associated with the Lanczos decomposition.",
      nb::rv_policy::reference_internal)
    //
    .def(
      "computeDecompositionResidual", &LanczosDecomposition::computeDecompositionResidual<MatrixXs>,
      "matrix"_a,
      "Computes the residual associated with the decomposition, namely, the quantity \f$ A "
      "Q_s - Q_s T_s \f$")
    // size
    .def("size", &LanczosDecomposition::size, "Returns the size of the underlying matrix.")
    .def(
      "decompositionSize", &LanczosDecomposition::decompositionSize,
      "Returns the size of the decomposition.")
    // comparison
    .def(ComparableVisitor<LanczosDecomposition>());
}

void exposeMathUtil(nb::module_ m)
{

  exposeLinalg(m);
  exposeLanczosDecomposition(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
