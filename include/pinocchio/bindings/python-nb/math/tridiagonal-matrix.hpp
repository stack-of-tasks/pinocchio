// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"

#include "pinocchio/math.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<typename Scalar, int Options>
void exposeTridiagonalMatrix(nb::module_ m)
{
  using namespace nb::literals;
  using TridiagonalMatrix = TridiagonalSymmetricMatrixTpl<Scalar, Options>;
  using CoeffVectorType = typename TridiagonalMatrix::CoeffVectorType;
  using PlainMatrix = typename TridiagonalMatrix::PlainMatrixType;
  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  nb::class_<TridiagonalMatrix>(m, "TridiagonalMatrix", "Tridiagonal symmetric matrix.")
    .def(nb::init<Eigen::Index>(), "size"_a, "Default constructor from a given size.")
    .def(
      "diagonal", (CoeffVectorType & (TridiagonalMatrix::*)()) & TridiagonalMatrix::diagonal,
      "Reference of the diagonal elements of the symmetric tridiagonal matrix.")
    .def(
      "subDiagonal", (CoeffVectorType & (TridiagonalMatrix::*)()) & TridiagonalMatrix::subDiagonal,
      "Reference of the sub-diagonal elements of the symmetric tridiagonal matrix.")
    .def(
      "setIdentity", &TridiagonalMatrix::setIdentity,
      "Set the current tridiagonal matrix to the identity matrix.")
    .def(
      "setZero", &TridiagonalMatrix::setZero,
      "Set the current tridiagonal matrix coefficients to zero.")
    .def(
      "setRandom", &TridiagonalMatrix::setRandom,
      "Set the current tridiagonal matrix coefficients to random values.")
  // comparisons
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
    .def(
      "isIdentity", &TridiagonalMatrix::isIdentity, "prec"_a = dummy_precision,
      "Returns true if *this is approximately equal to the identity matrix, within the "
      "precision given by prec.")
    .def(
      "isZero", &TridiagonalMatrix::isZero, "prec"_a = dummy_precision,
      "Returns true if *this is approximately equal to the zero matrix, within the "
      "precision given by prec.")
    .def(
      "isDiagonal", &TridiagonalMatrix::isDiagonal, "prec"_a = dummy_precision,
      "Returns true if *this is approximately equal to the a diagonal matrix, within the "
      "precision given by prec.")
#endif // PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
    // set coeffs
    .def(
      "setDiagonal",
      [](TridiagonalMatrix & self, Eigen::Ref<const CoeffVectorType> coeffs) {
        self.setDiagonal(coeffs);
      },
      "diagonal"_a,
      "Set the current tridiagonal matrix's diagonal coefficients to those given by the input "
      "vector.")
    // sizes
    .def("rows", &TridiagonalMatrix::rows, "Number of rows in the tridiagonal matrix.")
    .def("cols", &TridiagonalMatrix::cols, "Number of columns in the tridiagonal matrix.")
    .def("matrix", &TridiagonalMatrix::matrix, "Return a plain NumPy array.")
    // util
    .def(
      "computeEigenValue", &TridiagonalMatrix::computeEigenvalue, "eigenvalue_index"_a,
      "eps"_a = 1e-8,
      "Computes the kth eigenvalue associated with the input tridiagonal matrix up to "
      "precision eps.")
    .def(
      "computeSpectrum", &TridiagonalMatrix::computeSpectrum, "eps"_a = 1e-8,
      "Computes the full spectrum associated with the input tridiagonal matrix up to "
      "precision eps.")
    // operators
    .def(
      "__mul__",
      [](const TridiagonalMatrix & self, const PlainMatrix & mat) {
        return PlainMatrix(self * mat);
      },
      nb::is_operator())
    .def(
      "__rmul__",
      [](const TridiagonalMatrix & self, const PlainMatrix & mat) {
        return PlainMatrix(mat * self);
      },
      nb::is_operator())
    .def(ComparableVisitor<TridiagonalMatrix>());
}

PINOCCHIO_PYTHON_NAMESPACE_END
