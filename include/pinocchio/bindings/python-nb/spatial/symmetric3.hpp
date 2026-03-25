// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Symmetric3>
void exposeSymmetric3(nb::module_ m)
{
  using namespace nb::literals;
  using Self = Symmetric3;
  using Scalar = typename Self::Scalar;
  using Vector3 = typename Self::Vector3;
  using Vector6 = typename Self::Vector6;
  using Matrix3 = typename Self::Matrix3;
  using Matrix32 = typename Self::Matrix32;

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  nb::class_<Symmetric3>(
    m, "Symmetric3",
    "Symmetric 3x3 matrix stored as a 6D vector [a0, a1, a2, a3, a4, a5] representing\n"
    "[[a0, a1, a3],\n"
    " [a1, a2, a4],\n"
    " [a3, a4, a5]].")
    // Constructors
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const Symmetric3 &>(), "other"_a, "Copy constructor.")
    .def(nb::init<const Matrix3 &>(), "I"_a, "Initialize from a symmetric 3x3 matrix.")
    .def(nb::init<const Vector6 &>(), "I"_a, "Initialize from a 6D vector.")
    .def(
      "__init__",
      [](Symmetric3 * self, Scalar a0, Scalar a1, Scalar a2, Scalar a3, Scalar a4, Scalar a5) {
        new (self) Symmetric3(a0, a1, a2, a3, a4, a5);
      },
      "a0"_a, "a1"_a, "a2"_a, "a3"_a, "a4"_a, "a5"_a, "Initialize from 6 scalar values.")
    // Properties
    .def_prop_rw(
      "data", [](const Self & self) { return Vector6(self.data()); },
      [](Self & self, const Vector6 & d) { self.data() = d; },
      "6D vector containing the data of the symmetric 3x3 matrix.")
    // Matrix representation
    .def("matrix", &Symmetric3::matrix, "Returns a 3x3 matrix representation of this Symmetric3.")
    .def_prop_ro(
      "np", &Symmetric3::matrix, "Returns a 3x3 matrix representation (alias for matrix()).")
    // Static factory methods
    .def_static("Zero", &Symmetric3::Zero, "Returns the zero symmetric 3x3 matrix.")
    .def_static("Random", &Symmetric3::Random, "Returns a random symmetric 3x3 matrix.")
    .def_static("Identity", &Symmetric3::Identity, "Returns the identity symmetric 3x3 matrix.")
    // Mutating methods
    .def("setZero", &Symmetric3::setZero, "Set all the components of this Symmetric3 to zero.")
    .def(
      "setRandom", &Symmetric3::setRandom,
      "Set all the components of this Symmetric3 to random values.")
    .def(
      "setIdentity", &Symmetric3::setIdentity, "Set the components of this Symmetric3 to identity.")
    .def(
      "setDiagonal",
      [](Self & self, const Vector3 & diag) { self.template setDiagonal<Vector3>(diag); }, "diag"_a,
      "Set the diagonal elements of the 3x3 matrix.")
    .def("fill", &Symmetric3::fill, "value"_a, "Set all 6 components of this Symmetric3 to value.")
    // Inverse
    .def(
      "inverse", (Matrix3(Symmetric3::*)() const)&Symmetric3::inverse,
      "Returns the inverse of this symmetric 3x3 matrix.")
    // Quadratic form
    .def("vtiv", &Symmetric3::vtiv, "v"_a, "Returns the scalar v^T * S * v.")
    // Cross-product operations
    // .def(
    //   "vxs",
    //   [](const Self & self, const Vector3 & v) { return Matrix3(self.template vxs<Vector3>(v));
    //   }, "v"_a, "Performs the operation [v]_x * S, returning a 3x3 matrix.")
    .def_static(
      "vxs",
      [](const Vector3 & v, const Self & S3) {
        Matrix3 M;
        Self::template vxs<Vector3, Matrix3>(v, S3, M);
        return M;
      },
      "v"_a, "S3"_a, "Performs the operation M = [v]_x * S3, returning a 3x3 matrix.")
    // .def(
    //   "svx",
    //   [](const Self & self, const Vector3 & v) { return Matrix3(self.template svx<Vector3>(v));
    //   }, "v"_a, "Performs the operation S * [v]_x, returning a 3x3 matrix.")
    .def_static(
      "svx",
      [](const Vector3 & v, const Self & S3) {
        Matrix3 M;
        Self::template svx<Vector3, Matrix3>(v, S3, M);
        return M;
      },
      "v"_a, "S3"_a, "Performs the operation M = S3 * [v]_x, returning a 3x3 matrix.")
    .def_static(
      "rhsMult",
      [](const Self & S3, const Vector3 & vin) {
        Vector3 vout;
        Self::template rhsMult<Vector3, Vector3>(S3, vin, vout);
        return vout;
      },
      "S3"_a, "vin"_a, "Computes and returns S3 * vin.")
    .def("decomposeltI", &Symmetric3::decomposeltI, "Computes L for this symmetric matrix.")
    .def(
      "rotate",
      [](const Self & self, const Matrix3 & R) {
        return Symmetric3(self.template rotate<Matrix3>(R));
      },
      "R"_a, "Computes R * S * R^T.")
    // Arithmetic operators
    .def(
      "__add__", [](const Self & a, const Self & b) { return Self(a + b); }, nb::is_operator())
    .def(
      "__iadd__", [](Self & a, const Self & b) -> Self & { return a += b; }, nb::is_operator())
    .def(
      "__sub__", [](const Self & a, const Self & b) { return Self(a - b); }, nb::is_operator())
    .def(
      "__isub__", [](Self & a, const Self & b) -> Self & { return a -= b; }, nb::is_operator())
    .def(
      "__imul__", [](Self & a, Scalar s) -> Self & { return a *= s; }, nb::is_operator())
    .def(
      "__mul__", [](const Self & a, const Vector3 & v) { return Vector3(a * v); },
      nb::is_operator())
    .def(
      "__sub__", [](const Self & a, const Matrix3 & S) { return Self(a - S); }, nb::is_operator())
    .def(
      "__add__", [](const Self & a, const Matrix3 & S) { return Self(a + S); }, nb::is_operator())
    // Comparison
    .def(
      "isApprox",
      [](const Self & self, const Self & other, Scalar prec) { return self.isApprox(other, prec); },
      "other"_a, nb::arg("prec") = dummy_precision,
      "Returns true if this Symmetric3 is approximately equal to other, within the precision "
      "given by prec.")
    .def(
      "isZero", [](const Self & self, Scalar prec) { return self.isZero(prec); },
      nb::arg("prec") = dummy_precision,
      "Returns true if this Symmetric3 is approximately equal to the zero matrix, within the "
      "precision given by prec.")
    .def(ComparableVisitor<Symmetric3>())
    // Array interface
    .def("__array__", [](const Self & self) { return Matrix3(self.matrix()); })
    // String representation
    .def(PrintableVisitor<Symmetric3>());
}
PINOCCHIO_PYTHON_NAMESPACE_END
