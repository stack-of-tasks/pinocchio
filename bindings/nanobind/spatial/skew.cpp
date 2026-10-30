// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

using Vector3 = Eigen::Matrix<Scalar, 3, 1, Options>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3, Options>;

void exposeSkew(nb::module_ m)
{
  m.def(
    "skew", [](const Vector3 & u) -> Matrix3 { return pinocchio::skew(u); }, "u"_a,
    "Computes the skew representation of a given 3d vector, "
    "i.e. the antisymmetric matrix representation of the cross product operator, aka U = "
    "[u]x.\n"
    "Parameters:\n"
    "\tu: the input vector of dimension 3");

  m.def(
    "skewSquare",
    [](const Vector3 & u, const Vector3 & v) -> Matrix3 { return pinocchio::skewSquare(u, v); },
    "u"_a, "v"_a,
    "Computes the skew square representation of two given 3d vectors, "
    "i.e. the antisymmetric matrix representation of the chained cross product operator, u x "
    "(v x w), where w is another 3d vector.\n"
    "Parameters:\n"
    "\tu: the first input vector of dimension 3\n"
    "\tv: the second input vector of dimension 3");

  m.def(
    "unSkew", [](const Matrix3 & U) -> Vector3 { return pinocchio::unSkew(U); }, "U"_a,
    "Inverse of skew operator. From a given skew symmetric matrix U (i.e U = -U.T)"
    "of dimension 3x3, it extracts the supporting vector, i.e. the entries of U.\n"
    "Mathematically speaking, it computes v such that U.dot(x) = cross(u, x).\n"
    "Parameters:\n"
    "\tU: the input skew symmetric matrix of dimension 3x3.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
