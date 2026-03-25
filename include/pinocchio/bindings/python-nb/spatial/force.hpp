// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Force>
void exposeForce(nb::module_ m)
{
  using namespace nb::literals;
  using Self = Force;
  using Scalar = typename Self::Scalar;
  static constexpr int Options = Self::Options;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1, Options>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1, Options>;
  using Vector3Ref = Eigen::Ref<Vector3>;
  using Vector6Ref = Eigen::Ref<Vector6>;
  using SE3 = SE3Tpl<Scalar, Options>;
  using Motion = MotionTpl<Scalar, Options>;

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  auto to_vector_ = [](Self & self) -> Vector6Ref { return self.toVector(); };

  nb::class_<Force>(m, "Force")
    // Constructor
    .def(nb::init<>(), "Default constructor")
    .def(
      nb::init<const Vector3 &, const Vector3 &>(), "linear"_a, "angular"_a,
      "Initialize from linear and angular components of a Wrench vector (do NOT mix the order).")
    .def(nb::init<const Vector6 &>(), "array"_a, "Initialize from a 6D vector [force, torque]")
    .def(nb::init<const Force &>(), "other"_a, "Copy constructor.")
    .def_prop_rw(
      "linear", [](Self & self) -> Vector3Ref { return self.linear(); },
      [](Self & self, const Vector3 & v) { self.linear(v); }, "Linear part of the Force object.")
    .def_prop_rw(
      "angular", [](Self & self) -> Vector3Ref { return self.angular(); },
      [](Self & self, const Vector3 & v) { self.angular(v); }, "Angular part of the Force object.")
    .def_prop_ro("vector", to_vector_, "Returns the components of the Force object as a 6D vector.")
    .def("toVector", to_vector_, "Returns the components of the Force object as a 6D vector.")
    .def_prop_ro("np", to_vector_)
    // Factories
    .def(
      "setZero", &Force::setZero,
      "Set the linear and angular components of the Force object to zero.")
    .def(
      "setRandom", &Force::setRandom,
      "Set the linear and angular components of the Force object to random values.")
    .def_static("Random", &Force::Random, "Returns a random Force.")
    .def_static("Zero", &Force::Zero, "Returns a zero Force.")
    // SE3 action
    .def(
      "se3Action", [](const Self & self, const SE3 & M) { return self.se3Action(M); }, "M"_a,
      "Returns the result of the dual action of M on this Force.")
    .def(
      "se3ActionInverse", [](const Self & self, const SE3 & M) { return self.se3ActionInverse(M); },
      "M"_a, "Returns the result of the dual action of the inverse of M on this Force.")
    // Dot product
    .def(
      "dot", [](const Self & self, const Motion & m) { return self.dot(m); }, "m"_a,
      "Dot product of this Force with a Motion m.")
    // Arithmetic operators
    .def(
      "__add__", [](const Force & a, const Force & b) { return Force(a + b); }, nb::is_operator())
    .def(
      "__sub__", [](const Force & a, const Force & b) { return Force(a - b); }, nb::is_operator())
    .def(
      "__neg__", [](const Force & a) { return Force(-a); }, nb::is_operator())
    .def(
      "__iadd__", [](Force & a, const Force & b) { return Force(a += b); }, nb::is_operator())
    .def(
      "__isub__", [](Force & a, const Force & b) { return Force(a -= b); }, nb::is_operator())
    .def(
      "__mul__", [](const Force & f, Scalar s) { return Force(f * s); }, nb::is_operator())
    .def(
      "__rmul__", [](const Force & f, Scalar s) { return Force(s * f); }, nb::is_operator())
    .def(
      "__truediv__", [](const Force & f, Scalar s) { return Force(f / s); }, nb::is_operator())
    // Comparison
    .def(
      "isApprox",
      [](const Force & self, const Force & other, Scalar prec) {
        return self.isApprox(other, prec);
      },
      "other"_a, "prec"_a = dummy_precision,
      "Returns true if this Force is approximately equal to other, within the precision given by "
      "prec.")
    .def(
      "isZero", [](const Force & self, Scalar prec) { return self.isZero(prec); },
      "prec"_a = dummy_precision,
      "Returns true if this Force is approximately equal to zero, within the precision "
      "given by prec.")
    .def(ComparableVisitor<Force>())
    // Array interface
    .def("__array__", [](const Self & self) { return Vector6(self.toVector()); })
    // Repr and str
    .def(PrintableVisitor<Force>());

  nb::bind_vector<std::vector<Force>, nb::rv_policy::reference_internal>(m, "ForceStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
