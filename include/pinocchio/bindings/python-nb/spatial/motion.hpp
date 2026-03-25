// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/tuple.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Motion>
void exposeMotion(nb::module_ m)
{
  using namespace nb::literals;
  using Self = Motion;
  using Scalar = typename Self::Scalar;
  using Vector3 = typename Self::Vector3;
  using Vector6 = typename Self::Vector6;
  using SE3 = SE3Tpl<Scalar, Self::Options>;

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  nb::class_<Motion>(
    m, "Motion",
    "Motion vectors, in se3 == M^6.\n\n"
    "A spatial motion vector (v, w) where v is the linear velocity and w is the angular velocity.")
    // Constructors
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const Motion &>(), "clone"_a, "Copy constructor.")
    .def(
      nb::init<const Vector3 &, const Vector3 &>(), "linear"_a, "angular"_a,
      "Initialize from linear and angular components of a Motion vector (don't mix the order).")
    .def(
      nb::init<const Vector6 &>(), "array"_a,
      "Init from a vector 6 [linear velocity, angular velocity].")
    // Properties
    .def_prop_rw(
      "linear", [](Self & self) { return Vector3(self.linear()); },
      [](Self & self, const Vector3 & v) { self.linear(v); },
      "Linear part of the Motion object, corresponding to the linear velocity in case of a Spatial "
      "velocity.")
    .def_prop_rw(
      "angular", [](Self & self) { return Vector3(self.angular()); },
      [](Self & self, const Vector3 & w) { self.angular(w); },
      "Angular part of the Motion object, corresponding to the angular velocity in case of a "
      "Spatial "
      "velocity.")
    .def_prop_rw(
      "vector", [](const Self & self) { return Vector6(self.toVector()); },
      [](Self & self, const Vector6 & v) { self = v; },
      "Returns the components of this Motion as a 6d vector.")
    .def_prop_ro(
      "np", [](const Self & self) { return Vector6(self.toVector()); },
      "Returns the components of this Motion as a 6d vector.")
    .def_prop_ro(
      "action", [](const Self & self) { return self.toActionMatrix(); },
      "Returns the action matrix of this Motion (acting on Motion).")
    .def_prop_ro(
      "dualAction", [](const Self & self) { return self.toDualActionMatrix(); },
      "Returns the dual action matrix of this Motion (acting on Force).")
    .def_prop_ro(
      "homogeneous", [](const Self & self) { return self.toHomogeneousMatrix(); },
      "Equivalent homogeneous representation of the Motion vector.")
    // Mutating methods
    .def(
      "setZero", [](Self & self) { self.setZero(); },
      "Set the linear and angular components of this Motion to zero.")
    .def(
      "setRandom", [](Self & self) { self.setRandom(); },
      "Set the linear and angular components of this Motion to random values.")
    // SE3 action
    .def(
      "se3Action", [](const Self & self, const SE3 & M) { return self.se3Action(M); }, "M"_a,
      "Returns the result of the action of M on this Motion.")
    .def(
      "se3ActionInverse", [](const Self & self, const SE3 & M) { return self.se3ActionInverse(M); },
      "M"_a, "Returns the result of the action of the inverse of M on this Motion.")
    // Cross product (Motion x Motion only; Force not yet bound)
    .def(
      "cross", [](const Self & self, const Motion & m) { return Motion(self.cross(m)); }, "m"_a,
      "Action of this Motion onto another Motion m. Returns self ^ m.")
    // Arithmetic operators
    .def(
      "__add__", [](const Motion & a, const Motion & b) { return Motion(a + b); },
      nb::is_operator())
    .def(
      "__sub__", [](const Motion & a, const Motion & b) { return Motion(a - b); },
      nb::is_operator())
    .def(
      "__neg__", [](const Motion & a) { return Motion(-a); }, nb::is_operator())
    .def(
      "__iadd__", [](Motion & a, const Motion & b) { return Motion(a += b); }, nb::is_operator())
    .def(
      "__isub__", [](Motion & a, const Motion & b) { return Motion(a -= b); }, nb::is_operator())
    .def(
      "__mul__", [](const Motion & m, Scalar s) { return Motion(m * s); }, nb::is_operator())
    .def(
      "__rmul__", [](const Motion & m, Scalar s) { return Motion(s * m); }, nb::is_operator())
    .def(
      "__truediv__", [](const Motion & m, Scalar s) { return Motion(m / s); }, nb::is_operator())
    .def(
      "__xor__", [](const Motion & a, const Motion & b) { return Motion(a.cross(b)); },
      nb::is_operator())
    // Comparison
    .def(
      "isApprox",
      [](const Motion & self, const Motion & other, Scalar prec) {
        return self.isApprox(other, prec);
      },
      "other"_a, nb::arg("prec") = dummy_precision,
      "Returns true if this Motion is approximately equal to other, within the precision given by "
      "prec.")
    .def(
      "isZero", [](const Motion & self, Scalar prec) { return self.isZero(prec); },
      nb::arg("prec") = dummy_precision,
      "Returns true if this Motion is approximately equal to zero, within the precision "
      "given by prec.")
    .def(
      "__eq__", [](const Motion & a, const Motion & b) { return a == b; }, nb::is_operator())
    .def(
      "__ne__", [](const Motion & a, const Motion & b) { return a != b; }, nb::is_operator())
    // Static factory methods
    .def_static("Zero", &Motion::Zero, "Returns a zero Motion.")
    .def_static("Random", &Motion::Random, "Returns a random Motion.")
    // Array interface
    .def("__array__", [](const Self & self) { return Vector6(self.toVector()); })
    // Pickle
    .def(
      "__getstate__",
      [](const Motion & self) {
        return std::make_tuple(Vector3(self.linear()), Vector3(self.angular()));
      })
    .def(
      "__setstate__",
      [](Motion & self, std::tuple<Vector3, Vector3> t) {
        new (&self) Motion(std::get<0>(t), std::get<1>(t));
      })
    // String representation
    .def(PrintableVisitor<Motion>());

  nb::bind_vector<std::vector<Motion>>(m, "MotionStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
