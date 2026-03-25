// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/printable.hpp"

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
    //
    // Factories
    .def(
      "setZero", &Force::setZero,
      "Set the linear and angular components of the Force object to zero.")
    .def(
      "setRandom", &Force::setRandom,
      "Set the linear and angular components of the Force object to random values.")
    .def_static("Random", &Force::Random, "Returns a random Force.")
    .def_static("Zero", &Force::Zero, "Returns a zero Force.")
    //
    .def(PrintableVisitor<Force>());
}
PINOCCHIO_PYTHON_NAMESPACE_END
