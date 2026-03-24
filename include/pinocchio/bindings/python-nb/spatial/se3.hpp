// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class SE3>
void exposeSE3(nb::module_ m)
{
  using namespace nb::literals;
  using Self = SE3;
  using Vector3 = typename Self::Vector3;
  nb::class_<SE3>(
    m, "SE3", "Rigid transformation defined by a 3D translation vector and rotation matrix.")
    .def(nb::init<>())
    .def_prop_ro("translation", [](Self & self) { return self.translation(); })
    .def_prop_ro("rotation", [](Self & self) { return self.rotation(); })
    .def_prop_ro(
      "homogeneous", &SE3::toHomogeneousMatrix, "Returns the equivalent 4x4 homogeneous matrix.")
    //
    .def("setIdentity", &SE3::setIdentity, "Initialize this SE3 object to the identity transform.")
    .def(
      "setRandom", &SE3::setRandom, "Initialize this SE3 object to a random rigid transformation.")
    //
    .def("inverse", &SE3::inverse, "Return the inverse rigid transformation.")
    // action
    .def(
      "act", [](const SE3 & self, const Vector3 & v) { return self.act(v); }, "point"_a)
    .def(
      "actInv", [](const SE3 & self, const Vector3 & v) { return self.actInv(v); }, "point"_a)
    .def_prop_ro(
      "action", [](Self const & self) { return self.toActionMatrix(); },
      "Returns the related action matrix (acting on Motion).")
    .def_prop_ro(
      "toActionMatrix", [](Self const & self) { return self.toActionMatrix(); },
      "Returns the related action matrix (acting on Motion).");
}
PINOCCHIO_PYTHON_NAMESPACE_END
