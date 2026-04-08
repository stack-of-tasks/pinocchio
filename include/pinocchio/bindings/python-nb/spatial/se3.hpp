// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/copyable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/spatial.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/tuple.h>

#ifdef PINOCCHIO_WITH_COLLISION
  #include <coal/math/transform.h>
#endif

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class SE3>
void exposeSE3(nb::module_ m)
{
  using namespace nb::literals;
  using Self = SE3;
  using Scalar = typename Self::Scalar;
  using Matrix3 = typename Self::Matrix3;
  using Matrix4 = typename Self::Matrix4;
  using Vector3 = typename Self::Vector3;
  using Quaternion = typename Self::Quaternion;
  using Motion = MotionTpl<Scalar, Self::Options>;
  using Force = ForceTpl<Scalar, Self::Options>;
  using Inertia = InertiaTpl<Scalar, Self::Options>;

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  nb::class_<SE3>(
    m, "SE3", "Rigid transformation defined by a 3D translation vector and rotation matrix.")
    // Constructors
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const SE3 &>(), "clone"_a, "Copy constructor.")
    .def(
      nb::init<const Matrix3 &, const Vector3 &>(), "rotation"_a, "translation"_a,
      "Initialize from a rotation matrix and a translation vector.")
    .def(
      nb::init<const Quaternion &, const Vector3 &>(), "quat"_a, "translation"_a,
      "Initialize from a quaternion and a translation vector.")
    .def(nb::init<int>(), "int"_a, "Init to identity.")
    .def(nb::init<const Matrix4 &>(), "array"_a, "Initialize from a homogeneous matrix.")
#ifdef PINOCCHIO_WITH_COLLISION
    .def(
      "__init__",
      [](SE3 * self, const coal::Transform3s & t) {
        new (self) SE3(t.getRotation(), t.getTranslation());
      },
      "transform"_a, "Initialize from a coal.Transform3s.")
#endif
    // Properties
    .def_prop_rw(
      "rotation", [](Self & self) { return make_ref(self.rotation()); },
      [](Self & self, const Matrix3 & R) { self.rotation(R); },
      "The rotation part of the transformation.")
    .def_prop_rw(
      "translation", [](Self & self) { return make_ref(self.translation()); },
      [](Self & self, const Vector3 & t) { self.translation(t); },
      "The translation part of the transformation.")
    .def_prop_ro(
      "homogeneous", &Self::toHomogeneousMatrix, "Returns the equivalent 4x4 homogeneous matrix.")
    .def_prop_ro(
      "np", &Self::toHomogeneousMatrix, "Returns the homogeneous matrix (alias for homogeneous).")
    // Action matrices
    .def_prop_ro(
      "action", [](const Self & self) { return self.toActionMatrix(); },
      "Returns the related action matrix (acting on Motion).")
    .def(
      "toActionMatrix", [](const Self & self) { return self.toActionMatrix(); },
      "Returns the related action matrix (acting on Motion).")
    .def_prop_ro(
      "actionInverse", [](const Self & self) { return self.toActionMatrixInverse(); },
      "Returns the inverse of the action matrix (acting on Motion).\n"
      "This is equivalent to do m.inverse().action")
    .def(
      "toActionMatrixInverse", [](const Self & self) { return self.toActionMatrixInverse(); },
      "Returns the inverse of the action matrix (acting on Motion).\n"
      "This is equivalent to do m.inverse().toActionMatrix()")
    .def_prop_ro(
      "dualAction", [](const Self & self) { return self.toDualActionMatrix(); },
      "Returns the related dual action matrix (acting on Force).")
    .def(
      "toDualActionMatrix", [](const Self & self) { return self.toDualActionMatrix(); },
      "Returns the related dual action matrix (acting on Force).")
    // Mutating methods
    .def("setIdentity", &SE3::setIdentity, "Set this SE3 to the identity placement.")
    .def("setRandom", &SE3::setRandom, "Set this SE3 to a random placement.")
    // Inverse
    .def("inverse", &SE3::inverse, "Returns the inverse rigid transformation.")
    .def("__invert__", &SE3::inverse, nb::is_operator(), "Returns the inverse of this SE3.")
    // act / actInv overloads
    .def(
      "act", [](const SE3 & self, const Vector3 & v) { return self.act(v); }, "point"_a,
      "Returns a point which is the result of the entry point transformed by this SE3.")
    .def(
      "actInv", [](const SE3 & self, const Vector3 & v) { return self.actInv(v); }, "point"_a,
      "Returns a point which is the result of the entry point transformed by the inverse of this "
      "SE3.")
    .def(
      "act", [](const SE3 & self, const SE3 & M) { return self.act(M); }, "M"_a,
      "Returns the result of this SE3 * M.")
    .def(
      "actInv", [](const SE3 & self, const SE3 & M) { return self.actInv(M); }, "M"_a,
      "Returns the result of the inverse of this SE3 times M.")
    .def(
      "act", [](const SE3 & self, const Motion & m) { return self.act(m); }, "motion"_a,
      "Returns the result of the action of this SE3 onto a Motion.")
    .def(
      "actInv", [](const SE3 & self, const Motion & m) { return self.actInv(m); }, "motion"_a,
      "Returns the result of the inverse of this SE3 onto a Motion.")
    .def(
      "act", [](const SE3 & self, const Inertia & I) { return self.act(I); }, "inertia"_a,
      "Returns the result of this SE3 onto an Inertia.")
    .def(
      "actInv", [](const SE3 & self, const Inertia & I) { return self.actInv(I); }, "inertia"_a,
      "Returns the result of the inverse of this SE3 onto an Inertia.")
    // Operators
    .def(
      "__mul__", [](const SE3 & self, const SE3 & other) { return self.act(other); },
      nb::is_operator())
    .def(
      "__mul__", [](const SE3 & self, const Motion & m) { return self.act(m); }, nb::is_operator())
    .def(
      "__mul__", [](const SE3 & self, const Force & m) { return self.act(m); }, nb::is_operator())
    .def(
      "__mul__", [](const SE3 & self, const Vector3 & v) { return self.act(v); }, nb::is_operator())
    .def(
      "__mul__", [](const SE3 & self, const Inertia & I) { return self.act(I); }, nb::is_operator())
    // Comparison
    .def(
      "isApprox", &SE3::isApprox, "other"_a, nb::arg("prec") = dummy_precision,
      "Returns true if this SE3 is approximately equal to other, within the precision given by "
      "prec.")
    .def(
      "isIdentity", &SE3::isIdentity, nb::arg("prec") = dummy_precision,
      "Returns true if this SE3 is approximately equal to the identity placement, within the "
      "precision given by prec.")
    .def(ComparableVisitor<SE3>())
    .def(CopyableVisitor<SE3>())
    // Static factory methods
    .def_static("Identity", &SE3::Identity, "Returns the identity transformation.")
    .def_static("Random", &SE3::Random, "Returns a random transformation.")
    .def_static(
      "Interpolate", &SE3::template Interpolate<Scalar>, "A"_a, "B"_a, "alpha"_a,
      "Linear interpolation on the SE3 manifold.\n\n"
      "This method computes the linear interpolation between A and B, such that the result C "
      "= A + (B-A)*t if it would be applied on classic Euclidian space.\n"
      "This operation is very similar to the SLERP operation on Rotations.\n"
      "Parameters:\n"
      "\tA: Initial transformation\n"
      "\tB: Target transformation\n"
      "\talpha: Interpolation factor")
    // Array interface
    .def("__array__", [](const Self & self) { return self.toHomogeneousMatrix(); })
    // Pickle
    .def(
      "__getstate__",
      [](const SE3 & self) {
        return std::make_tuple(Matrix3(self.rotation()), Vector3(self.translation()));
      })
    .def(
      "__setstate__",
      [](SE3 & self, std::tuple<Matrix3, Vector3> t) {
        new (&self) SE3(std::get<0>(t), std::get<1>(t));
      })
    // String representation
    .def(PrintableVisitor<SE3>());

  nb::bind_vector<std::vector<SE3>, nb::rv_policy::reference_internal>(m, "StdVec_SE3");

#ifdef PINOCCHIO_WITH_COLLISION
  nb::implicitly_convertible<coal::Transform3s, SE3>();
#endif
}
PINOCCHIO_PYTHON_NAMESPACE_END
