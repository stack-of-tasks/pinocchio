// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "pinocchio/multibody.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace
{
  using nb::literals::operator""_a;
}

/// Visitor that adds type-specific constructors and properties to each concrete JointData
/// class. execute() dispatches via tag to an overloaded static expose(); the primary
/// template is a no-op so types with no extra bindings require no boilerplate.
///
/// This visitor is only defined for the current context Scalar type.
struct JointDataDerivedVisitor : nb::def_visitor<JointDataDerivedVisitor>
{
  template<typename T>
  struct Tag
  {
  };

  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  // ── default: nothing extra
  template<typename T, typename PyClass>
  static void expose(Tag<T>, PyClass &)
  {
  }

  // ── Unaligned joints: constructor from axis ─────────────────────────────────────────────────
  template<typename PyClass>
  static void expose(Tag<JointDataRevoluteUnaligned>, PyClass & cl)
  {
    cl.def(
      nb::init<const Vector3 &>(), "axis"_a,
      "Init JointDataRevoluteUnaligned from an axis with x-y-z components.");
  }

  template<typename PyClass>
  static void expose(Tag<JointDataPrismaticUnaligned>, PyClass & cl)
  {
    cl.def(
      nb::init<const Vector3 &>(), "axis"_a,
      "Init JointDataPrismaticUnaligned from an axis with x-y-z components.");
  }

  template<typename PyClass>
  static void expose(Tag<JointDataHelicalUnaligned>, PyClass & cl)
  {
    cl.def(
      nb::init<const Vector3 &>(), "axis"_a,
      "Init JointDataHelicalUnaligned from an axis with x-y-z components.");
  }

  // ── Joints with StU ─────────────────────────────────────────────────────────────────────────
  template<typename PyClass>
  static void expose(Tag<JointDataPlanar>, PyClass & cl)
  {
    cl.def_ro("StU", &JointDataPlanar::StU);
  }

  template<typename PyClass>
  static void expose(Tag<JointDataSphericalZYX>, PyClass & cl)
  {
    cl.def_ro("StU", &JointDataSphericalZYX::StU);
  }

  template<typename PyClass>
  static void expose(Tag<JointDataEllipsoid>, PyClass & cl)
  {
    cl.def_ro("StU", &JointDataEllipsoid::StU);
  }

  // ── JointDataComposite ──────────────────────────────────────────────────────────────────────
  template<typename PyClass>
  static void expose(Tag<JointDataComposite>, PyClass & cl)
  {
    using JointDataVector = JointDataComposite::JointDataVector;
    cl.def(
        nb::init<const JointDataVector &, int, int>(), "joint_data_vector"_a, "nq"_a, "nv"_a,
        "Init JointDataComposite from a collection of joint data.")
      .def_ro("joints", &JointDataComposite::joints)
      .def_ro("iMlast", &JointDataComposite::iMlast)
      .def_ro("pjMi", &JointDataComposite::pjMi)
      .def_ro("StU", &JointDataComposite::StU);
  }

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    expose(Tag<typename PyClass::Type>{}, cl);
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
