// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "pinocchio/multibody.hpp"

#include <boost/variant.hpp>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace
{
  using nb::literals::operator""_a;
}

/// Visitor that adds type-specific constructors, properties, and methods to each concrete
/// JointModel class. execute() dispatches via tag to an overloaded static expose(); the
/// primary template is a no-op so types with no extra bindings require no boilerplate.
///
/// This visitor is only defined for the current context Scalar type.
struct JointModelDerivedVisitor : nb::def_visitor<JointModelDerivedVisitor>
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

  // ── JointModelRevolute (aligned)
  template<typename PyClass>
  static void expose(Tag<JointModelRX>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRX::getMotionAxis, "Rotation axis of the JointModelRX.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelRY>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRY::getMotionAxis, "Rotation axis of the JointModelRY.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelRZ>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRZ::getMotionAxis, "Rotation axis of the JointModelRZ.");
  }

  // ── JointModelRevoluteUnaligned
  template<typename PyClass>
  static void expose(Tag<JointModelRevoluteUnaligned>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar, Scalar, Scalar>(), "x"_a, "y"_a, "z"_a,
        "Init JointModelRevoluteUnaligned from the components x, y, z of the axis.")
      .def(
        nb::init<const Vector3 &>(), "axis"_a,
        "Init JointModelRevoluteUnaligned from an axis with x-y-z components.")
      .def_rw(
        "axis", &JointModelRevoluteUnaligned::axis,
        "Rotation axis of the JointModelRevoluteUnaligned.");
  }

  // ── JointModelRevoluteUnbounded (aligned)
  template<typename PyClass>
  static void expose(Tag<JointModelRUBX>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRUBX::getMotionAxis, "Rotation axis of the JointModelRUBX.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelRUBY>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRUBY::getMotionAxis, "Rotation axis of the JointModelRUBY.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelRUBZ>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelRUBZ::getMotionAxis, "Rotation axis of the JointModelRUBZ.");
  }

  // ── JointModelPrismatic (aligned)
  template<typename PyClass>
  static void expose(Tag<JointModelPX>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelPX::getMotionAxis, "Translation axis of the JointModelPX.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelPY>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelPY::getMotionAxis, "Translation axis of the JointModelPY.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelPZ>, PyClass & cl)
  {
    cl.def("getMotionAxis", &JointModelPZ::getMotionAxis, "Translation axis of the JointModelPZ.");
  }

  // ── JointModelPrismaticUnaligned
  template<typename PyClass>
  static void expose(Tag<JointModelPrismaticUnaligned>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar, Scalar, Scalar>(), "x"_a, "y"_a, "z"_a,
        "Init JointModelPrismaticUnaligned from the components x, y, z of the axis.")
      .def(
        nb::init<const Vector3 &>(), "axis"_a,
        "Init JointModelPrismaticUnaligned from an axis with x-y-z components.")
      .def_rw(
        "axis", &JointModelPrismaticUnaligned::axis,
        "Translation axis of the JointModelPrismaticUnaligned.");
  }

  // ── JointModelHelicalUnaligned
  template<typename PyClass>
  static void expose(Tag<JointModelHelicalUnaligned>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar, Scalar, Scalar, Scalar>(), "x"_a, "y"_a, "z"_a, "pitch"_a,
        "Init JointModelHelicalUnaligned from the components x, y, z of the axis and the pitch.")
      .def(
        nb::init<const Vector3 &, Scalar>(), "axis"_a, "pitch"_a,
        "Init JointModelHelicalUnaligned from an axis with x-y-z components and the pitch.")
      .def_rw(
        "axis", &JointModelHelicalUnaligned::axis,
        "Rotation axis of the JointModelHelicalUnaligned.")
      .def_rw(
        "pitch", &JointModelHelicalUnaligned::m_pitch,
        "Pitch h of the JointModelHelicalUnaligned.");
  }

  // ── JointModelHelical (aligned)
  template<typename PyClass>
  static void expose(Tag<JointModelHX>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar>(), "pitch"_a,
        "Init JointModelHX with a pitch value and the X axis ([1, 0, 0]) as rotation axis.")
      .def("getMotionAxis", &JointModelHX::getMotionAxis, "Rotation axis of the JointModelHX.")
      .def_rw("pitch", &JointModelHX::m_pitch, "Pitch h of the JointModelHX.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelHY>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar>(), "pitch"_a,
        "Init JointModelHY with a pitch value and the Y axis ([0, 1, 0]) as rotation axis.")
      .def("getMotionAxis", &JointModelHY::getMotionAxis, "Rotation axis of the JointModelHY.")
      .def_rw("pitch", &JointModelHY::m_pitch, "Pitch h of the JointModelHY.");
  }

  template<typename PyClass>
  static void expose(Tag<JointModelHZ>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar>(), "pitch"_a,
        "Init JointModelHZ with a pitch value and the Z axis ([0, 0, 1]) as rotation axis.")
      .def("getMotionAxis", &JointModelHZ::getMotionAxis, "Rotation axis of the JointModelHZ.")
      .def_rw("pitch", &JointModelHZ::m_pitch, "Pitch h of the JointModelHZ.");
  }

  // ── JointModelEllipsoid
  template<typename PyClass>
  static void expose(Tag<JointModelEllipsoid>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar, Scalar, Scalar>(), "radius_x"_a, "radius_y"_a, "radius_z"_a,
        "Init JointModelEllipsoid with radii along the x, y, z axes.")
      .def_rw(
        "radius_x", &JointModelEllipsoid::radius_x,
        "Radius of the JointModelEllipsoid along the X axis.")
      .def_rw(
        "radius_y", &JointModelEllipsoid::radius_y,
        "Radius of the JointModelEllipsoid along the Y axis.")
      .def_rw(
        "radius_z", &JointModelEllipsoid::radius_z,
        "Radius of the JointModelEllipsoid along the Z axis.");
  }

  // ── JointModelUniversal
  template<typename PyClass>
  static void expose(Tag<JointModelUniversal>, PyClass & cl)
  {
    cl.def(
        nb::init<Scalar, Scalar, Scalar, Scalar, Scalar, Scalar>(), "x1"_a, "y1"_a, "z1"_a, "x2"_a,
        "y2"_a, "z2"_a, "Init JointModelUniversal from the components x, y, z of each axis.")
      .def(
        nb::init<const Vector3 &, const Vector3 &>(), "axis1"_a, "axis2"_a,
        "Init JointModelUniversal from two axes with x-y-z components.")
      .def_rw(
        "axis1", &JointModelUniversal::axis1, "First rotation axis of the JointModelUniversal.")
      .def_rw(
        "axis2", &JointModelUniversal::axis2, "Second rotation axis of the JointModelUniversal.");
  }

  // ── JointModelComposite
  template<typename PyClass>
  static void expose(Tag<JointModelComposite>, PyClass & cl)
  {
    cl.def(
        nb::init<std::size_t>(), "size"_a,
        "Init JointModelComposite with a pre-allocated number of joints.")
      .def(
        "__init__",
        [](JointModelComposite * self, const JointModel & jmodel) {
          boost::apply_visitor(
            [self](const auto & jm) { new (self) JointModelComposite(jm); }, jmodel.toVariant());
        },
        "joint_model"_a, "Init JointModelComposite from a joint.")
      .def(
        "__init__",
        [](JointModelComposite * self, const JointModel & jmodel, const SE3 & placement) {
          boost::apply_visitor(
            [self, &placement](const auto & jm) { new (self) JointModelComposite(jm, placement); },
            jmodel.toVariant());
        },
        "joint_model"_a, "joint_placement"_a,
        "Init JointModelComposite from a joint and a placement.")
      .def(
        "__init__",
        [](
          JointModelComposite * self, const JointModel & jmodel, const SE3 & placement,
          const std::string & name) {
          boost::apply_visitor(
            [self, &placement, &name](const auto & jm) {
              new (self) JointModelComposite(jm, placement, name);
            },
            jmodel.toVariant());
        },
        "joint_model"_a, "joint_placement"_a, "name"_a,
        "Init JointModelComposite from a joint, a placement, and a name.")
      .def_ro("joints", &JointModelComposite::joints)
      .def_ro("jointPlacements", &JointModelComposite::jointPlacements)
      .def_ro("njoints", &JointModelComposite::njoints)
      .def(
        "addJoint",
        [](
          JointModelComposite & self, const JointModel & jmodel, const SE3 & placement,
          const std::string & name) -> JointModelComposite & {
          return boost::apply_visitor(
            [&self, &placement, &name](const auto & jm) -> JointModelComposite & {
              return self.addJoint(jm, placement, name);
            },
            jmodel.toVariant());
        },
        "joint_model"_a, "joint_placement"_a = SE3::Identity(), "name"_a = std::string("joint_1"),
        nb::rv_policy::reference, "Add a joint to the composite joint.")
      .def(
        "getJointId", &JointModelComposite::getJointId, "joint_name"_a,
        "Find the index of a joint inside a JointModelComposite by name.");
  }

  // ── JointModelMimic
  template<typename PyClass>
  static void expose(Tag<JointModelMimic>, PyClass & cl)
  {
    cl.def(
        "__init__",
        [](JointModelMimic * self, const JointModel & jmodel, Scalar scaling, Scalar offset) {
          boost::apply_visitor(
            [self, scaling, offset](const auto & jm) {
              new (self) JointModelMimic(jm, scaling, offset);
            },
            jmodel.toVariant());
        },
        "joint_model"_a, "scaling"_a, "offset"_a,
        "Init JointModelMimic from an existing joint with scaling and offset.")
      .def_prop_ro("scaling", [](const JointModelMimic & self) { return self.scaling(); })
      .def_prop_ro("offset", [](const JointModelMimic & self) { return self.offset(); });
  }

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    expose(Tag<typename PyClass::Type>{}, cl);
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
