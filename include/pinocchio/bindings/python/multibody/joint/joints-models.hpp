//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifndef __pinocchio_python_joints_models_hpp__
#define __pinocchio_python_joints_models_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    // generic expose_joint_model : do nothing special
    template<class T>
    bp::class_<T> & expose_joint_model(bp::class_<T> & cl)
    {
      return cl;
    }

    // specialization for JointModelRevolute
    template<>
    bp::class_<context::JointModelRX> &
    expose_joint_model<context::JointModelRX>(bp::class_<context::JointModelRX> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRX with the X axis ([1, 0, 0]) as rotation axis."))
        .def(
          "getMotionAxis", &context::JointModelRX::getMotionAxis,
          "Rotation axis of the JointModelRX.");
    }

    template<>
    bp::class_<context::JointModelRY> &
    expose_joint_model<context::JointModelRY>(bp::class_<context::JointModelRY> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRY with the Y axis ([0, 1, 0]) as rotation axis."))
        .def(
          "getMotionAxis", &context::JointModelRY::getMotionAxis,
          "Rotation axis of the JointModelRY.");
    }

    template<>
    bp::class_<context::JointModelRZ> &
    expose_joint_model<context::JointModelRZ>(bp::class_<context::JointModelRZ> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRZ with the Z axis ([0, 0, 1]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelRZ::getMotionAxis,
          "Rotation axis of the JointModelRZ.");
    }

    // specialization for JointModelRevoluteUnaligned
    template<>
    bp::class_<context::JointModelRevoluteUnaligned> &
    expose_joint_model<context::JointModelRevoluteUnaligned>(
      bp::class_<context::JointModelRevoluteUnaligned> & cl)
    {
      return cl
        .def(bp::init<context::Scalar, context::Scalar, context::Scalar>(
          bp::args("self", "x", "y", "z"),
          "Init JointModelRevoluteUnaligned from the components x, y, z of the axis"))
        .def(bp::init<const context::Vector3s &>(
          bp::args("self", "axis"),
          "Init JointModelRevoluteUnaligned from an axis with x-y-z components"))
        .def_readwrite(
          "axis", &context::JointModelRevoluteUnaligned::axis,
          "Rotation axis of the JointModelRevoluteUnaligned.");
    }

    // specialization for JointModelRevoluteUnbounded
    template<>
    bp::class_<context::JointModelRUBX> &
    expose_joint_model<context::JointModelRUBX>(bp::class_<context::JointModelRUBX> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRUBX with the X axis ([1, 0, 0]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelRUBX::getMotionAxis,
          "Rotation axis of the JointModelRUBX.");
    }

    template<>
    bp::class_<context::JointModelRUBY> &
    expose_joint_model<context::JointModelRUBY>(bp::class_<context::JointModelRUBY> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRUBY with the Y axis ([0, 1, 0]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelRUBY::getMotionAxis,
          "Rotation axis of the JointModelRUBY.");
    }

    template<>
    bp::class_<context::JointModelRUBZ> &
    expose_joint_model<context::JointModelRUBZ>(bp::class_<context::JointModelRUBZ> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelRUBZ with the Z axis ([0, 0, 1]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelRUBZ::getMotionAxis,
          "Rotation axis of the JointModelRUBZ.");
    }

    // specialization for JointModelPrismatic
    template<>
    bp::class_<context::JointModelPX> &
    expose_joint_model<context::JointModelPX>(bp::class_<context::JointModelPX> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelPX with the X axis ([1, 0, 0]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelPX::getMotionAxis,
          "Rotation axis of the JointModelPX.");
    }

    template<>
    bp::class_<context::JointModelPY> &
    expose_joint_model<context::JointModelPY>(bp::class_<context::JointModelPY> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelPY with the Y axis ([0, 1, 0]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelPY::getMotionAxis,
          "Rotation axis of the JointModelPY.");
    }

    template<>
    bp::class_<context::JointModelPZ> &
    expose_joint_model<context::JointModelPZ>(bp::class_<context::JointModelPZ> & cl)
    {
      return cl
        .def(bp::init<>(
          bp::args("self"), "Init JointModelPZ with the Z axis ([0, 0, 1]) as rotation axis"))
        .def(
          "getMotionAxis", &context::JointModelPZ::getMotionAxis,
          "Rotation axis of the JointModelPZ.");
    }

    // specialization for JointModelPrismaticUnaligned
    template<>
    bp::class_<context::JointModelPrismaticUnaligned> &
    expose_joint_model<context::JointModelPrismaticUnaligned>(
      bp::class_<context::JointModelPrismaticUnaligned> & cl)
    {
      return cl
        .def(bp::init<context::Scalar, context::Scalar, context::Scalar>(
          bp::args("self", "x", "y", "z"),
          "Init JointModelPrismaticUnaligned from the components x, y, z of the axis"))
        .def(bp::init<const context::Vector3s &>(
          bp::args("self", "axis"),
          "Init JointModelPrismaticUnaligned from an axis with x-y-z components"))
        .def_readwrite(
          "axis", &context::JointModelPrismaticUnaligned::axis,
          "Translation axis of the JointModelPrismaticUnaligned.");
    }

    // specialization for JointModelHelicalUnaligned
    template<>
    bp::class_<context::JointModelHelicalUnaligned> &
    expose_joint_model<context::JointModelHelicalUnaligned>(
      bp::class_<context::JointModelHelicalUnaligned> & cl)
    {
      return cl
        .def(bp::init<context::Scalar, context::Scalar, context::Scalar, context::Scalar>(
          bp::args("self", "x", "y", "z", "pitch"),
          "Init JointModelHelicalUnaligned from the components x, y, z of the axis and the pitch"))
        .def(bp::init<const context::Vector3s &, context::Scalar>(
          bp::args("self", "axis", "pitch"),
          "Init JointModelHelicalUnaligned from an axis with x-y-z components and the pitch"))
        .def_readwrite(
          "axis", &context::JointModelHelicalUnaligned::axis,
          "Translation axis of the JointModelHelicalUnaligned.")
        .def_readwrite(
          "pitch", &context::JointModelHelicalUnaligned::m_pitch,
          "Pitch h of the JointModelHelicalUnaligned.");
    }

    // specialization for JointModelHelical
    template<>
    bp::class_<context::JointModelHX> &
    expose_joint_model<context::JointModelHX>(bp::class_<context::JointModelHX> & cl)
    {
      return cl
        .def(bp::init<context::Scalar>(
          bp::args("self", "pitch"),
          "Init JointModelHX with pitch value and the X axis ([1, 0, 0]) as a rotation axis."))
        .def(bp::init<>(
          bp::args("self"),
          "Init JointModelHX with pitch 0.0 and the X axis ([1, 0, 0]) as a rotation axis."))
        .def(
          "getMotionAxis", &context::JointModelHX::getMotionAxis,
          "Rotation axis of the JointModelHX.")
        .def_readwrite("pitch", &context::JointModelHX::m_pitch, "Pitch h of the JointModelHX.");
    }

    template<>
    bp::class_<context::JointModelHY> &
    expose_joint_model<context::JointModelHY>(bp::class_<context::JointModelHY> & cl)
    {
      return cl
        .def(bp::init<context::Scalar>(
          bp::args("self", "pitch"),
          "Init JointModelHY with pitch value and the Y axis ([0, 1, 0]) as a rotation axis."))
        .def(bp::init<>(
          bp::args("self"),
          "Init JointModelHY with pitch 0.0 and the Y axis ([0, 1, 0]) as a rotation axis."))
        .def(
          "getMotionAxis", &context::JointModelHY::getMotionAxis,
          "Rotation axis of the JointModelHY.")
        .def_readwrite("pitch", &context::JointModelHY::m_pitch, "Pitch h of the JointModelHY.");
    }

    template<>
    bp::class_<context::JointModelHZ> &
    expose_joint_model<context::JointModelHZ>(bp::class_<context::JointModelHZ> & cl)
    {
      return cl
        .def(bp::init<context::Scalar>(
          bp::args("self", "pitch"),
          "Init JointModelHZ with pitch value and the Z axis ([0, 0, 1]) as a rotation axis."))
        .def(bp::init<>(
          bp::args("self"),
          "Init JointModelHZ with pitch 0.0 and the Z axis ([0, 0, 1]) as a rotation axis."))
        .def(
          "getMotionAxis", &context::JointModelHZ::getMotionAxis,
          "Rotation axis of the JointModelHZ.")
        .def_readwrite("pitch", &context::JointModelHZ::m_pitch, "Pitch h of the JointModelHZ.");
    }

    // specialization for JointModelUniversal
    template<>
    bp::class_<context::JointModelUniversal> &
    expose_joint_model<context::JointModelUniversal>(bp::class_<context::JointModelUniversal> & cl)
    {
      return cl
        .def(bp::init<
             context::Scalar, context::Scalar, context::Scalar, context::Scalar, context::Scalar,
             context::Scalar>(
          bp::args("self", "x1", "y1", "z1", "x2", "y2", "z2"),
          "Init JointModelUniversal from the components x, y, z of the axes"))
        .def(bp::init<const context::Vector3s &, const context::Vector3s &>(
          bp::args("self", "axis1", "axis2"),
          "Init JointModelUniversal from two axes with x-y-z components"))
        .def_readwrite(
          "axis1", &context::JointModelUniversal::axis1,
          "First rotation axis of the JointModelUniversal.")
        .def_readwrite(
          "axis2", &context::JointModelUniversal::axis2,
          "Second rotation axis of the JointModelUniversal.");
    }

    // specialization for JointModelComposite

    struct JointModelCompositeAddJointVisitor
    : public boost::static_visitor<context::JointModelComposite &>
    {
      context::JointModelComposite & m_joint_composite;
      const context::SE3 & m_joint_placement;

      JointModelCompositeAddJointVisitor(
        context::JointModelComposite & joint_composite, const context::SE3 & joint_placement)
      : m_joint_composite(joint_composite)
      , m_joint_placement(joint_placement)
      {
      }

      template<typename JointModelDerived>
      context::JointModelComposite & operator()(JointModelDerived & jmodel) const
      {
        return m_joint_composite.addJoint(jmodel, m_joint_placement);
      }
    }; // struct JointModelCompositeAddJointVisitor

    static context::JointModelComposite & addJoint_proxy(
      context::JointModelComposite & joint_composite,
      const context::JointModel & jmodel,
      const context::SE3 & joint_placement = context::SE3::Identity())
    {
      return boost::apply_visitor(
        JointModelCompositeAddJointVisitor(joint_composite, joint_placement), jmodel.toVariant());
    }

    struct JointModelCompositeConstructorVisitor
    : public boost::static_visitor<context::JointModelComposite *>
    {
      const context::SE3 & m_joint_placement;

      JointModelCompositeConstructorVisitor(const context::SE3 & joint_placement)
      : m_joint_placement(joint_placement)
      {
      }

      template<typename JointModelDerived>
      context::JointModelComposite * operator()(JointModelDerived & jmodel) const
      {
        return new context::JointModelComposite(jmodel, m_joint_placement);
      }
    }; // struct JointModelCompositeConstructorVisitor

    static context::JointModelComposite * init_proxy1(const context::JointModel & jmodel)
    {
      return boost::apply_visitor(
        JointModelCompositeConstructorVisitor(context::SE3::Identity()), jmodel);
    }

    static context::JointModelComposite *
    init_proxy2(const context::JointModel & jmodel, const context::SE3 & joint_placement)
    {
      return boost::apply_visitor(JointModelCompositeConstructorVisitor(joint_placement), jmodel);
    }

    template<>
    bp::class_<context::JointModelComposite> &
    expose_joint_model<context::JointModelComposite>(bp::class_<context::JointModelComposite> & cl)
    {
      return cl
        .def(bp::init<const size_t>(
          bp::args("self", "size"), "Init JointModelComposite with a defined size"))
        .def(
          "__init__",
          bp::make_constructor(init_proxy1, bp::default_call_policies(), bp::args("joint_model")),
          "Init JointModelComposite from a joint")
        .def(
          "__init__",
          bp::make_constructor(
            init_proxy2, bp::default_call_policies(), bp::args("joint_model", "joint_placement")),
          "Init JointModelComposite from a joint and a placement")
        .add_property("joints", &context::JointModelComposite::joints)
        .add_property("jointPlacements", &context::JointModelComposite::jointPlacements)
        .add_property("njoints", &context::JointModelComposite::njoints)
        .def(
          "addJoint", &addJoint_proxy,
          (bp::arg("self"), bp::arg("joint_model"),
           bp::arg("joint_placement") = context::SE3::Identity()),
          "Add a joint to the vector of joints.", bp::return_internal_reference<>())

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
#endif

        ;
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_models_hpp__
