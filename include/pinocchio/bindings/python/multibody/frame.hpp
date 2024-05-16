//
// Copyright (c) 2016-2022 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_frame_hpp__
#define __pinocchio_python_multibody_frame_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"

#include "pinocchio/bindings/python/utils/deprecation.hpp"
#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Frame>
    struct FramePythonVisitor : public boost::python::def_visitor<FramePythonVisitor<Frame>>
    {
      typedef typename Frame::SE3 SE3;
      typedef typename Frame::Inertia Inertia;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<>(bp::arg("self"), "Default constructor"))
          .def(bp::init<const Frame &>(bp::args("self", "other"), "Copy constructor"))
          .def(bp::init<
               const std::string &, const JointIndex, const SE3 &, FrameType,
               bp::optional<const Inertia &>>(
            (bp::arg("name"), bp::arg("parent_joint"), bp::arg("placement"), bp::arg("type"),
             bp::arg("inertia")),
            "Initialize from a given name, type, parent frame index and placement wrt parent joint "
            "and an spatial inertia object."))
          .def(bp::init<
               const std::string &, const JointIndex, const FrameIndex, const SE3 &, FrameType,
               bp::optional<const Inertia &>>(
            (bp::arg("name"), bp::arg("parent_joint"), bp::args("parent_frame"),
             bp::arg("placement"), bp::arg("type"), bp::arg("inertia")),
            "Initialize from a given name, type, parent joint index, parent frame index and "
            "placement wrt parent joint and an spatial inertia object."))
          .def(bp::init<const Frame &>((bp::arg("self"), bp::arg("clone")), "Copy constructor"))

          .def_readwrite("name", &Frame::name, "name of the frame")
          .def_readwrite("parentJoint", &Frame::parentJoint, "Index of the parent joint")
          .def_readwrite("parentFrame", &Frame::parentFrame, "Index of the parent frame")
          .add_property(
            "parent",
            bp::make_function(
              +[](const Frame & self) { return self.parentJoint; },
              deprecated_member<>("Deprecated member. Use Frame.parentJoint instead.")),
            bp::make_function(
              +[](Frame & self, const JointIndex index) { self.parentJoint = index; },
              deprecated_member<>("Deprecated member. Use Frame.parentJoint instead.")),
            "See parentJoint property.")
          .add_property(
            "previousFrame",
            bp::make_function(
              +[](const Frame & self) { return self.parentFrame; },
              deprecated_member<>("Deprecated member. Use Frame.parentFrame instead.")),
            bp::make_function(
              +[](Frame & self, const FrameIndex index) { self.parentFrame = index; },
              deprecated_member<>("Deprecated member. Use Frame.parentFrame instead.")),
            "See parentFrame property.")
          .def_readwrite(
            "placement", &Frame::placement, "placement in the parent joint local frame")
          .def_readwrite("type", &Frame::type, "type of the frame")
          .def_readwrite("inertia", &Frame::inertia, "Inertia information attached to the frame.")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
          ;
      }

      static void expose()
      {
        if (!register_symbolic_link_to_registered_type<FrameType>())
        {
          bp::enum_<FrameType>("FrameType")
            .value("OP_FRAME", OP_FRAME)
            .value("JOINT", JOINT)
            .value("FIXED_JOINT", FIXED_JOINT)
            .value("BODY", BODY)
            .value("SENSOR", SENSOR)
            .export_values();
        }

        bp::class_<Frame>(
          "Frame",
          "A Plucker coordinate frame related to a parent joint inside a kinematic tree.\n",
          bp::no_init)
          .def(FramePythonVisitor())
          .def(CastVisitor<Frame>())
          .def(ExposeConstructorByCastVisitor<Frame, ::pinocchio::Frame>())
          .def(CopyableVisitor<Frame>())
          .def(PrintableVisitor<Frame>())
          .def_pickle(Pickle());
      }

    private:
      struct Pickle : bp::pickle_suite
      {
        static bp::tuple getinitargs(const Frame &)
        {
          return bp::make_tuple();
        }

        static bp::tuple getstate(const Frame & f)
        {
          return bp::make_tuple(
            f.name, f.parentJoint, f.parentFrame, f.placement, (int)f.type, f.inertia);
        }

        static void setstate(Frame & f, bp::tuple tup)
        {
          f.name = bp::extract<std::string>(tup[0]);
          f.parentJoint = bp::extract<JointIndex>(tup[1]);
          f.parentFrame = bp::extract<JointIndex>(tup[2]);
          f.placement = bp::extract<SE3 &>(tup[3]);
          f.type = (FrameType)(int)bp::extract<int>(tup[4]);
          if (bp::len(tup) > 5)
            f.inertia = bp::extract<Inertia &>(tup[5]);
        }

        static bool getstate_manages_dict()
        {
          return true;
        }
      };
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_multibody_frame_hpp__
