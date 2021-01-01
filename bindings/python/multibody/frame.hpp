//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_frame_hpp__
#define __pinocchio_python_multibody_frame_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"

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
    struct FramePythonVisitor
    : public boost::python::def_visitor< FramePythonVisitor<Frame> >
    {
      typedef typename Frame::SE3 SE3;
      
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::init<>(bp::arg("self"),"Default constructor"))
        .def(bp::init<const std::string&,const JointIndex, const FrameIndex, const SE3&, FrameType>((bp::arg("self"),bp::arg("name"),bp::arg("parent_joint_id"),bp::args("parent_frame_id"),bp::arg("placement"),bp::arg("type")),
                "Initialize from name, the parent joint id, the parent frame id, the placement wrt parent joint and the type (pinocchio.FrameType)."))
        .def(bp::init<const Frame &>((bp::arg("self"),bp::arg("clone")),"Copy constructor"))

        .def_readwrite("name", &Frame::name, "name of the frame")
        .def_readwrite("parent", &Frame::parent, "id of the parent joint")
        .def_readwrite("previousFrame", &Frame::previousFrame, "id of the previous frame")
        .def_readwrite("placement",
                       &Frame::placement,
                       "placement in the parent joint local frame")
        .def_readwrite("type", &Frame::type, "type of the frame")
        ;
      }
      
      static void expose()
      {
        if(!register_symbolic_link_to_registered_type<FrameType>())
        {
          bp::enum_<FrameType>("FrameType")
          .value("OP_FRAME",OP_FRAME)
          .value("JOINT",JOINT)
          .value("FIXED_JOINT",FIXED_JOINT)
          .value("BODY",BODY)
          .value("SENSOR",SENSOR)
          .export_values()
          ;
        }

        bp::class_<Frame>("Frame",
                          "A Plucker coordinate frame related to a parent joint inside a kinematic tree.\n",
                          bp::no_init
                         )
        .def(FramePythonVisitor())
        .def(CastVisitor<Frame>())
        .def(ExposeConstructorByCastVisitor<Frame,::pinocchio::Frame>())
        .def(CopyableVisitor<Frame>())
        .def(PrintableVisitor<Frame>())
        ;
      }

    };
    

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_multibody_frame_hpp__
