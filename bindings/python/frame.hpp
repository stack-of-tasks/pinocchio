//
// Copyright (c) 2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_frame_hpp__
#define __se3_python_frame_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/model.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    struct FramePythonVisitor
      : public boost::python::def_visitor< FramePythonVisitor >
    {
      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      typedef Model::Index Index;
      typedef Model::JointIndex JointIndex;
      typedef Model::FrameIndex FrameIndex;

    public:

      static PyObject* convert(Frame const& f)
      {
        return boost::python::incref(boost::python::object(f).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
          .def(bp::init< const std::string&,const JointIndex, const SE3_fx&,FrameType> ((bp::arg("name (string)"),bp::arg("parent (index)"), bp::arg("SE3 placement"), bp::arg("type (FrameType)")),
                "Initialize from name, parent id and placement wrt parent joint."))

          .def_readwrite("name", &Frame::name, "name  of the frame")
          .def_readwrite("parent", &Frame::parent, "id of the parent joint")
          .add_property("placement", 
                        &FramePythonVisitor::getPlacementWrtParentJoint, 
                        &FramePythonVisitor::setPlacementWrtParentJoint, 
                        "placement in the parent joint local frame")
          .def_readwrite("type", &Frame::type, "type of the frame")
          ;
      }


      static SE3_fx getPlacementWrtParentJoint( const Frame & self) { return self.placement; }
      static void setPlacementWrtParentJoint(Frame & self, const SE3_fx & placement) { self.placement = placement; }

      static void expose()
      {
        bp::enum_<FrameType>("FrameType")
            .value("OP_FRAME",OP_FRAME)
            .value("JOINT",JOINT)
            .value("FIXED_JOINT",FIXED_JOINT)
            .value("BODY",BODY)
            .value("SENSOR",SENSOR)
            ;

        bp::class_<Frame>("Frame",
                           "A Plucker coordinate frame related to a parent joint inside a kinematic tree.\n\n",
	                         bp::no_init
                         )
	                       .def(FramePythonVisitor())
	                       ;
    
//        bp::to_python_converter< Frame,FramePythonVisitor >();
        bp::class_< std::vector<Frame> >("StdVec_Frame")
        .def(bp::vector_indexing_suite< std::vector<Frame> >());
      }


    };
    

  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_frame_hpp__
