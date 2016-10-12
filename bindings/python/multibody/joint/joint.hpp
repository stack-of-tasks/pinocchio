//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_joint_hpp__
#define __se3_python_joint_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/multibody/joint/joint.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct JointModelPythonVisitor
      : public boost::python::def_visitor< JointModelPythonVisitor >
    {

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>())
        // All are add_properties cause ReadOnly
        .add_property("id",&JointModelPythonVisitor::getId)
        .add_property("idx_q",&JointModelPythonVisitor::getIdx_q)
        .add_property("idx_v",&JointModelPythonVisitor::getIdx_v)
        .add_property("nq",&JointModelPythonVisitor::getNq)
        .add_property("nv",&JointModelPythonVisitor::getNv)

        .def("setIndexes",&JointModel::setIndexes)
        ;
      }

      static JointIndex getId( const JointModel & self ) { return self.id(); }
      static int getIdx_q(const JointModel & self) {return self.idx_q();}
      static int getIdx_v(const JointModel & self) {return self.idx_v();}
      static int getNq(const JointModel & self) {return self.nq();}
      static int getNv(const JointModel & self) {return self.nv();}

      static void expose()
      {
        bp::class_<JointModel>("JointModel",
                               "Generic Joint Model",
                               bp::no_init)
        .def(bp::init<se3::JointModelVariant>())
        .def(JointModelPythonVisitor())
        ;
      }

    }; 
    
}} // namespace se3::python

#endif // ifndef __se3_python_joint_hpp__
