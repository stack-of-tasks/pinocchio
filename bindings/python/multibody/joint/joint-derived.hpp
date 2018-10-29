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

#ifndef __se3_python_joint_dense_hpp__
#define __se3_python_joint_dense_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<class JointModelDerived>
    struct JointPythonVisitor
      : public boost::python::def_visitor< JointPythonVisitor<JointModelDerived> >
    {
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>())
        // All are add_properties cause ReadOnly
        .add_property("id",&JointPythonVisitor::getId)
        .add_property("idx_q",&JointPythonVisitor::getIdx_q)
        .add_property("idx_v",&JointPythonVisitor::getIdx_v)
        .add_property("nq",&JointPythonVisitor::getNq)
        .add_property("nv",&JointPythonVisitor::getNv)
        .def("setIndexes",&JointModelDerived::setIndexes);

      }

      static JointIndex getId( const JointModelDerived & self ) { return self.id(); }
      static int getIdx_q(const JointModelDerived & self) {return self.idx_q();}
      static int getIdx_v(const JointModelDerived & self) {return self.idx_v();}
      static int getNq(const JointModelDerived & self) {return self.nq();}
      static int getNv(const JointModelDerived & self) {return self.nv();}


      static void expose()
      {

      }

    }; 
    


  }} // namespace se3::python

#endif // ifndef __se3_python_joint_dense_hpp__

