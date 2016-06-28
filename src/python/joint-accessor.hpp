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

#ifndef __se3_python_joint_accessor_hpp__
#define __se3_python_joint_accessor_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-accessor.hpp"


// namespace eigenpy
// {
//   template<>
//   struct UnalignedEquivalentTypes<se3::JointModelAccessor>
//   {
//     typedef Eigen::Matrix<double, -1, 1, Eigen::DontAlign> MatrixNQd_fx;
//     typedef Eigen::Matrix<double, -1, 1, Eigen::DontAlign> MatrixNVd_fx;
//   };
// } // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct JointModelAccessorPythonVisitor
      : public boost::python::def_visitor< JointModelAccessorPythonVisitor >
    {
      // typedef eigenpy::UnalignedEquivalentTypes<JointModelAccessor>::MatrixNQd_fx MatrixNQd_fx;
      // typedef eigenpy::UnalignedEquivalentTypes<JointModelAccessor>::MatrixNVd_fx MatrixNVd_fx;

    public:

      static PyObject* convert(JointModelAccessor const& jm)
      {
        return boost::python::incref(boost::python::object(jm).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>())
        // All are add_properties cause ReadOnly
        .add_property("id",&JointModelAccessorPythonVisitor::getId)
        .add_property("idx_q",&JointModelAccessorPythonVisitor::getIdx_q)
        .add_property("idx_v",&JointModelAccessorPythonVisitor::getIdx_v)
        .add_property("nq",&JointModelAccessorPythonVisitor::getNq)
        .add_property("nv",&JointModelAccessorPythonVisitor::getNv)

        .def("setIndexes",&JointModelAccessor::setIndexes)
        ;
      }

      static JointIndex getId( const JointModelAccessor & self ) { return self.id(); }
      static int getIdx_q(const JointModelAccessor & self) {return self.idx_q();}
      static int getIdx_v(const JointModelAccessor & self) {return self.idx_v();}
      static int getNq(const JointModelAccessor & self) {return self.nq();}
      static int getNv(const JointModelAccessor & self) {return self.nv();}

      // static void setIndexes(JointModelAccessor & self, JointIndex id,
      //                        int idx_q, int idx_v)
      // { 
      //   self.setIndexes(id, idx_q, idx_v);
      // }

      static void expose()
      {
        bp::class_<JointModelAccessor>("JointModelAccessor",
                                       "Generic Joint Model",
                                       bp::no_init)
        .def(bp::init<se3::JointModelVariant>())
        .def(JointModelAccessorPythonVisitor())
        ;
        
        // bp::to_python_converter< JointModelAccessor,JointModelAccessorPythonVisitor >();
      }

    }; 
    


  }} // namespace se3::python

#endif // ifndef __se3_python_joint_accessor_hpp__

