//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_joint_joint_hpp__
#define __pinocchio_python_multibody_joint_joint_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct JointModelPythonVisitor
    : public boost::python::def_visitor< JointModelPythonVisitor >
    {
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>(bp::arg("self"),"Default constructor"))
        // All are add_properties cause ReadOnly
        .add_property("id",&JointModelPythonVisitor::getId)
        .add_property("idx_q",&JointModelPythonVisitor::getIdx_q)
        .add_property("idx_v",&JointModelPythonVisitor::getIdx_v)
        .add_property("nq",&JointModelPythonVisitor::getNq)
        .add_property("nv",&JointModelPythonVisitor::getNv)
        .def("setIndexes",
             &JointModel::setIndexes,
             bp::args("self","joint_id","idx_q","idx_v"))
        .def("shortname",
             &JointModel::shortname,
             bp::arg("self"))
        ;
      }
      
      static void expose()
      {
        bp::class_<JointModel>("JointModel",
                               "Generic Joint Model",
                               bp::no_init)
        .def(bp::init<pinocchio::JointModelVariant>())
        .def(JointModelPythonVisitor())
        .def(PrintableVisitor<JointModel>())
        ;
      }
      
    private:
      
      static JointIndex getId( const JointModel & self ) { return self.id(); }
      static int getIdx_q(const JointModel & self) { return self.idx_q(); }
      static int getIdx_v(const JointModel & self) { return self.idx_v(); }
      static int getNq(const JointModel & self) { return self.nq(); }
      static int getNv(const JointModel & self) { return self.nv(); }

    }; 
    
}} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_joint_joint_hpp__
