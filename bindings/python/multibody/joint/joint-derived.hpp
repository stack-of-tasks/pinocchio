//
// Copyright (c) 2015 CNRS
//

#ifndef __pinocchio_python_joint_dense_hpp__
#define __pinocchio_python_joint_dense_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace pinocchio
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
    


  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_joint_dense_hpp__

