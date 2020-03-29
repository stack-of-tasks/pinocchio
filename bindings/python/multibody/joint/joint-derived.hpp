//
// Copyright (c) 2015 CNRS
//

#ifndef __pinocchio_python_joint_dense_hpp__
#define __pinocchio_python_joint_dense_hpp__

#include <boost/python.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<class JointModelDerived>
    struct JointModelDerivedPythonVisitor
      : public boost::python::def_visitor< JointModelDerivedPythonVisitor<JointModelDerived> >
    {
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        // All are add_properties cause ReadOnly
        .add_property("id",&JointModelDerivedPythonVisitor::getId)
        .add_property("idx_q",&JointModelDerivedPythonVisitor::getIdx_q)
        .add_property("idx_v",&JointModelDerivedPythonVisitor::getIdx_v)
        .add_property("nq",&JointModelDerivedPythonVisitor::getNq)
        .add_property("nv",&JointModelDerivedPythonVisitor::getNv)
        .def("setIndexes",&JointModelDerived::setIndexes)
        .def("shortname",&JointModelDerived::shortname)
        ;
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

    template<class JointDataDerived>
    struct JointDataDerivedPythonVisitor
      : public boost::python::def_visitor< JointDataDerivedPythonVisitor<JointDataDerived> >
    {
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
          // All are add_properties cause ReadOnly
          .add_property("S",&JointDataDerivedPythonVisitor::getS)
          .add_property("M",&JointDataDerivedPythonVisitor::getM)
          .add_property("v",&JointDataDerivedPythonVisitor::getv)
          .add_property("c",&JointDataDerivedPythonVisitor::getc)
          .add_property("U",&JointDataDerivedPythonVisitor::getU)
          .add_property("Dinv",&JointDataDerivedPythonVisitor::getDinv)
          .add_property("UDinv",&JointDataDerivedPythonVisitor::getUDinv)
          .def("shortname",&JointDataDerived::shortname)
        ;
      }

      static typename JointDataDerived::Constraint_t getS(const JointDataDerived & self )
      { return self.S_accessor(); }
      static typename JointDataDerived::Transformation_t getM(const JointDataDerived & self )
      { return self.M_accessor(); }
      static typename JointDataDerived::Motion_t getv(const JointDataDerived & self )
      { return self.v_accessor(); }
      static typename JointDataDerived::Bias_t getc(const JointDataDerived & self )
      { return self.c_accessor(); }
      static typename JointDataDerived::U_t getU(const JointDataDerived & self )
      { return self.U_accessor(); }
      static typename JointDataDerived::D_t getDinv(const JointDataDerived & self )
      { return self.Dinv_accessor(); }
      static typename JointDataDerived::UD_t getUDinv(const JointDataDerived & self )
      { return self.UDinv_accessor(); }
      
      static void expose()
      {

      }

    }; 
    


  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_joint_dense_hpp__

