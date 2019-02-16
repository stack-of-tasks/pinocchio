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
          //.add_property("id",&JointDataDerivedPythonVisitor::getId)
          //.add_property("idx_q",&JointDataDerivedPythonVisitor::getIdx_q)
          //.add_property("idx_v",&JointDataDerivedPythonVisitor::getIdx_v)
          //.add_property("nq",&JointDataDerivedPythonVisitor::getNq)
          //.add_property("nv",&JointDataDerivedPythonVisitor::getNv)
          //.def("setIndexes",&JointDataDerived::setIndexes)
          .def("shortname",&JointDataDerived::shortname)
        ;
      }

      //static JointIndex getId( const JointDataDerived & self ) { return self.id(); }
      //static int getIdx_q(const JointDataDerived & self) {return self.idx_q();}
      //static int getIdx_v(const JointDataDerived & self) {return self.idx_v();}
      //static int getNq(const JointDataDerived & self) {return self.nq();}
      //static int getNv(const JointDataDerived & self) {return self.nv();}


      static void expose()
      {

      }

    }; 
    


  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_joint_dense_hpp__

