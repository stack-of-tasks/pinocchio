//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_python_motion_hpp__
#define __se3_python_motion_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::Motion)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Motion>
    struct MotionPythonVisitor
      : public boost::python::def_visitor< MotionPythonVisitor<Motion> >
    {
      typedef typename Motion::Force Force;
      typedef typename Motion::Matrix3 Matrix3;
      typedef typename Motion::Matrix6 Matrix6;
      typedef typename Motion::Vector6 Vector6;
      typedef typename Motion::Vector3 Vector3;

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Vector3,Vector3>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6>((bp::arg("Vector 6d")),"Init from vector 6 [v,w]"))
        .def(bp::init<Motion>((bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",&MotionPythonVisitor::getLinear,&MotionPythonVisitor::setLinear)
        .add_property("angular",&MotionPythonVisitor::getAngular,&MotionPythonVisitor::setAngular)
        .add_property("vector",&MotionPythonVisitor::getVector,&MotionPythonVisitor::setVector)
        .add_property("np",&MotionPythonVisitor::getVector)
        
        .def("se3Action",&Motion::se3Action)
        .def("se3ActionInverse",&Motion::se3ActionInverse)
        
        .def("setZero",&MotionPythonVisitor::setZero)
        .def("setRandom",&MotionPythonVisitor::setRandom)
        
        .def("cross_motion",&MotionPythonVisitor::cross_motion)
        .def("cross_force",&MotionPythonVisitor::cross_force)
        
        .def("__add__",&MotionPythonVisitor::add)
        .def("__sub__",&MotionPythonVisitor::subst)
        .def("__neg__",&MotionPythonVisitor::neg)
        .def(bp::self_ns::str(bp::self_ns::self))
        
        .def("Random",&Motion::Random)
        .staticmethod("Random")
        .def("Zero",&Motion::Zero)
        .staticmethod("Zero")
        ;
      }

      static Vector3 getLinear(const Motion & self) { return self.linear(); }
      static void setLinear (Motion & self, const Vector3 & v) { self.linear(v); }
      static Vector3 getAngular(const Motion & self) { return self.angular(); }
      static void setAngular(Motion & self, const Vector3 & w) { self.angular(w); }
      
      static Vector6 getVector(const Motion & self) { return self.toVector(); }
      static void setVector(Motion & self, const Vector6 & v) { self = v; }
      
      static void setZero(Motion & self) { self.setZero(); }
      static void setRandom(Motion & self) { self.setRandom(); }
      
      static Motion add( const Motion& m1,const Motion& m2 ) { return m1+m2; }     
      static Motion subst( const Motion& m1,const Motion& m2 ) { return m1-m2; }     
      static Motion neg(const Motion & m1) { return -m1; }
      static Motion cross_motion( const Motion& m1,const Motion& m2 ) { return m1.cross(m2); }
      static Force cross_force( const Motion& m,const Force& f ) { return m.cross(f); }

      static void expose()
      {
        bp::class_<Motion>("Motion",
                              "Motion vectors, in se3 == M^6.\n\n"
                              "Supported operations ...",
                              bp::init<>())
        .def(MotionPythonVisitor<Motion>())
        .def(CopyableVisitor<Motion>())
        ;
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

