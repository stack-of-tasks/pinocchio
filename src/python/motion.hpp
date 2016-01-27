//
// Copyright (c) 2015-2016 CNRS
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
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Motion>
  {
    typedef se3::MotionTpl<se3::Motion::Scalar_t,Eigen::DontAlign> type;
  };
} // namespace eigenpy

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

      typedef typename eigenpy::UnalignedEquivalent<Motion>::type Motion_fx;
      typedef typename Motion_fx::Force Force_fx;
      typedef typename Motion_fx::Matrix3 Matrix3_fx;
      typedef typename Motion_fx::Matrix6 Matrix6_fx;
      typedef typename Motion_fx::Vector6 Vector6_fx;
      typedef typename Motion_fx::Vector3 Vector3_fx;

    public:

      static PyObject* convert(Motion const& m)
      {
	Motion_fx m_fx (m);
	return boost::python::incref(boost::python::object(m_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Vector3_fx,Vector3_fx>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6_fx>((bp::arg("Vector 6d")),"Init from vector 6 [v,w]"))
        
        .add_property("linear",&MotionPythonVisitor::getLinear,&MotionPythonVisitor::setLinear)
        .add_property("angular",&MotionPythonVisitor::getAngular,&MotionPythonVisitor::setAngular)
        .add_property("vector",&MotionPythonVisitor::getVector,&MotionPythonVisitor::setVector)
        .add_property("np",&MotionPythonVisitor::getVector)
        
        .def("se3Action",&Motion_fx::se3Action)
        .def("se3ActionInverse",&Motion_fx::se3ActionInverse)
        
        .def("setZero",&MotionPythonVisitor::setZero)
        .def("setRandom",&MotionPythonVisitor::setRandom)
        
        .def("cross_motion",&MotionPythonVisitor::cross_motion)
        .def("cross_force",&MotionPythonVisitor::cross_force)
        
        .def("__add__",&MotionPythonVisitor::add)
        .def("__sub__",&MotionPythonVisitor::subst)
        .def(bp::self_ns::str(bp::self_ns::self))
        
        .def("Random",&Motion_fx::Random)
        .staticmethod("Random")
        .def("Zero",&Motion_fx::Zero)
        .staticmethod("Zero")
        ;
      }

      static Vector3_fx getLinear(const Motion_fx & self) { return self.linear(); }
      static void setLinear (Motion_fx & self, const Vector3_fx & v) { self.linear(v); }
      static Vector3_fx getAngular(const Motion_fx & self) { return self.angular(); }
      static void setAngular(Motion_fx & self, const Vector3_fx & w) { self.angular(w); }
      
      static Vector6_fx getVector(const Motion_fx & self) { return self.toVector(); }
      static void setVector(Motion_fx & self, const Vector6_fx & v) { self = v; }
      
      static void setZero(Motion_fx & self) { self.setZero(); }
      static void setRandom(Motion_fx & self) { self.setRandom(); }
      
      static Motion_fx add( const Motion_fx& m1,const Motion_fx& m2 ) { return m1+m2; }     
      static Motion_fx subst( const Motion_fx& m1,const Motion_fx& m2 ) { return m1-m2; }     
      static Motion_fx cross_motion( const Motion_fx& m1,const Motion_fx& m2 ) { return m1.cross(m2); }
      static Force_fx cross_force( const Motion_fx& m,const Force_fx& f ) { return m.cross(f); }

      static void expose()
      {
	bp::class_<Motion_fx>("Motion",
			     "Motion vectors, in se3* == F^6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(MotionPythonVisitor<Motion>())
	;
    
	bp::to_python_converter< Motion,MotionPythonVisitor<Motion> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

