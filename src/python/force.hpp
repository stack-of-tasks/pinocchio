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

#ifndef __se3_python_force_hpp__
#define __se3_python_force_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/spatial/force.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Force>
  {
    typedef se3::ForceTpl<se3::Force::Scalar_t,Eigen::DontAlign> type;
  };
} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Force>
    struct ForcePythonVisitor
      : public boost::python::def_visitor< ForcePythonVisitor<Force> >
    {
      typedef typename eigenpy::UnalignedEquivalent<Force>::type Force_fx;
      typedef typename Force::Matrix3 Matrix3;
      typedef typename Force::Matrix6 Matrix6;
      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;

      typedef typename Force_fx::Matrix3 Matrix3_fx;
      typedef typename Force_fx::Matrix6 Matrix6_fx;
      typedef typename Force_fx::Vector6 Vector6_fx;
      typedef typename Force_fx::Vector3 Vector3_fx;

    public:

      static PyObject* convert(Force const& f)
      {
        Force_fx f_fx (f);
        return boost::python::incref(boost::python::object(f_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Vector3_fx,Vector3_fx>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6_fx>((bp::arg("Vector 6d")),"Init from vector 6 [f,n]"))
        
        .add_property("linear",&ForcePythonVisitor::getLinear,&ForcePythonVisitor::setLinear)
        .add_property("angular",&ForcePythonVisitor::getAngular,&ForcePythonVisitor::setAngular)
        .add_property("vector",&ForcePythonVisitor::getVector,&ForcePythonVisitor::setVector)
        .add_property("np",&ForcePythonVisitor::getVector)
        
        .def("se3Action",&Force_fx::se3Action)
        .def("se3ActionInverse",&Force_fx::se3ActionInverse)
        
        .def("setZero",&ForcePythonVisitor::setZero)
        .def("setRandom",&ForcePythonVisitor::setRandom)
        
        .def("__add__",&ForcePythonVisitor::add)
        .def("__sub__",&ForcePythonVisitor::subst)
        .def(bp::self_ns::str(bp::self_ns::self))
        .def(bp::self_ns::repr(bp::self_ns::self))
        
        .def("Random",&Force_fx::Random)
        .staticmethod("Random")
        .def("Zero",&Force_fx::Zero)
        .staticmethod("Zero")
        ;
      }

      static Vector3_fx getLinear(const Force_fx & self ) { return self.linear(); }
      static void setLinear(Force_fx & self, const Vector3_fx & f) { self.linear(f); }
      static Vector3_fx getAngular(const Force_fx & self) { return self.angular(); }
      static void setAngular(Force_fx & self, const Vector3_fx & n) { self.angular(n); }
      
      static void setZero(Force_fx & self) { self.setZero(); }
      static void setRandom(Force_fx & self) { self.setRandom(); }
      
      static Vector6_fx getVector(const Force_fx & self) { return self.toVector(); }
      static void setVector(Force_fx & self, const Vector6_fx & f) { self = f; }
      
      static Force_fx add(const Force_fx & f1, const Force_fx & f2) { return f1+f2; }
      static Force_fx subst(const Force_fx & f1, const Force_fx & f2) { return f1-f2; }

      static void expose()
      {
	bp::class_<Force_fx>("Force",
			     "Force vectors, in se3* == F^6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(ForcePythonVisitor<Force>())
	;
    
	bp::to_python_converter< Force,ForcePythonVisitor<Force> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

