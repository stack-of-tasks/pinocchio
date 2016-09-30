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

#ifndef __se3_python_force_hpp__
#define __se3_python_force_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::Force)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Force>
    struct ForcePythonVisitor
      : public boost::python::def_visitor< ForcePythonVisitor<Force> >
    {
      typedef typename Force::Matrix3 Matrix3;
      typedef typename Force::Matrix6 Matrix6;
      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Vector3,Vector3>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6>((bp::arg("Vector 6d")),"Init from vector 6 [f,n]"))
        .def(bp::init<Force>((bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",&ForcePythonVisitor::getLinear,&ForcePythonVisitor::setLinear)
        .add_property("angular",&ForcePythonVisitor::getAngular,&ForcePythonVisitor::setAngular)
        .add_property("vector",&ForcePythonVisitor::getVector,&ForcePythonVisitor::setVector)
        .add_property("np",&ForcePythonVisitor::getVector)
        
        .def("se3Action",&Force::se3Action)
        .def("se3ActionInverse",&Force::se3ActionInverse)
        
        .def("setZero",&ForcePythonVisitor::setZero)
        .def("setRandom",&ForcePythonVisitor::setRandom)
        
        .def("__add__",&ForcePythonVisitor::add)
        .def("__sub__",&ForcePythonVisitor::subst)
        .def("__neg__",&ForcePythonVisitor::neg)
        .def(bp::self_ns::str(bp::self_ns::self))
        .def(bp::self_ns::repr(bp::self_ns::self))
        
        .def("Random",&Force::Random)
        .staticmethod("Random")
        .def("Zero",&Force::Zero)
        .staticmethod("Zero")
        ;
      }

      static Vector3 getLinear(const Force & self ) { return self.linear(); }
      static void setLinear(Force & self, const Vector3 & f) { self.linear(f); }
      static Vector3 getAngular(const Force & self) { return self.angular(); }
      static void setAngular(Force & self, const Vector3 & n) { self.angular(n); }
      
      static void setZero(Force & self) { self.setZero(); }
      static void setRandom(Force & self) { self.setRandom(); }
      
      static Vector6 getVector(const Force & self) { return self.toVector(); }
      static void setVector(Force & self, const Vector6 & f) { self = f; }
      
      static Force add(const Force & f1, const Force & f2) { return f1+f2; }
      static Force subst(const Force & f1, const Force & f2) { return f1-f2; }
      static Force neg(const Force & f1) { return -f1; }

      static void expose()
      {
	bp::class_<Force>("Force",
			     "Force vectors, in se3* == F^6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(ForcePythonVisitor<Force>())
        .def(CopyableVisitor<Force>())
	;
    
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

