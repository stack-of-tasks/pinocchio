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

#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

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
      typedef typename Motion::Vector6 Vector6;
      typedef typename Motion::Vector3 Vector3;

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>("Default constructor"))
        .def(bp::init<Vector3,Vector3>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6>((bp::arg("Vector6")),"Init from a vector 6 [v,w]"))
        .def(bp::init<Motion>((bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",
                      &MotionPythonVisitor::getLinear,
                      &MotionPythonVisitor::setLinear,
                      "Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.")
        .add_property("angular",
                      &MotionPythonVisitor::getAngular,
                      &MotionPythonVisitor::setAngular,
                      "Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.")
        .add_property("vector",
                      &MotionPythonVisitor::getVector,
                      &MotionPythonVisitor::setVector,
                      "Returns the components of *this as a 6d vector.")
        .add_property("np",&MotionPythonVisitor::getVector)
        
        .def("se3Action",&Motion::se3Action,
             bp::args("M"),"Returns the result of the action of M on *this.")
        .def("se3ActionInverse",&Motion::se3ActionInverse,
             bp::args("M"),"Returns the result of the action of the inverse of M on *this.")
        
        .def("setZero",&MotionPythonVisitor::setZero,
             "Set the linear and angular components of *this to zero.")
        .def("setRandom",&MotionPythonVisitor::setRandom,
             "Set the linear and angular components of *this to random values.")
        
        .def("cross",(Motion (Motion::*)(const Motion &) const) &Motion::cross,
             bp::args("m"),"Action of *this onto another Motion m. Returns ¨*this x m.")
        .def("cross",(Force (Motion::*)(const Force &) const) &Motion::cross,
             bp::args("f"),"Dual action of *this onto a Force f. Returns *this x* f.")
        
        .def(bp::self + bp::self)
        .def(bp::self += bp::self)
        .def(bp::self - bp::self)
        .def(bp::self -= bp::self)
        .def(-bp::self)
        .def(bp::self ^ bp::self)
        .def(bp::self ^ Force())
        
        .def("Random",&Motion::Random,"Returns a random Motion.")
        .staticmethod("Random")
        .def("Zero",&Motion::Zero,"Returns a zero Motion.")
        .staticmethod("Zero")
        ;
      }

      static void expose()
      {
        bp::class_<Motion>("Motion",
                              "Motion vectors, in se3 == M^6.\n\n"
                              "Supported operations ...",
                              bp::init<>())
        .def(MotionPythonVisitor<Motion>())
        .def(CopyableVisitor<Motion>())
        .def(PrintableVisitor<Motion>())
        ;
      }
      
    private:
      static Vector3 getLinear(const Motion & self) { return self.linear(); }
      static void setLinear (Motion & self, const Vector3 & v) { self.linear(v); }
      static Vector3 getAngular(const Motion & self) { return self.angular(); }
      static void setAngular(Motion & self, const Vector3 & w) { self.angular(w); }
      
      static Vector6 getVector(const Motion & self) { return self.toVector(); }
      static void setVector(Motion & self, const Vector6 & v) { self = v; }
      
      static void setZero(Motion & self) { self.setZero(); }
      static void setRandom(Motion & self) { self.setRandom(); }

    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

