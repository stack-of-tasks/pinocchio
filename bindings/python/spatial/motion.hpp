//
// Copyright (c) 2015-2018 CNRS
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
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
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
      enum { Options = traits<Motion>::Options };
      
      typedef typename Motion::Scalar Scalar;
      typedef ForceTpl<Scalar,traits<Motion>::Options> Force;
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
        
        .def("se3Action",&Motion::template se3Action<Scalar,Options>,
             bp::args("M"),"Returns the result of the action of M on *this.")
        .def("se3ActionInverse",&Motion::template se3ActionInverse<Scalar,Options>,
             bp::args("M"),"Returns the result of the action of the inverse of M on *this.")
        
        .add_property("action",&Motion::toActionMatrix,"Returns the action matrix of *this (acting on Motion).")
        .add_property("dualAction",&Motion::toDualActionMatrix,"Returns the dual action matrix of *this (acting on Force).")
        
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
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def(bp::self * Scalar())
        .def(Scalar() * bp::self)
        .def(bp::self / Scalar())
        
        .def("isApprox",(bool (Motion::*)(const Motion & other, const Scalar & prec) const) &Motion::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("Random",&Motion::Random,"Returns a random Motion.")
        .staticmethod("Random")
        .def("Zero",&Motion::Zero,"Returns a zero Motion.")
        .staticmethod("Zero")
        
        .def_pickle(Pickle())
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
      
      struct Pickle : bp::pickle_suite
      {
        static
        boost::python::tuple
        getinitargs(const Motion & m)
        { return bp::make_tuple((Vector3)m.linear(),(Vector3)m.angular()); }
      };
      
      static Vector3 getLinear(const Motion & self) { return self.linear(); }
      static void setLinear (Motion & self, const Vector3 & v) { self.linear(v); }
      static Vector3 getAngular(const Motion & self) { return self.angular(); }
      static void setAngular(Motion & self, const Vector3 & w) { self.angular(w); }
      
      static Vector6 getVector(const Motion & self) { return self.toVector(); }
      static void setVector(Motion & self, const Vector6 & v) { self = v; }
      
      static void setZero(Motion & self) { self.setZero(); }
      static void setRandom(Motion & self) { self.setRandom(); }
      
      static bool isApprox(const Motion & self, const Motion & other)
      { return self.isApprox(other); }

    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

