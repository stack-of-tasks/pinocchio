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

#ifndef __se3_python_force_hpp__
#define __se3_python_force_hpp__

#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

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
      enum { Options = traits<Motion>::Options };
      
      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;
      typedef typename Force::Scalar Scalar;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>("Default constructor"))
        .def(bp::init<Vector3,Vector3>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components (dont mix the order)."))
        .def(bp::init<Vector6>((bp::arg("Vector 6d")),"Init from a vector 6[f,n]"))
        .def(bp::init<Force>((bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",
                      &ForcePythonVisitor::getLinear,
                      &ForcePythonVisitor::setLinear,
                      "Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.")
        .add_property("angular",
                      &ForcePythonVisitor::getAngular,
                      &ForcePythonVisitor::setAngular,
                      "Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.")
        .add_property("vector",
                      &ForcePythonVisitor::getVector,
                      &ForcePythonVisitor::setVector,
                      "Returns the components of *this as a 6d vector.")
        .add_property("np",&ForcePythonVisitor::getVector)
        
        .def("se3Action",&Force::template se3Action<Scalar,Options>,
             bp::args("M"),"Returns the result of the dual action of M on *this.")
        .def("se3ActionInverse",&Force::template se3ActionInverse<Scalar,Options>,
             bp::args("M"),"Returns the result of the dual action of the inverse of M on *this.")
        
        .def("setZero",&ForcePythonVisitor::setZero,
             "Set the linear and angular components of *this to zero.")
        .def("setRandom",&ForcePythonVisitor::setRandom,
             "Set the linear and angular components of *this to random values.")
        
        .def(bp::self + bp::self)
        .def(bp::self += bp::self)
        .def(bp::self - bp::self)
        .def(bp::self -= bp::self)
        .def(-bp::self)
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def(bp::self * Scalar())
        .def(Scalar() * bp::self)
        .def(bp::self / Scalar())
        
        .def("isApprox",(bool (Force::*)(const Force & other, const Scalar & prec) const) &Force::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("Random",&Force::Random,"Returns a random Force.")
        .staticmethod("Random")
        .def("Zero",&Force::Zero,"Returns a zero Force.")
        .staticmethod("Zero")
        
        .def_pickle(Pickle())
        ;
      }
      
      static void expose()
      {
        bp::class_<Force>("Force",
                          "Force vectors, in se3* == F^6.\n\n"
                          "Supported operations ...",
                          bp::init<>())
        .def(ForcePythonVisitor<Force>())
        .def(CopyableVisitor<Force>())
        .def(PrintableVisitor<Force>())
        ;
        
      }
      
    private:
      
      struct Pickle : bp::pickle_suite
      {
        static
        boost::python::tuple
        getinitargs(const Force & f)
        { return bp::make_tuple((Vector3)f.linear(),(Vector3)f.angular()); }
      };
      
      static Vector3 getLinear(const Force & self ) { return self.linear(); }
      static void setLinear(Force & self, const Vector3 & f) { self.linear(f); }
      static Vector3 getAngular(const Force & self) { return self.angular(); }
      static void setAngular(Force & self, const Vector3 & n) { self.angular(n); }
      
      static void setZero(Force & self) { self.setZero(); }
      static void setRandom(Force & self) { self.setRandom(); }
      
      static Vector6 getVector(const Force & self) { return self.toVector(); }
      static void setVector(Force & self, const Vector6 & f) { self = f; }
      
      static bool isApprox(const Force & self, const Force & other)
      { return self.isApprox(other); }

    };
    
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_se3_hpp__

