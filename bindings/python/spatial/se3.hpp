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

#ifndef __se3_python_se3_hpp__
#define __se3_python_se3_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::SE3)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename SE3>
    struct SE3PythonVisitor
      : public boost::python::def_visitor< SE3PythonVisitor<SE3> >
    {
      typedef typename SE3::Scalar Scalar;
      typedef typename SE3::Matrix3 Matrix3;
      typedef typename SE3::Vector3 Vector3;
      
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Matrix3,Vector3>
             ((bp::arg("Rotation"),bp::arg("translation")),
              "Initialize from rotation and translation."))
        .def(bp::init<int>((bp::arg("trivial arg (should be 1)")),"Init to identity."))
        .def(bp::init<SE3>((bp::arg("other")), "Copy constructor."))

        .add_property("rotation",
                      &getRotation,
                      (void (SE3::*)(const Matrix3 &)) &SE3::rotation,
                      "The rotation part of the transformation."
                      )
        .add_property("translation",
                      &getTranslation,
                      (void (SE3::*)(const Vector3 &)) &SE3::translation,
                      "The translation part of the transformation."
                      )
        
        .add_property("homogeneous",&SE3::toHomogeneousMatrix,"Returns the homegeneous matrix of *this.")
        .add_property("action",&SE3::toActionMatrix,"Returns the action matrix of *this.")
        
        .def("setIdentity",&SE3PythonVisitor::setIdentity,"Set *this to the identity placement.")
        .def("setRandom",&SE3PythonVisitor::setRandom,"Set *this to a random placement.")

        .def("inverse", &SE3::inverse)
        .def("act", (Vector3 (SE3::*)(const Vector3 &) const) &SE3::act,
             bp::args("point"),
             "Returns a point which is the result of the entry point transforms by *this.")
        .def("actInv", (Vector3 (SE3::*)(const Vector3 &) const) &SE3::actInv,
             bp::args("point"),
             "Returns a point which is the result of the entry point by the inverse of *this.")
        .def("act", (SE3 (SE3::*)(const SE3 & other) const) &SE3::act,
             bp::args("M"), "Returns the result of *this * M.")
        .def("actInv", (SE3 (SE3::*)(const SE3 & other) const) &SE3::actInv,
             bp::args("M"), "Returns the result of the inverse of *this times M.")
        
        .def("isApprox",(bool (SE3::*)(const SE3 & other, const Scalar & prec)) &SE3::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",(bool (SE3::*)(const SE3 & other)) &SE3::isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("isIdentity",(bool (SE3::*)(const Scalar & prec)) &SE3::isIdentity,bp::args("prec"),"Returns true if *this is approximately equal to the identity placement, within the precision given by prec.")
        .def("isIdentity",(bool (SE3::*)(void)) &SE3::isIdentity,"Returns true if *this is approximately equal to the identity placement.")
        
        .def("__invert__",&SE3::inverse,"Returns the inverse of *this.")
        .def(bp::self * bp::self)
        .add_property("np",&SE3::toHomogeneousMatrix)
        
        .def("Identity",&SE3::Identity,"Returns the identity transformation.")
        .staticmethod("Identity")
        .def("Random",&SE3::Random,"Returns a random transformation.")
        .staticmethod("Random")
        ;
      }
      
      static void expose()
      {
        bp::class_<SE3>("SE3",
                        "SE3 transformation composed defined by its translation and its rotation",
                        bp::init<>())
        .def(SE3PythonVisitor<SE3>())
        .def(CopyableVisitor<SE3>())
        .def(PrintableVisitor<SE3>())
        ;
        
      }
    private:
      
      static Vector3 getTranslation(const SE3 & self) { return self.translation(); }
      static Matrix3 getRotation(const SE3 & self) { return self.rotation(); }
      
      static void setIdentity(SE3 & self) { self.setIdentity(); }
      static void setRandom(SE3 & self) { self.setRandom(); }
    };
    


  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_se3_hpp__

