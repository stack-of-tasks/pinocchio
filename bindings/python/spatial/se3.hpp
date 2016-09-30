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

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

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
      typedef typename SE3::Matrix6 Matrix6;
      typedef typename SE3::Matrix4 Matrix4;
      typedef typename SE3::Vector6 Vector6;
      typedef typename SE3::Vector3 Vector3;
      
    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Matrix3,Vector3>
             ((bp::arg("Rotation"),bp::arg("translation")),
              "Initialize from rotation and translation."))
        .def(bp::init<int>((bp::arg("trivial arg (should be 1)")),"Init to identity."))
        .def(bp::init<SE3> ((bp::arg("other")), "Copy constructor."))

	  .add_property("rotation",&SE3PythonVisitor::getRotation,&SE3PythonVisitor::setRotation)
	  .add_property("translation",&SE3PythonVisitor::getTranslation,&SE3PythonVisitor::setTranslation)
	  .add_property("homogeneous",&SE3::toHomogeneousMatrix)
	  .add_property("action",&SE3::toActionMatrix)
        
    .def("setIdentity",&SE3PythonVisitor::setIdentity)
    .def("setRandom",&SE3PythonVisitor::setRandom)

	  .def("inverse", &SE3::inverse)
	  .def("act_point", &SE3PythonVisitor::act_point)
	  .def("actInv_point", &SE3PythonVisitor::actInv_point)
	  .def("act_se3", &SE3PythonVisitor::act_se3)
	  .def("actInv_se3", &SE3PythonVisitor::actInv_se3)
        
        .def("isApprox",(bool (SE3::*)(const SE3 & other, const Scalar & prec)) &SE3::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",(bool (SE3::*)(const SE3 & other)) &SE3::isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("isIdentity",(bool (SE3::*)(const Scalar & prec)) &SE3::isIdentity,bp::args("prec"),"Returns true if *this is approximately equal to the identity placement, within the precision given by prec.")
        .def("isIdentity",(bool (SE3::*)(void)) &SE3::isIdentity,"Returns true if *this is approximately equal to the identity placement.")
	  
	  .def("__str__",&SE3PythonVisitor::toString)
	  .def("__invert__",&SE3::inverse)
	  .def(bp::self * bp::self)
	  .add_property("np",&SE3::toActionMatrix)

	  .def("Identity",&SE3::Identity)
	  .staticmethod("Identity")
	  .def("Random",&SE3::Random)
	  .staticmethod("Random")
	  ;
	  }

      static Matrix3 getRotation( const SE3 & self ) { return self.rotation(); }
      static void setRotation( SE3 & self, const Matrix3 & R ) { self.rotation(R); }
      static Vector3 getTranslation( const SE3 & self ) { return self.translation(); }
      static void setTranslation( SE3 & self, const Vector3 & p ) { self.translation(p); }
      
      static void setIdentity(SE3 & self) { self.setIdentity(); }
      static void setRandom(SE3 & self) { self.setRandom(); }

      static Vector3 act_point( const SE3 & self, const Vector3 & pt ) { return self.act(pt); }
      static Vector3 actInv_point( const SE3 & self, const Vector3 & pt ) { return self.actInv(pt); }
      static SE3 act_se3( const SE3 & self, const SE3 & x ) { return self.act(x); }
      static SE3 actInv_se3( const SE3 & self, const SE3 & x ) { return self.actInv(x); }
      static std::string toString(const SE3& m) 
      {	  std::ostringstream s; s << m; return s.str();       }

      static void expose()
      {
	bp::class_<SE3>("SE3",
			   "SE3 transformations, composing rotation and translation.\n\n"
			   "Supported operations ...",
			   bp::init<>())
	  .def(SE3PythonVisitor<SE3>())
        .def(CopyableVisitor<SE3>())
	;
    
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

