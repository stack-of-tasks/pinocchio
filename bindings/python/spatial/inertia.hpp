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

#ifndef __se3_python_inertia_hpp__
#define __se3_python_inertia_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Inertia>
    struct InertiaPythonVisitor
      : public boost::python::def_visitor< InertiaPythonVisitor<Inertia> >
    {
      typedef typename Inertia::Matrix3 Matrix3;
      typedef typename Inertia::Matrix6 Matrix6;
      typedef typename Inertia::Vector6 Vector6;
      typedef typename Inertia::Vector3 Vector3;
      typedef typename Inertia::Scalar Scalar;

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def("__init__",
             bp::make_constructor(&InertiaPythonVisitor::makeFromMCI,
                                  bp::default_call_policies(),
                                  (bp::arg("mass"),bp::arg("lever"),bp::arg("inertia"))),
             "Initialize from mass, lever and 3d inertia.")
        .def(bp::init<Inertia>((bp::arg("other")),"Copy constructor."))
        
        .add_property("mass", &InertiaPythonVisitor::getMass, &InertiaPythonVisitor::setMass)
        .add_property("lever", &InertiaPythonVisitor::getLever, &InertiaPythonVisitor::setLever)
        .add_property("inertia", &InertiaPythonVisitor::getInertia, &InertiaPythonVisitor::setInertia)
        
        .def("matrix",&Inertia::matrix)
        .def("se3Action",&Inertia::se3Action)
        .def("se3ActionInverse",&Inertia::se3ActionInverse)
        
        .def("setIdentity",&Inertia::setIdentity)
        .def("setZero",&Inertia::setZero)
        .def("setRandom",&Inertia::setRandom)
        
        .def("__str__",&InertiaPythonVisitor::toString)
        .def( bp::self + bp::self)
        .def( bp::self * bp::other<Motion>() )
        .add_property("np",&Inertia::matrix)
        
        .def("Identity",&Inertia::Identity)
        .staticmethod("Identity")
        .def("Zero",&Inertia::Zero)
        .staticmethod("Zero")
        .def("Random",&Inertia::Random)
        .staticmethod("Random")
        .def("FromEllipsoid", &Inertia::FromEllipsoid,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("length_x"), bp::arg("length_y"), bp::arg("length_z")))
        .staticmethod("FromEllipsoid")
        .def("FromCylinder", &Inertia::FromCylinder,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("radius"), bp::arg("length")))
        .staticmethod("FromCylinder")
        .def("FromBox", &Inertia::FromBox,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("length_x"), bp::arg("length_y"), bp::arg("length_z")))
        .staticmethod("FromBox")
        ;
	  }
      
      static Scalar getMass( const Inertia & self ) { return self.mass(); }
      static void setMass( Inertia & self, Scalar mass ) { self.mass() = mass; }
      
      static Vector3 getLever( const Inertia & self ) { return self.lever(); }
      static void setLever( Inertia & self, const Vector3 & lever ) { self.lever() = lever; }
      
      static Matrix3 getInertia( const Inertia & self ) { return self.inertia().matrix(); }
      static void setInertia( Inertia & self, const Vector6 & minimal_inertia ) { self.inertia().data() = minimal_inertia; }

      static Inertia* makeFromMCI(const double & mass,
				     const Vector3 & lever,
				     const Matrix3 & inertia) 
      {
        if(! inertia.isApprox(inertia.transpose()) )
          throw eigenpy::Exception("The 3d inertia should be symmetric.");
        if( (Eigen::Vector3d::UnitX().transpose()*inertia*Eigen::Vector3d::UnitX()<0)
           || (Eigen::Vector3d::UnitY().transpose()*inertia*Eigen::Vector3d::UnitY()<0)
           || (Eigen::Vector3d::UnitZ().transpose()*inertia*Eigen::Vector3d::UnitZ()<0) )
          throw eigenpy::Exception("The 3d inertia should be positive.");
        return new Inertia(mass,lever,inertia);
      }
      
      static std::string toString(const Inertia& m)
      {	  std::ostringstream s; s << m; return s.str();       }
      
      static void expose()
      {
        bp::class_<Inertia>("Inertia",
                               "Inertia matrix, in L(se3,se3*) == R^6x6.\n\n"
                               "Supported operations ...",
                               bp::init<>())
        .def(InertiaPythonVisitor<Inertia>())
        .def(CopyableVisitor<Inertia>())
        ;
        
      }


    }; // struct InertiaPythonVisitor
    
  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

