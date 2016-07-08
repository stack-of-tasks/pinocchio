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
#include "pinocchio/spatial/inertia.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Inertia>
  {
    typedef se3::InertiaTpl<se3::Inertia::Scalar,Eigen::DontAlign> type;
  };

} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Inertia>
    struct InertiaPythonVisitor
      : public boost::python::def_visitor< InertiaPythonVisitor<Inertia> >
    {
      typedef typename eigenpy::UnalignedEquivalent<Inertia>::type Inertia_fx;
      typedef typename Inertia::Matrix3 Matrix3;
      typedef typename Inertia::Matrix6 Matrix6;
      typedef typename Inertia::Vector6 Vector6;
      typedef typename Inertia::Vector3 Vector3;

      typedef typename Inertia_fx::Matrix3 Matrix3_fx;
      typedef typename Inertia_fx::Matrix6 Matrix6_fx;
      typedef typename Inertia_fx::Vector6 Vector6_fx;
      typedef typename Inertia_fx::Vector3 Vector3_fx;
      typedef typename Inertia_fx::Motion  Motion_fx ;
      
      typedef typename Inertia_fx::Scalar Scalar;
      
    public:

      static PyObject* convert(Inertia const& m)
      {
	Inertia_fx m_fx = m;
	return bp::incref(bp::object(m_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def("__init__",
             bp::make_constructor(&InertiaPythonVisitor::makeFromMCI,
                                  bp::default_call_policies(),
                                  (bp::arg("mass"),bp::arg("lever"),bp::arg("inertia"))),
             "Initialize from mass, lever and 3d inertia.")
        
        .add_property("mass", &InertiaPythonVisitor::getMass, &InertiaPythonVisitor::setMass)
        .add_property("lever", &InertiaPythonVisitor::getLever, &InertiaPythonVisitor::setLever)
        .add_property("inertia", &InertiaPythonVisitor::getInertia, &InertiaPythonVisitor::setInertia)
        
        .def("matrix",&Inertia_fx::matrix)
        .def("se3Action",&Inertia_fx::se3Action)
        .def("se3ActionInverse",&Inertia_fx::se3ActionInverse)
        
        .def("setIdentity",&Inertia_fx::setIdentity)
        .def("setZero",&Inertia_fx::setZero)
        .def("setRandom",&Inertia_fx::setRandom)
        
        .def("__str__",&InertiaPythonVisitor::toString)
        .def( bp::self + bp::self)
        .def( bp::self * bp::other<Motion_fx>() )
        .add_property("np",&Inertia_fx::matrix)
        
        .def("Identity",&Inertia_fx::Identity)
        .staticmethod("Identity")
        .def("Zero",&Inertia_fx::Zero)
        .staticmethod("Zero")
        .def("Random",&Inertia_fx::Random)
        .staticmethod("Random")
        .def("FromEllipsoid", &Inertia_fx::FromEllipsoid,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("length_x"), bp::arg("length_y"), bp::arg("length_z")))
        .staticmethod("FromEllipsoid")
        .def("FromCylinder", &Inertia_fx::FromCylinder,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("radius"), bp::arg("length")))
        .staticmethod("FromCylinder")
        .def("FromBox", &Inertia_fx::FromBox,
            bp::default_call_policies(), (bp::arg("mass"),
              bp::arg("length_x"), bp::arg("length_y"), bp::arg("length_z")))
        .staticmethod("FromBox")
        ;
	  }
      
      static Scalar getMass( const Inertia_fx & self ) { return self.mass(); }
      static void setMass( Inertia_fx & self, Scalar mass ) { self.mass() = mass; }
      
      static Vector3_fx getLever( const Inertia_fx & self ) { return self.lever(); }
      static void setLever( Inertia_fx & self, const Vector3_fx & lever ) { self.lever() = lever; }
      
      static Matrix3_fx getInertia( const Inertia_fx & self ) { return self.inertia().matrix(); }
      static void setInertia( Inertia_fx & self, const Vector6_fx & minimal_inertia ) { self.inertia().data() = minimal_inertia; }

      static Inertia_fx* makeFromMCI(const double & mass,
				     const Vector3_fx & lever,
				     const Matrix3_fx & inertia) 
      {
        if(! inertia.isApprox(inertia.transpose()) )
          throw eigenpy::Exception("The 3d inertia should be symmetric.");
        if( (Eigen::Vector3d::UnitX().transpose()*inertia*Eigen::Vector3d::UnitX()<0)
           || (Eigen::Vector3d::UnitY().transpose()*inertia*Eigen::Vector3d::UnitY()<0)
           || (Eigen::Vector3d::UnitZ().transpose()*inertia*Eigen::Vector3d::UnitZ()<0) )
          throw eigenpy::Exception("The 3d inertia should be positive.");
        return new Inertia_fx(mass,lever,inertia);
      }
      
      static std::string toString(const Inertia_fx& m)
      {	  std::ostringstream s; s << m; return s.str();       }
      
      static void expose()
      {
        bp::class_<Inertia_fx>("Inertia",
                               "Inertia matrix, in L(se3,se3*) == R^6x6.\n\n"
                               "Supported operations ...",
                               bp::init<>())
        .def(InertiaPythonVisitor<Inertia>())
        ;
        
        bp::to_python_converter< Inertia,InertiaPythonVisitor<Inertia> >();
      }


    }; // struct InertiaPythonVisitor
    
  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

