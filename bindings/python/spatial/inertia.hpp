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

#ifndef __se3_python_inertia_hpp__
#define __se3_python_inertia_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::Inertia)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Inertia>
    struct InertiaPythonVisitor
      : public boost::python::def_visitor< InertiaPythonVisitor<Inertia> >
    {
      
      typedef typename Inertia::Scalar Scalar;
      typedef typename Inertia::Vector3 Vector3;
      typedef typename Inertia::Matrix3 Matrix3;
      typedef typename Inertia::Vector6 Vector6;
      typedef typename Inertia::Matrix6 Matrix6;

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
        
        .add_property("mass",
                      &InertiaPythonVisitor::getMass,
                      &InertiaPythonVisitor::setMass,
                      "Mass of the Spatial Inertia.")
        .add_property("lever",
                      &InertiaPythonVisitor::getLever,
                      &InertiaPythonVisitor::setLever,
                      "Center of mass location of the Spatial Inertia. It corresponds to the location of the center of mass regarding to the frame where the Spatial Inertia is expressed.")
        .add_property("inertia",
                      &InertiaPythonVisitor::getInertia,
                      &InertiaPythonVisitor::setInertia,
                      "Rotational part of the Spatial Inertia, i.e. a symmetric matrix representing the rotational inertia around the center of mass.")
        
        .def("matrix",&Inertia::matrix)
        .def("se3Action",&Inertia::se3Action,
             bp::args("M"),"Returns the result of the action of M on *this.")
        .def("se3ActionInverse",&Inertia::se3ActionInverse,
             bp::args("M"),"Returns the result of the action of the inverse of M on *this.")
        
        .def("setIdentity",&Inertia::setIdentity,"Set *this to be the Identity inertia.")
        .def("setZero",&Inertia::setZero,"Set all the components of *this to zero.")
        .def("setRandom",&Inertia::setRandom,"Set all the components of *this to random values.")
        
        .def(bp::self + bp::self)
        .def(bp::self * bp::other<Motion>() )
        .add_property("np",&Inertia::matrix)
        .def("vxiv",&Inertia::vxiv,bp::arg("Motion v"),"Returns the result of v x Iv.")
        .def("vtiv",&Inertia::vtiv,bp::arg("Motion v"),"Returns the result of v.T * Iv.")
        .def("vxi",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::vxi,bp::arg("Motion v"),"Returns the result of v x* I, a 6x6 matrix.")
        .def("ivx",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::ivx,bp::arg("Motion v"),"Returns the result of I vx, a 6x6 matrix.")
        .def("variation",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::variation,bp::arg("Motion v"),"Returns the time derivative of the inertia.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("isApprox",(bool (Inertia::*)(const Inertia & other, const Scalar & prec)) &Inertia::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",(bool (Inertia::*)(const Inertia & other)) &Inertia::isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("Identity",&Inertia::Identity,"Returns the identity Inertia.")
        .staticmethod("Identity")
        .def("Zero",&Inertia::Zero,"Returns the null Inertia.")
        .staticmethod("Zero")
        .def("Random",&Inertia::Random,"Returns a random Inertia.")
        .staticmethod("Random")
        
        .def("FromEllipsoid", &Inertia::FromEllipsoid,
             bp::args("mass","length_x","length_y","length_z"),
             "Returns an Inertia of an ellipsoid shape with a mass and of dimension the semi axis of length_{x,y,z}.")
        .staticmethod("FromEllipsoid")
        .def("FromCylinder", &Inertia::FromCylinder,
             bp::args("mass","radius","length"),
             "Returns the Inertia of a cylinder shape ith a mass and of dimension radius and length.")
        .staticmethod("FromCylinder")
        .def("FromBox", &Inertia::FromBox,
             bp::args("mass","length_x","length_y","length_z"),
             "Returns an Inertia of a box shape with a mass and of dimension the semi axis of length_{x,y,z}.")
        .staticmethod("FromBox")
        
        .def_pickle(Pickle())
        ;
      }
      
      static Scalar getMass( const Inertia & self ) { return self.mass(); }
      static void setMass( Inertia & self, Scalar mass ) { self.mass() = mass; }
      
      static Vector3 getLever( const Inertia & self ) { return self.lever(); }
      static void setLever( Inertia & self, const Vector3 & lever ) { self.lever() = lever; }
      
      static Matrix3 getInertia( const Inertia & self ) { return self.inertia().matrix(); }
//      static void setInertia(Inertia & self, const Vector6 & minimal_inertia) { self.inertia().data() = minimal_inertia; }
      static void setInertia(Inertia & self, const Matrix3 & symmetric_inertia)
      {
        assert(symmetric_inertia.isApprox(symmetric_inertia.transpose()));
        self.inertia().data() <<
        symmetric_inertia(0,0),
        symmetric_inertia(1,0),
        symmetric_inertia(1,1),
        symmetric_inertia(0,2),
        symmetric_inertia(1,2),
        symmetric_inertia(2,2);
      }

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
      
      static void expose()
      {
        bp::class_<Inertia>("Inertia",
                            "This class represenses a sparse version of a Spatial Inertia and its is defined by its mass, its center of mass location and the rotational inertia expressed around this center of mass.\n\n"
                            "Supported operations ...",
                            bp::init<>())
        .def(InertiaPythonVisitor<Inertia>())
        .def(CopyableVisitor<Inertia>())
        .def(PrintableVisitor<Inertia>())
        ;
        
      }
      
    private:
      
      struct Pickle : bp::pickle_suite
      {
        static
        boost::python::tuple
        getinitargs(const Inertia & I)
        { return bp::make_tuple(I.mass(),(Vector3)I.lever(),I.inertia().matrix()); }
      };
      

    }; // struct InertiaPythonVisitor
    
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_se3_hpp__
