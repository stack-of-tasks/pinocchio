//
// Copyright (c) 2015-2023 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_inertia_hpp__
#define __pinocchio_python_spatial_inertia_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Inertia)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
      
    template<typename T> struct call;
      
    template<typename Scalar, int Options>
    struct call< InertiaTpl<Scalar,Options> >
    {
      typedef InertiaTpl<Scalar,Options> Inertia;
      
      static bool isApprox(const Inertia & self, const Inertia & other,
                           const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isApprox(other,prec);
      }
      
      static bool isZero(const Inertia & self,
                         const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isZero(prec);
      }
    };
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(isApproxInertia_overload,call<Inertia>::isApprox,2,3)
    BOOST_PYTHON_FUNCTION_OVERLOADS(isZero_overload,call<Inertia>::isZero,1,2)

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
                                  bp::args("mass","lever","inertia")),
             "Initialize from mass, lever and 3d inertia.")
        .def(bp::init<Inertia>(bp::args("self","other"),"Copy constructor."))
        
        .add_property("mass",
                      &InertiaPythonVisitor::getMass,
                      &InertiaPythonVisitor::setMass,
                      "Mass of the Spatial Inertia.")
        .add_property("lever",
                      bp::make_function((typename Inertia::Vector3 & (Inertia::*)())&Inertia::lever,
                                        bp::return_internal_reference<>()),
                      &InertiaPythonVisitor::setLever,
                      "Center of mass location of the Spatial Inertia. It corresponds to the location of the center of mass regarding to the frame where the Spatial Inertia is expressed.")
        .add_property("inertia",
                      &InertiaPythonVisitor::getInertia,
                      &InertiaPythonVisitor::setInertia,
                      "Rotational part of the Spatial Inertia, i.e. a symmetric matrix representing the rotational inertia around the center of mass.")
        
        .def("matrix",&Inertia::matrix,bp::arg("self"))
        .def("se3Action",&Inertia::se3Action,
             bp::args("self","M"),"Returns the result of the action of M on *this.")
        .def("se3ActionInverse",&Inertia::se3ActionInverse,
             bp::args("self","M"),"Returns the result of the action of the inverse of M on *this.")
        
        .def("setIdentity",&Inertia::setIdentity,bp::arg("self"),
             "Set *this to be the Identity inertia.")
        .def("setZero",&Inertia::setZero,bp::arg("self"),
             "Set all the components of *this to zero.")
        .def("setRandom",&Inertia::setRandom,bp::arg("self"),
             "Set all the components of *this to random values.")
        
        .def(bp::self + bp::self)
        .def(bp::self * bp::other<Motion>() )
        .add_property("np",&Inertia::matrix)
        .def("vxiv",&Inertia::vxiv,bp::args("self","v"),"Returns the result of v x Iv.")
        .def("vtiv",&Inertia::vtiv,bp::args("self","v"),"Returns the result of v.T * Iv.")
        .def("vxi",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::vxi,
             bp::args("self","v"),
             "Returns the result of v x* I, a 6x6 matrix.")
        .def("ivx",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::ivx,
             bp::args("self","v"),
             "Returns the result of I vx, a 6x6 matrix.")
        .def("variation",(Matrix6 (Inertia::*)(const Motion &) const)&Inertia::variation,
             bp::args("self","v"),
             "Returns the time derivative of the inertia.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("isApprox",
             call<Inertia>::isApprox,
             isApproxInertia_overload(bp::args("self","other","prec"),
                                      "Returns true if *this is approximately equal to other, within the precision given by prec."))
                                                                                                                         
        .def("isZero",
             call<Inertia>::isZero,
             isZero_overload(bp::args("self","prec"),
                             "Returns true if *this is approximately equal to the zero Inertia, within the precision given by prec."))
        
        .def("Identity",&Inertia::Identity,"Returns the identity Inertia.")
        .staticmethod("Identity")
        .def("Zero",&Inertia::Zero,"Returns the null Inertia.")
        .staticmethod("Zero")
        .def("Random",&Inertia::Random,"Returns a random Inertia.")
        .staticmethod("Random")
        
        .def("toDynamicParameters",&InertiaPythonVisitor::toDynamicParameters_proxy,bp::arg("self"),
             "Returns the representation of the matrix as a vector of dynamic parameters."
              "\nThe parameters are given as v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T "
              "where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter"
        )
        .def("FromDynamicParameters",&Inertia::template FromDynamicParameters<Eigen::VectorXd>,
              bp::args("dynamic_parameters"),
              "Builds and inertia matrix from a vector of dynamic parameters."
              "\nThe parameters are given as dynamic_parameters = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T "
              "where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter."
        )
        .staticmethod("FromDynamicParameters")

        .def("FromSphere", &Inertia::FromSphere,
             bp::args("mass","radius"),
             "Returns the Inertia of a sphere defined by a given mass and radius.")
        .staticmethod("FromSphere")
        .def("FromEllipsoid", &Inertia::FromEllipsoid,
             bp::args("mass","length_x","length_y","length_z"),
             "Returns the Inertia of an ellipsoid shape defined by a mass and given dimensions the semi-axis of values length_{x,y,z}.")
        .staticmethod("FromEllipsoid")
        .def("FromCylinder", &Inertia::FromCylinder,
             bp::args("mass","radius","length"),
             "Returns the Inertia of a cylinder defined by its mass, radius and length along the Z axis.")
        .staticmethod("FromCylinder")
        .def("FromBox", &Inertia::FromBox,
             bp::args("mass","length_x","length_y","length_z"),
             "Returns the Inertia of a box shape with a mass and of dimension the semi axis of length_{x,y,z}.")
        .staticmethod("FromBox")
        
        .def("__array__",&Inertia::matrix)
        
        .def_pickle(Pickle())
        ;
      }
      
      static Scalar getMass( const Inertia & self ) { return self.mass(); }
      static void setMass( Inertia & self, Scalar mass ) { self.mass() = mass; }
      
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

      static Eigen::VectorXd toDynamicParameters_proxy(const Inertia & self)
      {
        return self.toDynamicParameters();
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
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2,9,0)
    typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(Inertia) HolderType;
#else
    typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<Inertia,HolderType>("Inertia",
                            "This class represenses a sparse version of a Spatial Inertia and its is defined by its mass, its center of mass location and the rotational inertia expressed around this center of mass.\n\n"
                            "Supported operations ...",
                            bp::init<>(bp::arg("self"),"Default constructor."))
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
        
        static bool getstate_manages_dict() { return true; }
      };
      

    }; // struct InertiaPythonVisitor
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_spatial_inertia_hpp__
