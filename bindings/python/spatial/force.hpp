//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_force_hpp__
#define __pinocchio_python_spatial_force_hpp__

#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Force)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
  
    template<typename T> struct call;
  
    template<typename Scalar, int Options>
    struct call< ForceTpl<Scalar,Options> >
    {
      typedef ForceTpl<Scalar,Options> Force;
      
      static bool isApprox(const Force & self, const Force & other,
                           const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isApprox(other,prec);
      }
      
      static bool isZero(const Force & self,
                         const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isZero(prec);
      }
    };

    BOOST_PYTHON_FUNCTION_OVERLOADS(isApproxForce_overload,call<Force>::isApprox,2,3)
    BOOST_PYTHON_FUNCTION_OVERLOADS(isZero_overload,call<Force>::isZero,1,2)

    template<typename Force>
    struct ForcePythonVisitor
    : public boost::python::def_visitor< ForcePythonVisitor<Force> >
    {
      enum { Options = traits<Motion>::Options };
      
      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;
      typedef typename Force::Scalar Scalar;
      
      typedef typename Eigen::Map<Vector3> MapVector3;
      typedef typename Eigen::Ref<Vector3> RefVector3;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>("Default constructor"))
        .def(bp::init<Vector3,Vector3>
             ((bp::arg("linear"),bp::arg("angular")),
              "Initialize from linear and angular components of a Wrench vector (don't mix the order)."))
        .def(bp::init<Vector6>((bp::arg("Vector 6d")),"Init from a vector 6 [force,torque]"))
        .def(bp::init<Force>((bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",
                      bp::make_function(&ForcePythonVisitor::getLinear,
                                        bp::with_custodian_and_ward_postcall<0,1>()),
                      &ForcePythonVisitor::setLinear,
                      "Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.")
        .add_property("angular",
                      bp::make_function(&ForcePythonVisitor::getAngular,
                                        bp::with_custodian_and_ward_postcall<0,1>()),
                      &ForcePythonVisitor::setAngular,
                      "Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.")
        .add_property("vector",
                      bp::make_function((typename Force::ToVectorReturnType (Force::*)())&Force::toVector,
                                        bp::return_internal_reference<>()),
                      &ForcePythonVisitor::setVector,
                      "Returns the components of *this as a 6d vector.")
        .add_property("np",
                      bp::make_function((typename Force::ToVectorReturnType (Force::*)())&Force::toVector,
                                        bp::return_internal_reference<>()))
        
        .def("se3Action",&Force::template se3Action<Scalar,Options>,
             bp::args("self","M"),"Returns the result of the dual action of M on *this.")
        .def("se3ActionInverse",&Force::template se3ActionInverse<Scalar,Options>,
             bp::args("self","M"),"Returns the result of the dual action of the inverse of M on *this.")
        
        .def("setZero",&ForcePythonVisitor::setZero,bp::arg("self"),
             "Set the linear and angular components of *this to zero.")
        .def("setRandom",&ForcePythonVisitor::setRandom,bp::arg("self"),
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
        
        .def("isApprox",
             &call<Force>::isApprox,
             isApproxForce_overload(bp::args("self","other","prec"),
                                     "Returns true if *this is approximately equal to other, within the precision given by prec."))
                                                                                           
        .def("isZero",
             &call<Force>::isZero,
             isZero_overload(bp::args("self","prec"),
                             "Returns true if *this is approximately equal to the zero Force, within the precision given by prec."))
        
        .def("Random",&Force::Random,"Returns a random Force.")
        .staticmethod("Random")
        .def("Zero",&Force::Zero,"Returns a zero Force.")
        .staticmethod("Zero")
        
        .def("__array__",bp::make_function((typename Force::ToVectorReturnType (Force::*)())&Force::toVector,
                                           bp::return_internal_reference<>()))
        
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
      
      static RefVector3 getLinear(Force & self ) { return self.linear(); }
      static void setLinear(Force & self, const Vector3 & f) { self.linear(f); }
      static RefVector3 getAngular(Force & self) { return self.angular(); }
      static void setAngular(Force & self, const Vector3 & n) { self.angular(n); }
      
      static void setZero(Force & self) { self.setZero(); }
      static void setRandom(Force & self) { self.setRandom(); }
      
      static void setVector(Force & self, const Vector6 & f) { self = f; }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_spatial_force_hpp__
