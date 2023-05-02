//
// Copyright (c) 2015-2023 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_motion_hpp__
#define __pinocchio_python_spatial_motion_hpp__

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Motion)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename T> struct call;
    
    template<typename Scalar, int Options>
    struct call< MotionTpl<Scalar,Options> >
    {
      typedef MotionTpl<Scalar,Options> Motion;
      
      static bool isApprox(const Motion & self, const Motion & other,
                           const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isApprox(other,prec);
      }
      
      static bool isZero(const Motion & self,
                         const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isZero(prec);
      }
    };
  
    BOOST_PYTHON_FUNCTION_OVERLOADS(isApproxMotion_overload,call<Motion>::isApprox,2,3)
    BOOST_PYTHON_FUNCTION_OVERLOADS(isZero_overload,call<Motion>::isZero,1,2)

    template<typename Motion>
    struct MotionPythonVisitor
      : public boost::python::def_visitor< MotionPythonVisitor<Motion> >
    {
      enum { Options = traits<Motion>::Options };
      
      typedef typename Motion::Scalar Scalar;
      typedef ForceTpl<Scalar,traits<Motion>::Options> Force;
      typedef typename Motion::Vector6 Vector6;
      typedef typename Motion::Vector3 Vector3;
      
      typedef typename Eigen::Map<Vector3> MapVector3;
      typedef typename Eigen::Ref<Vector3> RefVector3;

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<>(bp::arg("self"),"Default constructor"))
        .def(bp::init<Vector3,Vector3>
             (bp::args("self","linear","angular"),
              "Initialize from linear and angular components of a Motion vector (don't mix the order)."))
        .def(bp::init<Vector6>((bp::arg("self"),bp::arg("array")),"Init from a vector 6 [linear velocity, angular velocity]"))
        .def(bp::init<Motion>((bp::arg("self"),bp::arg("other")),"Copy constructor."))
        
        .add_property("linear",
                      bp::make_function(&MotionPythonVisitor::getLinear,
                                        bp::with_custodian_and_ward_postcall<0,1>()),
                      &MotionPythonVisitor::setLinear,
                      "Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.")
        .add_property("angular",
                      bp::make_function(&MotionPythonVisitor::getAngular,
                                        bp::with_custodian_and_ward_postcall<0,1>()),
                      &MotionPythonVisitor::setAngular,
                      "Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.")
        .add_property("vector",
                      bp::make_function((typename Motion::ToVectorReturnType (Motion::*)())&Motion::toVector,
                                         bp::return_internal_reference<>()),
                      &MotionPythonVisitor::setVector,
                      "Returns the components of *this as a 6d vector.")
        .add_property("np",
                      bp::make_function((typename Motion::ToVectorReturnType (Motion::*)())&Motion::toVector,
                                         bp::return_internal_reference<>()))
        
        .def("se3Action",&Motion::template se3Action<Scalar,Options>,
             bp::args("self","M"),"Returns the result of the action of M on *this.")
        .def("se3ActionInverse",&Motion::template se3ActionInverse<Scalar,Options>,
             bp::args("self","M"),"Returns the result of the action of the inverse of M on *this.")
        
        .add_property("action",&Motion::toActionMatrix,"Returns the action matrix of *this (acting on Motion).")
        .add_property("dualAction",&Motion::toDualActionMatrix,"Returns the dual action matrix of *this (acting on Force).")
        .add_property("homogeneous", &Motion::toHomogeneousMatrix, "Equivalent homogeneous representation of the Motion vector")
        
        .def("setZero",&MotionPythonVisitor::setZero,bp::arg("self"),
             "Set the linear and angular components of *this to zero.")
        .def("setRandom",&MotionPythonVisitor::setRandom,bp::arg("self"),
             "Set the linear and angular components of *this to random values.")
        
        .def("cross",(Motion (Motion::*)(const Motion &) const) &Motion::cross,
             bp::args("self","m"),"Action of *this onto another Motion m. Returns ¨*this x m.")
        .def("cross",(Force (Motion::*)(const Force &) const) &Motion::cross,
             bp::args("self","f"),"Dual action of *this onto a Force f. Returns *this x* f.")
        
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
        
        .def("isApprox",
             call<Motion>::isApprox,
             isApproxMotion_overload(bp::args("self","other","prec"),
                                     "Returns true if *this is approximately equal to other, within the precision given by prec."))
                                                              
        .def("isZero",
             call<Motion>::isZero,
             isZero_overload(bp::args("self","prec"),
                             "Returns true if *this is approximately equal to the zero Motion, within the precision given by prec."))
        
        .def("Random",&Motion::Random,"Returns a random Motion.")
        .staticmethod("Random")
        .def("Zero",&Motion::Zero,"Returns a zero Motion.")
        .staticmethod("Zero")
        
        .def("__array__",bp::make_function((typename Motion::ToVectorReturnType (Motion::*)())&Motion::toVector,
                                            bp::return_internal_reference<>()))
        
        .def_pickle(Pickle())
        ;
      }

      static void expose()
      {
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2,9,0)
    typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(Motion) HolderType;
#else
    typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<Motion,HolderType>("Motion",
                           "Motion vectors, in se3 == M^6.\n\n"
                           "Supported operations ...",
                           bp::no_init)
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
        
        static bool getstate_manages_dict() { return true; }
      };
      
      static RefVector3 getLinear(Motion & self) { return self.linear(); }
      static void setLinear (Motion & self, const Vector3 & v) { self.linear(v); }
      static RefVector3 getAngular(Motion & self) { return self.angular(); }
      static void setAngular(Motion & self, const Vector3 & w) { self.angular(w); }
      
      static void setVector(Motion & self, const Vector6 & v) { self = v; }
      
      static void setZero(Motion & self) { self.setZero(); }
      static void setRandom(Motion & self) { self.setRandom(); }
      
    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_spatial_motion_hpp__
