//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_se3_hpp__
#define __pinocchio_python_se3_hpp__

#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::SE3)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(isIdentity_overload,SE3::isIdentity,0,1)

    template<typename SE3>
    struct SE3PythonVisitor
      : public boost::python::def_visitor< SE3PythonVisitor<SE3> >
    {
      typedef typename SE3::Scalar Scalar;
      typedef typename SE3::Matrix3 Matrix3;
      typedef typename SE3::Vector3 Vector3;
      typedef typename SE3::Matrix4 Matrix4;

    public:

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Matrix3,Vector3>
             ((bp::arg("Rotation"),bp::arg("translation")),
              "Initialize from rotation and translation."))
        .def(bp::init<int>((bp::arg("trivial arg (should be 1)")),"Init to identity."))
        .def(bp::init<SE3>((bp::arg("other")), "Copy constructor."))
        .def(bp::init<Matrix4>
             ((bp::arg("Homogeneous matrix")),
              "Initialize from a homogeneous matrix."))

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
        
        .add_property("homogeneous",&SE3::toHomogeneousMatrix,
                      "Returns the homegeneous matrix of *this (acting on SE3).")
        .add_property("action",&SE3::toActionMatrix,
                      "Returns the action matrix of *this (acting on Motion).")
        .def("toActionMatrix",&SE3::toActionMatrix,
             "Returns the action matrix of *this (acting on Motion).")
        .add_property("actionInverse",&SE3::toActionMatrixInverse,
                      "Returns the inverse of the action matrix of *this (acting on Motion).\n"
                      "This is equivalent to do m.inverse().action")
        .def("toActionMatrixInverse",&SE3::toActionMatrixInverse,
             "Returns the inverse of the action matrix of *this (acting on Motion).\n"
             "This is equivalent to do m.inverse().toActionMatrix()")
        .add_property("dualAction",&SE3::toDualActionMatrix,
                      "Returns the dual action matrix of *this (acting on Force).")
        .def("toDualActionMatrix",&SE3::toDualActionMatrix,
             "Returns the dual action matrix of *this (acting on Force).")
        
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
        
        .def("act", (Motion (SE3::*)(const Motion &) const) &SE3::act,
             bp::args("motion"), "Returns the result action of *this onto a Motion.")
        .def("actInv", (Motion (SE3::*)(const Motion &) const) &SE3::actInv,
             bp::args("motion"), "Returns the result of the inverse of *this onto a Motion.")
        
        .def("act", (Force (SE3::*)(const Force &) const) &SE3::act,
             bp::args("force"), "Returns the result of *this onto a Force.")
        .def("actInv", (Force (SE3::*)(const Force &) const) &SE3::actInv,
             bp::args("force"), "Returns the result of the inverse of *this onto an Inertia.")
        
        .def("act", (Inertia (SE3::*)(const Inertia &) const) &SE3::act,
             bp::args("inertia"), "Returns the result of *this onto a Force.")
        .def("actInv", (Inertia (SE3::*)(const Inertia &) const) &SE3::actInv,
             bp::args("inertia"), "Returns the result of the inverse of *this onto an Inertia.")
        
        .def("isApprox",(bool (SE3::*)(const SE3 & other, const Scalar & prec)) &SE3::isApprox,bp::args("other","prec"),"Returns true if *this is approximately equal to other, within the precision given by prec.")
        .def("isApprox",(bool (SE3::*)(const SE3 & other)) &SE3::isApprox,bp::args("other"),"Returns true if *this is approximately equal to other.")
        
        .def("isIdentity",&SE3::isIdentity,isIdentity_overload(bp::args("prec"),"Returns true if *this is approximately equal to the identity placement, within the precision given by prec."))
        
        .def("__invert__",&SE3::inverse,"Returns the inverse of *this.")
        .def(bp::self * bp::self)
        .def("__mul__",&__mul__<Motion>)
        .def("__mul__",&__mul__<Force>)
        .def("__mul__",&__mul__<Inertia>)
        .def("__mul__",&__mul__<Vector3>)
        .add_property("np",&SE3::toHomogeneousMatrix)
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("Identity",&SE3::Identity,"Returns the identity transformation.")
        .staticmethod("Identity")
        .def("Random",&SE3::Random,"Returns a random transformation.")
        .staticmethod("Random")
        
        .def_pickle(Pickle())
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
      
      struct Pickle : bp::pickle_suite
      {
        static
        boost::python::tuple
        getinitargs(const SE3 & M)
        { return bp::make_tuple((Matrix3)M.rotation(),(Vector3)M.translation()); }
      };  
      
      static Vector3 getTranslation(const SE3 & self) { return self.translation(); }
      static Matrix3 getRotation(const SE3 & self) { return self.rotation(); }
      
      static void setIdentity(SE3 & self) { self.setIdentity(); }
      static void setRandom(SE3 & self) { self.setRandom(); }
      
      template<typename Spatial>
      static Spatial __mul__(const SE3 & self, const Spatial & other)
      { return self.act(other); }
    };
    


  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_se3_hpp__

