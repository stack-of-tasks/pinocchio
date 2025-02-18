//
// Copyright (c) 2015-2024 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_se3_hpp__
#define __pinocchio_python_spatial_se3_hpp__

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "pinocchio/utils/string.hpp"

#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::SE3)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename SE3>
    struct SE3PythonVisitor : public boost::python::def_visitor<SE3PythonVisitor<SE3>>
    {
      typedef typename SE3::Scalar Scalar;
      enum
      {
        Options = SE3::Options
      };
      typedef typename SE3::Matrix3 Matrix3;
      typedef typename SE3::Vector3 Vector3;
      typedef typename SE3::Matrix4 Matrix4;
      typedef typename SE3::Quaternion Quaternion;
      typedef typename SE3::ActionMatrixType ActionMatrixType;

      typedef MotionTpl<Scalar, Options> Motion;
      typedef ForceTpl<Scalar, Options> Force;
      typedef InertiaTpl<Scalar, Options> Inertia;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

        cl.def(bp::init<const Matrix3 &, const Vector3 &>(
                 (bp::arg("self"), bp::arg("rotation"), bp::arg("translation")),
                 "Initialize from a rotation matrix and a translation vector."))
          .def(bp::init<const Quaternion &, const Vector3 &>(
            (bp::arg("self"), bp::arg("quat"), bp::arg("translation")),
            "Initialize from a quaternion and a translation vector."))
          .def(bp::init<int>((bp::arg("self"), bp::arg("int")), "Init to identity."))
          .def(bp::init<const Matrix4 &>(
            (bp::arg("self"), bp::arg("array")), "Initialize from an homogeneous matrix."))
          .def(bp::init<const SE3 &>((bp::arg("self"), bp::arg("clone")), "Copy constructor"))

          .add_property(
            "rotation",
            bp::make_function(
              (typename SE3::AngularRef(SE3::*)()) & SE3::rotation,
              bp::return_internal_reference<>()),
            (void(SE3::*)(const Matrix3 &)) & SE3::rotation,
            "The rotation part of the transformation.")
          .add_property(
            "translation",
            bp::make_function(
              (typename SE3::LinearRef(SE3::*)()) & SE3::translation,
              bp::return_internal_reference<>()),
            (void(SE3::*)(const Vector3 &)) & SE3::translation,
            "The translation part of the transformation.")

          .add_property(
            "homogeneous", &SE3::toHomogeneousMatrix,
            "Returns the equivalent homegeneous matrix (acting on SE3).")
          .add_property(
            "action", (ActionMatrixType(SE3::*)() const)&SE3::toActionMatrix,
            "Returns the related action matrix (acting on Motion).")
          .def(
            "toActionMatrix", (ActionMatrixType(SE3::*)() const)&SE3::toActionMatrix,
            bp::arg("self"), "Returns the related action matrix (acting on Motion).")
          .add_property(
            "actionInverse", (ActionMatrixType(SE3::*)() const)&SE3::toActionMatrixInverse,
            "Returns the inverse of the action matrix (acting on Motion).\n"
            "This is equivalent to do m.inverse().action")
          .def(
            "toActionMatrixInverse", (ActionMatrixType(SE3::*)() const)&SE3::toActionMatrixInverse,
            bp::arg("self"),
            "Returns the inverse of the action matrix (acting on Motion).\n"
            "This is equivalent to do m.inverse().toActionMatrix()")
          .add_property(
            "dualAction", (ActionMatrixType(SE3::*)() const)&SE3::toDualActionMatrix,
            "Returns the related dual action matrix (acting on Force).")
          .def(
            "toDualActionMatrix", (ActionMatrixType(SE3::*)() const)&SE3::toDualActionMatrix,
            bp::arg("self"), "Returns the related dual action matrix (acting on Force).")

          .def(
            "setIdentity", &SE3PythonVisitor::setIdentity, bp::arg("self"),
            "Set *this to the identity placement.")
          .def(
            "setRandom", &SE3PythonVisitor::setRandom, bp::arg("self"),
            "Set *this to a random placement.")

          .def("inverse", &SE3::inverse, bp::arg("self"), "Returns the inverse transform")

          .def(
            "act", (Vector3(SE3::*)(const Vector3 &) const)&SE3::act, bp::args("self", "point"),
            "Returns a point which is the result of the entry point transforms by *this.")
          .def(
            "actInv", (Vector3(SE3::*)(const Vector3 &) const)&SE3::actInv,
            bp::args("self", "point"),
            "Returns a point which is the result of the entry point by the inverse of *this.")

          .def(
            "act", (SE3(SE3::*)(const SE3 & other) const)&SE3::act, bp::args("self", "M"),
            "Returns the result of *this * M.")
          .def(
            "actInv", (SE3(SE3::*)(const SE3 & other) const)&SE3::actInv, bp::args("self", "M"),
            "Returns the result of the inverse of *this times M.")

          .def(
            "act", (Motion(SE3::*)(const Motion &) const)&SE3::act, bp::args("self", "motion"),
            "Returns the result action of *this onto a Motion.")
          .def(
            "actInv", (Motion(SE3::*)(const Motion &) const)&SE3::actInv,
            bp::args("self", "motion"), "Returns the result of the inverse of *this onto a Motion.")

          .def(
            "act", (Force(SE3::*)(const Force &) const)&SE3::act, bp::args("self", "force"),
            "Returns the result of *this onto a Force.")
          .def(
            "actInv", (Force(SE3::*)(const Force &) const)&SE3::actInv, bp::args("self", "force"),
            "Returns the result of the inverse of *this onto an Inertia.")

          .def(
            "act", (Inertia(SE3::*)(const Inertia &) const)&SE3::act, bp::args("self", "inertia"),
            "Returns the result of *this onto a Force.")
          .def(
            "actInv", (Inertia(SE3::*)(const Inertia &) const)&SE3::actInv,
            bp::args("self", "inertia"),
            "Returns the result of the inverse of *this onto an Inertia.")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(
            "isApprox", &SE3::isApprox,
            (bp::arg("self"), bp::arg("other"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to other, within the precision given "
            "by prec.")

          .def(
            "isIdentity", &SE3::isIdentity, (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the identity placement, within the "
            "precision given by prec.")
#endif

          .def("__invert__", &SE3::inverse, "Returns the inverse of *this.")
          .def(bp::self * bp::self)
          .def("__mul__", &__mul__<Motion>)
          .def("__mul__", &__mul__<Force>)
          .def("__mul__", &__mul__<Inertia>)
          .def("__mul__", &__mul__<Vector3>)
          .add_property("np", &SE3::toHomogeneousMatrix)

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif

          .def("Identity", &SE3::Identity, "Returns the identity transformation.")
          .staticmethod("Identity")
          .def("Random", &SE3::Random, "Returns a random transformation.")
          .staticmethod("Random")
          .def(
            "Interpolate", &SE3::template Interpolate<Scalar>, bp::args("A", "B", "alpha"),
            "Linear interpolation on the SE3 manifold.\n\n"
            "This method computes the linear interpolation between A and B, such that the "
            "result C = A + (B-A)*t if it would be applied on classic Euclidian space.\n"
            "This operation is very similar to the SLERP operation on Rotations.\n"
            "Parameters:\n"
            "\tA: Initial transformation\n"
            "\tB: Target transformation\n"
            "\talpha: Interpolation factor")
          .staticmethod("Interpolate")

          .def("__array__", &SE3::toHomogeneousMatrix)
          .def(
            "__array__", &__array__,
            (bp::arg("self"), bp::arg("dtype") = bp::object(), bp::arg("copy") = bp::object()))

#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          .def_pickle(Pickle())
#endif
          ;
      }

      static std::string scopeName()
      {
        static std::string scope_name;
        return scope_name;
      }

      static void expose()
      {
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2, 9, 0)
        typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(SE3) HolderType;
#else
        typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<SE3, HolderType>(
          "SE3", "SE3 transformation defined by a 3d vector and a rotation matrix.",
          bp::init<>(bp::arg("self"), "Default constructor."))
          .def(SE3PythonVisitor<SE3>())
          .def(CastVisitor<SE3>())
          .def(ExposeConstructorByCastVisitor<SE3, ::pinocchio::SE3>())
          .def(CopyableVisitor<SE3>())
          .def(bp::self_ns::str(bp::self_ns::self))
          .def("__repr__", &repr);
      }

    private:
      static Matrix4 __array__(const SE3 & self, bp::object, bp::object)
      {
        return self.toHomogeneousMatrix();
      }

      struct Pickle : bp::pickle_suite
      {
        static boost::python::tuple getinitargs(const SE3 & M)
        {
          return bp::make_tuple((Matrix3)M.rotation(), (Vector3)M.translation());
        }

        static bool getstate_manages_dict()
        {
          return true;
        }
      };

      static void setIdentity(SE3 & self)
      {
        self.setIdentity();
      }
      static void setRandom(SE3 & self)
      {
        self.setRandom();
      }

      template<typename Spatial>
      static Spatial __mul__(const SE3 & self, const Spatial & other)
      {
        return self.act(other);
      }

      static std::string repr(const SE3 & self)
      {
        //        bp::object
        //        py_rotation(bp::handle<>(eigenpy::EigenToPy<Matrix3,Scalar>::convert(self.rotation())));
        //        std::string rotation_repr =
        //        bp::extract<std::string>(py_rotation.attr("__repr__")());
        //
        //        bp::object
        //        py_translation(bp::handle<>(eigenpy::EigenToPy<Vector3,Scalar>::convert(self.translation())));
        //        std::string translation_repr =
        //        bp::extract<std::string>(py_translation.attr("__repr__")());

        bp::object py_homogeneous(
          bp::handle<>(eigenpy::EigenToPy<Matrix4, Scalar>::convert(self.toHomogeneousMatrix())));
        std::string homegeneous_repr = bp::extract<std::string>(py_homogeneous.attr("__repr__")());
        replace(homegeneous_repr, "\n", "");
        replace(homegeneous_repr, "       ", "");

        std::stringstream ss_repr;
        ss_repr << "SE3(";
        ss_repr << homegeneous_repr;
        ss_repr << ")";

        return ss_repr.str();
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_spatial_se3_hpp__
