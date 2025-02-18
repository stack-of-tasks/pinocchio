//
// Copyright (c) 2015-2023 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_spatial_force_hpp__
#define __pinocchio_python_spatial_force_hpp__

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"

#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Force)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename T>
    struct call;

    template<typename Scalar, int Options>
    struct call<ForceTpl<Scalar, Options>>
    {
      typedef ForceTpl<Scalar, Options> Force;

      static bool isApprox(
        const Force & self,
        const Force & other,
        const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isApprox(other, prec);
      }

      static bool
      isZero(const Force & self, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
      {
        return self.isZero(prec);
      }
    };

    template<typename Force>
    struct ForcePythonVisitor : public boost::python::def_visitor<ForcePythonVisitor<Force>>
    {
      enum
      {
        Options = traits<Force>::Options
      };

      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;
      typedef typename Force::Scalar Scalar;

      typedef typename Eigen::Map<Vector3> MapVector3;
      typedef typename Eigen::Ref<Vector3> RefVector3;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();
        PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
        PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_SELF_ASSIGN_OVERLOADED
        cl.def(bp::init<>(bp::arg("self"), "Default constructor"))
          .def(bp::init<const Vector3 &, const Vector3 &>(
            (bp::arg("self"), bp::arg("linear"), bp::arg("angular")),
            "Initialize from linear and angular components of a Wrench vector (don't mix the "
            "order)."))
          .def(bp::init<const Vector6 &>(
            (bp::args("self", "array")), "Init from a vector 6 [force,torque]"))
          .def(bp::init<const Force &>((bp::arg("self"), bp::arg("clone")), "Copy constructor"))

          .add_property(
            "linear",
            bp::make_function(
              &ForcePythonVisitor::getLinear, bp::with_custodian_and_ward_postcall<0, 1>()),
            &ForcePythonVisitor::setLinear,
            "Linear part of a *this, corresponding to the linear velocity in case of a "
            "Spatial velocity.")
          .add_property(
            "angular",
            bp::make_function(
              &ForcePythonVisitor::getAngular, bp::with_custodian_and_ward_postcall<0, 1>()),
            &ForcePythonVisitor::setAngular,
            "Angular part of a *this, corresponding to the angular velocity in case of "
            "a Spatial velocity.")
          .add_property(
            "vector",
            bp::make_function(
              (typename Force::ToVectorReturnType(Force::*)()) & Force::toVector,
              bp::return_internal_reference<>()),
            &ForcePythonVisitor::setVector, "Returns the components of *this as a 6d vector.")
          .add_property(
            "np", bp::make_function(
                    (typename Force::ToVectorReturnType(Force::*)()) & Force::toVector,
                    bp::return_internal_reference<>()))

          .def(
            "se3Action", &Force::template se3Action<Scalar, Options>, bp::args("self", "M"),
            "Returns the result of the dual action of M on *this.")
          .def(
            "se3ActionInverse", &Force::template se3ActionInverse<Scalar, Options>,
            bp::args("self", "M"),
            "Returns the result of the dual action of the inverse of M on *this.")

          .def(
            "setZero", &ForcePythonVisitor::setZero, bp::arg("self"),
            "Set the linear and angular components of *this to zero.")
          .def(
            "setRandom", &ForcePythonVisitor::setRandom, bp::arg("self"),
            "Set the linear and angular components of *this to random values.")

          .def(
            "dot", (Scalar(Force::*)(const MotionDense<context::Motion> &) const)&Force::dot,
            bp::args("self", "m"), "Dot product between *this and a Motion m.")

          .def(bp::self + bp::self)
          .def(bp::self += bp::self)
          .def(bp::self - bp::self)
          .def(bp::self -= bp::self)
          .def(-bp::self)

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif

          .def(bp::self * Scalar())
          .def(Scalar() * bp::self)
          .def(bp::self / Scalar())

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(
            "isApprox", &call<Force>::isApprox,
            (bp::arg("self"), bp::arg("other"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to other, within the precision given "
            "by prec.")

          .def(
            "isZero", &call<Force>::isZero, (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the zero Force, within the "
            "precision given by prec.")
#endif

          .def("Random", &Force::Random, "Returns a random Force.")
          .staticmethod("Random")
          .def("Zero", &Force::Zero, "Returns a zero Force.")
          .staticmethod("Zero")

          .def(
            "__array__", bp::make_function(
                           (typename Force::ToVectorReturnType(Force::*)()) & Force::toVector,
                           bp::return_internal_reference<>()))
          .def(
            "__array__", &__array__,
            (bp::arg("self"), bp::arg("dtype") = bp::object(), bp::arg("copy") = bp::object()),
            bp::return_internal_reference<>())
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          .def_pickle(Pickle())
#endif
          ;
        PINOCCHIO_COMPILER_DIAGNOSTIC_POP
      }

      static void expose()
      {
        typedef pinocchio::ForceBase<Force> ForceBase;
        bp::objects::register_dynamic_id<ForceBase>();
        bp::objects::register_conversion<Force, ForceBase>(false);

        typedef pinocchio::ForceDense<Force> ForceDense;
        bp::objects::register_dynamic_id<ForceBase>();
        bp::objects::register_conversion<Force, ForceDense>(false);

#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2, 9, 0)
        typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(Force) HolderType;
#else
        typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<Force, HolderType>(
          "Force",
          "Force vectors, in se3* == F^6.\n\n"
          "Supported operations ...",
          bp::no_init)
          .def(ForcePythonVisitor<Force>())
          .def(CastVisitor<Force>())
          .def(ExposeConstructorByCastVisitor<Force, ::pinocchio::Force>())
          .def(CopyableVisitor<Force>())
          .def(PrintableVisitor<Force>());
      }

    private:
      static typename Force::ToVectorConstReturnType
      __array__(const Force & self, bp::object, bp::object)
      {
        return self.toVector();
      }

      struct Pickle : bp::pickle_suite
      {
        static boost::python::tuple getinitargs(const Force & f)
        {
          return bp::make_tuple((Vector3)f.linear(), (Vector3)f.angular());
        }

        static bool getstate_manages_dict()
        {
          return true;
        }
      };

      static RefVector3 getLinear(Force & self)
      {
        return self.linear();
      }
      static void setLinear(Force & self, const Vector3 & f)
      {
        self.linear(f);
      }
      static RefVector3 getAngular(Force & self)
      {
        return self.angular();
      }
      static void setAngular(Force & self, const Vector3 & n)
      {
        self.angular(n);
      }

      static void setZero(Force & self)
      {
        self.setZero();
      }
      static void setRandom(Force & self)
      {
        self.setRandom();
      }

      static void setVector(Force & self, const Vector6 & f)
      {
        self = f;
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_spatial_force_hpp__
