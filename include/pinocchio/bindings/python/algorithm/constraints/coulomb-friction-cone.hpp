//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_algorithm_constraints_coulomb_friction_cone_hpp__
#define __pinocchio_python_algorithm_constraints_coulomb_friction_cone_hpp__

#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"

#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename CoulombFrictionCone>
    struct CoulombFrictionConePythonVisitor
    : public boost::python::def_visitor<CoulombFrictionConePythonVisitor<CoulombFrictionCone>>
    {
      typedef typename CoulombFrictionCone::Scalar Scalar;
      typedef CoulombFrictionCone Self;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<const Scalar &>(bp::args("self", "mu"), "Default constructor"))
          .def(bp::init<const Self &>(bp::args("self", "other"), "Copy constructor"))

          .def_readwrite("mu", &Self::mu, "Friction coefficient.")

          .def(
            "isInside", &Self::template isInside<context::Vector3s>, bp::args("self", "f"),
            "Check whether a vector x lies within the cone.")

          .def(
            "project", &Self::template project<context::Vector3s>, bp::args("self", "f"),
            "Normal projection of a vector f onto the cone.")
          .def(
            "weightedProject", &Self::template weightedProject<context::Vector3s>,
            bp::args("self", "f", "R"), "Weighted projection of a vector f onto the cone.")
          .def(
            "computeNormalCorrection", &Self::template computeNormalCorrection<context::Vector3s>,
            bp::args("self", "v"),
            "Compute the complementary shift associted to the Coulomb friction cone for "
            "complementarity satisfaction in complementary problems.")
          .def(
            "computeRadialProjection", &Self::template computeRadialProjection<context::Vector3s>,
            bp::args("self", "f"),
            "Compute the radial projection associted to the Coulomb friction cone.")

          .def("dual", &Self::dual, bp::arg("self"), "Returns the dual cone associated to this")

          .def("dim", Self::dim, "Returns the dimension of the cone.")
          .staticmethod("dim")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
          ;
      }

      static void expose()
      {
        bp::class_<CoulombFrictionCone>(
          "CoulombFrictionCone", "3d Coulomb friction cone.\n", bp::no_init)
          .def(CoulombFrictionConePythonVisitor())
          //        .def(CastVisitor<CoulombFrictionCone>())
          //        .def(ExposeConstructorByCastVisitor<CoulombFrictionCone,::pinocchio::CoulombFrictionCone>())
          .def(CopyableVisitor<CoulombFrictionCone>());
      }
    };

    template<typename DualCoulombFrictionCone>
    struct DualCoulombFrictionConePythonVisitor
    : public boost::python::def_visitor<
        DualCoulombFrictionConePythonVisitor<DualCoulombFrictionCone>>
    {
      typedef typename DualCoulombFrictionCone::Scalar Scalar;
      typedef DualCoulombFrictionCone Self;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<const Scalar &>(bp::args("self", "mu"), "Default constructor"))
          .def(bp::init<const Self &>(bp::args("self", "other"), "Copy constructor"))

          .def_readwrite("mu", &Self::mu, "Friction coefficient.")

          .def(
            "isInside", &Self::template isInside<context::Vector3s>, bp::args("self", "v"),
            "Check whether a vector x lies within the cone.")

          .def(
            "project", &Self::template project<context::Vector3s>, bp::args("self", "v"),
            "Project a vector v onto the cone.")

          .def("dual", &Self::dual, bp::arg("self"), "Returns the dual cone associated to this.")

          .def("dim", Self::dim, "Returns the dimension of the cone.")
          .staticmethod("dim")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
          ;
      }

      static void expose()
      {
        bp::class_<DualCoulombFrictionCone>(
          "DualCoulombFrictionCone", "Dual cone of the 3d Coulomb friction cone.\n", bp::no_init)
          .def(DualCoulombFrictionConePythonVisitor())
          //        .def(CastVisitor<DualCoulombFrictionCone>())
          //        .def(ExposeConstructorByCastVisitor<DualCoulombFrictionCone,::pinocchio::DualCoulombFrictionCone>())
          .def(CopyableVisitor<DualCoulombFrictionCone>());
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_constraints_coulomb_friction_cone_hpp__
