// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/constraints.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using Vector3s = Eigen::Matrix<Scalar, 3, 1>;

template<typename VectorType>
struct SetPythonVisitor : nb::def_visitor<SetPythonVisitor<VectorType>>
{
  template<class Set, class... Ts, class... Extra>
  void execute(nb::class_<Set, Ts...> & cl, const Extra &...) const
  {
    cl.def(
        "isInside",
        [](const Set & self, Eigen::Ref<const VectorType> v) {
          return self.isInside(v, Scalar(0.));
        },
        "f"_a, "Check if vector is inside the set.")
      .def(
        "project",
        [](const Set & self, Eigen::Ref<const VectorType> v) -> auto { return self.project(v); },
        "f"_a, "Normal projection of a vector f onto the set.")
      .def(ComparableVisitor<Set>());
  }
};

struct ConeSetPythonVisitor : nb::def_visitor<ConeSetPythonVisitor>
{
  template<class ConeSet, typename... Ts, class... Extra>
  void execute(nb::class_<ConeSet, Ts...> & cl, const Extra &...) const
  {
    cl.def("dual", &ConeSet::dual, "Returns the dual cone associated to this object.");
  }
};

void exposeCones(nb::module_ m)
{
  nb::class_<CoulombFrictionCone>(m, "CoulombFrictionCone", "3D Coulomn friction cone.")
    .def(SetPythonVisitor<Vector3s>())
    .def(ConeSetPythonVisitor())
    .def(
      nb::init<const Scalar &>(), "mu"_a,
      "Constructor from a given friction coefficient (held by reference).")
    .def(nb::init<const CoulombFrictionCone &>(), "other"_a)
    .def_prop_ro(
      "mu", [](const CoulombFrictionCone & self) -> Scalar { return self.mu; },
      "Friction coefficient.")
    .def(
      "weightedProject",
      [](const CoulombFrictionCone & self, const Vector3s & x, const Vector3s & R) {
        self.weightedProject(x, R);
      },
      "x"_a, "R"_a, "Weighted projection of a vector f onto the cone, following weights matrix R.")
    .def(
      "computeNormalCorrection",
      [](const CoulombFrictionCone & self, const Vector3s & v) {
        return self.computeNormalCorrection(v);
      },
      "Compute the complementary shift associted to the Coulomb friction cone for complementarity "
      "satisfaction in complementary problems.")
    .def(
      "computeRadialProjection",
      [](const CoulombFrictionCone & self, const Vector3s & f) {
        return self.computeRadialProjection(f);
      },
      "f"_a, "Compute the radial projection associted to the Coulomb friction cone.");

  nb::class_<DualCoulombFrictionCone>(
    m, "DualCoulombFrictionCone", "Dual cone of the 3D Coulomb friction cone")
    .def(SetPythonVisitor<Vector3s>())
    .def(ConeSetPythonVisitor())
    .def(
      nb::init<const Scalar &>(), "mu"_a,
      "Constructor from a given friction coefficient (held by reference).");
}

PINOCCHIO_PYTHON_NAMESPACE_END
