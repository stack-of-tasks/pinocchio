// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/printable.hpp"
#include "../utils/comparable.hpp"

#include "pinocchio/multibody.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

// Visitor for exposing the common interface of joint models.
template<typename Derived>
struct JointModelBaseVisitor : nb::def_visitor<JointModelBaseVisitor<Derived>>
{
  using Base = JointModelBase<Derived>;
  // Derived type's data.
  using JointDataDerived = typename Derived::JointDataDerived;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl //
       // The default constructor is protected in JointModelBase (CRTP), so nanobind cannot
       // call it directly. Use placement new via a lambda to bypass the access restriction.
      .def("__init__", [](Derived * t) { new (t) Derived(); })
      .def_prop_ro("id", &Derived::id)
      .def_prop_ro("idx_q", &Derived::idx_q)
      .def_prop_ro("idx_v", &Derived::idx_v)
      .def_prop_ro("idx_vExtended", &Derived::idx_vExtended)
      .def_prop_ro("nq", &Derived::nq)
      .def_prop_ro("nv", &Derived::nv)
      .def_prop_ro("nvExtended", &Derived::nvExtended)
      .def_prop_ro(
        "hasConfigurationLimit", &Derived::hasConfigurationLimit,
        "Return vector of boolean if joint has configuration limits.")
      .def_prop_ro(
        "hasConfigurationLimitInTangent", &Derived::hasConfigurationLimitInTangent,
        "Return vector of boolean if joint has configuration limits in tangent space.")
      .def(
        "setIndexes",
        [](Derived & self, const int id, const int idx_q, const int idx_v) {
          self.setIndexes(id, idx_q, idx_v);
        },
        "joint_id"_a, "idx_q"_a, "idx_v"_a)
      .def(
        "setIndexes",
        [](
          Derived & self, const int id, const int idx_q, const int idx_v, const int idx_vExtended) {
          self.setIndexes(id, idx_q, idx_v, idx_vExtended);
        },
        "joint_id"_a, "idx_q"_a, "idx_v"_a, "idx_vExtended"_a)
      .def_static("classname", &Derived::classname)
      .def(
        "calc",
        [](const Derived & self, JointDataDerived & jdata, const context::VectorXs & q) {
          self.calc(jdata, q);
        },
        "jdata"_a, "q"_a)
      .def(
        "calc",
        [](
          const Derived & self, JointDataDerived & jdata, const context::VectorXs & q,
          const context::VectorXs & v) { self.calc(jdata, q, v); },
        "jdata"_a, "q"_a, "v"_a)
      .def("createData", &Derived::createData, "Create data associated to the joint model.")
      .def(
        "hasSameIndexes",
        [](const Derived & self, const Derived & other) {
          return self.template hasSameIndexes<Derived>(other);
        },
        "other"_a, "Check if this joint model has the same indexes as other.")
      .def(
        "shortname", &Derived::shortname,
        "Returns string indicating the joint type (class name):"
        "\n\t- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]"
        "\n\t- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned "
        "with X, Y, nor Z"
        "\n\t- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with "
        "rotation axis [*] ∈ [X,Y,Z]"
        "\n\t- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation "
        "axis not aligned with X, Y, nor Z"
        "\n\t- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]"
        "\n\t- JointModelPlanar: Planar joint"
        "\n\t- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not "
        "aligned with X, Y, nor Z"
        "\n\t- JointModelSphericalZYX: Spherical joint (3D rotation)"
        "\n\t- JointModelTranslation: Translation joint (3D translation)"
        "\n\t- JointModelFreeFlyer: Joint enabling 3D rotation and translations.")
      .def(ComparableVisitor<Derived>());
  }
};

// Visitor for exposing the common interface of joint datas.
template<typename Derived>
struct JointDataBaseVisitor : nb::def_visitor<JointDataBaseVisitor<Derived>>
{
  using Base = JointDataBase<Derived>;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl //
      .def_prop_ro("joint_q", [](const Derived & self) { return self.joint_q_accessor(); })
      .def_prop_ro("joint_v", [](const Derived & self) { return self.joint_v_accessor(); })
      .def_prop_ro("S", [](const Derived & self) { return self.S_accessor().matrix(); })
      .def_prop_ro("M", [](const Derived & self) { return self.M_accessor(); })
      .def_prop_ro("v", [](const Derived & self) { return self.v_accessor(); })
      .def_prop_ro("c", [](const Derived & self) { return self.c_accessor(); })
      .def_prop_ro("U", [](const Derived & self) { return self.U_accessor(); })
      .def_prop_ro("Dinv", [](const Derived & self) { return self.Dinv_accessor(); })
      .def_prop_ro("UDinv", [](const Derived & self) { return self.UDinv_accessor(); })
      .def("shortname", &Derived::shortname)
      .def(ComparableVisitor<Derived>());
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
