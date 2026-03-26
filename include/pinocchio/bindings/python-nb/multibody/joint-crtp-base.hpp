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
      //
      .def("createData", &Derived::createData, "Create data associated to the joint model.")
      //
      .def_static("classname", &Derived::classname)
      .def(ComparableVisitor<Derived>());
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
