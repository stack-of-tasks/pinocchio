// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include "pinocchio/constraints.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Visitor that adds type-specific constructors and properties to each concrete constraint
/// data class. execute() dispatches via Tag to an overloaded static expose(); the primary
/// template is a no-op so types with no extra bindings require no boilerplate.
struct ConstraintDataDerivedVisitor : nb::def_visitor<ConstraintDataDerivedVisitor>
{
  template<typename T>
  struct Tag
  {
  };

  using JointFrictionConstraintData = JointFrictionConstraintDataTpl<Scalar, Options>;
  using JointLimitConstraintData = JointLimitConstraintDataTpl<Scalar, Options>;

  // ── default: nothing extra
  template<typename T, typename PyClass>
  static void expose(Tag<T>, PyClass &)
  {
  }

  // ── JointFrictionConstraintData
  template<typename PyClass>
  static void expose(Tag<JointFrictionConstraintData>, PyClass & cl)
  {
    using namespace nb::literals;
    using ConstraintModel = typename JointFrictionConstraintData::ConstraintModel;
    cl.def(nb::init<const ConstraintModel &>(), "constraint_model"_a, "From model constructor.");
  }

  // ── JointLimitConstraintData
  template<typename PyClass>
  static void expose(Tag<JointLimitConstraintData>, PyClass & cl)
  {
    using namespace nb::literals;
    using Self = JointLimitConstraintData;
    using ConstraintModel = typename Self::ConstraintModel;
    using VectorXs = typename Self::VectorXs;

    cl.def(nb::init<const ConstraintModel &>(), "constraint_model"_a, "From model constructor.")
      .def_rw("rowise_tangent_map", &Self::rowise_tangent_map, "Rowise tangent map.")
      .def_prop_ro(
        "constraint_residual",
        [](const Self & self) -> Eigen::Ref<const VectorXs> { return self.constraint_residual; },
        nb::rv_policy::reference_internal);
  }

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    expose(Tag<typename PyClass::Type>{}, cl);
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
