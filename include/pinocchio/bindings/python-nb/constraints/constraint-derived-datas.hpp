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
  using ConstantLengthConstraintData = ConstantLengthConstraintDataTpl<Scalar, Options>;

  // ── default: nothing extra
  template<typename T, typename PyClass>
  static void expose(Tag<T>, PyClass &)
  {
  }

  // ── ConstantLengthConstraintData
  template<typename PyClass>
  static void expose(Tag<ConstantLengthConstraintData>, PyClass & cl)
  {
    using namespace nb::literals;
    using Self = ConstantLengthConstraintData;
    using ConstraintModel = typename Self::ConstraintModel;

    cl.def(nb::init<>(), "Default constructor.")
      .def(nb::init<const ConstraintModel &>(), "constraint_model"_a, "From model constructor.")
      .def_rw("constraint_force", &Self::constraint_force, "Resulting force.")
      .def_rw("oMc1", &Self::oMc1, "Placement of the constraint frame 1 wrt WORLD.")
      .def_rw("oMc2", &Self::oMc2, "Placement of the constraint frame 2 wrt WORLD.")
      .def_rw("c1Mc2", &Self::c1Mc2, "Placement of the constraint frame 2 wrt frame 1.")
      .def_rw(
        "relative_position", &Self::relative_position,
        "Position of the second point expressed in the frame of the first one.")
      .def_rw("distance", &Self::distance, "Current distance between the two points.")
      .def_rw("direction", &Self::direction, "Unitary direction relative_position / distance.")
      .def_rw(
        "constraint_position_error", &Self::constraint_position_error,
        "Constraint position error.")
      .def_rw(
        "constraint_velocity_error", &Self::constraint_velocity_error,
        "Constraint velocity error.")
      .def_rw(
        "constraint_acceleration_error", &Self::constraint_acceleration_error,
        "Constraint acceleration error.")
      .def_rw(
        "constraint_acceleration_biais_term", &Self::constraint_acceleration_biais_term,
        "Constraint acceleration term.")
      .def_rw("A1_world", &Self::A1_world, "Transform for joint1 in world frame.")
      .def_rw("A2_world", &Self::A2_world, "Transform for joint2 in world frame.")
      .def_rw(
        "A_world", &Self::A_world, "Relative Transform between joint1 and joint2 in world frame.")
      .def_rw("A1_local", &Self::A1_local, "Transform for joint1 in local frame.")
      .def_rw("A2_local", &Self::A2_local, "Transform for joint2 in local frame.")
      .def_rw(
        "A_local", &Self::A_local, "Relative Transform between joint1 and joint2 in local frame.");
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
