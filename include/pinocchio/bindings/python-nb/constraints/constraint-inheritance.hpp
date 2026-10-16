// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include "pinocchio/constraints.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Visitor exposing members and methods introduced at each level of the constraint model CRTP
/// hierarchy.
template<typename ModelDerived>
struct ConstraintModelInheritanceVisitor
: nanobind::def_visitor<ConstraintModelInheritanceVisitor<ModelDerived>>
{
  using Self = ModelDerived;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;

    if constexpr (is_tpl_base_of_v<BinaryKinematicsConstraintModelBase, Self>)
    {
      using ConstraintData = typename Self::ConstraintData;
      using MatrixSize6 = typename Self::MatrixSize6;

      cl.def(
          nb::init<const Model &, JointIndex, const SE3 &, JointIndex, const SE3 &>(), "model"_a,
          "joint1_id"_a, "joint1_placement"_a, "joint2_id"_a, "joint2_placement"_a,
          "Constructor from given joint index and placement for the two joints "
          "implied in the constraint.")
        .def(
          nb::init<const Model &, JointIndex>(), "model"_a, "joint1_id"_a,
          "Constructor from given joint index of the first joint implied in the constraint.")
        .def(
          nb::init<const Model &, JointIndex, const SE3 &>(), "model"_a, "joint1_id"_a,
          "joint1_placement"_a,
          "Constructor from given joint index and placement of the first joint "
          "implied in the constraint.")
        .def(
          nb::init<const Model &, JointIndex, JointIndex>(), "model"_a, "joint1_id"_a,
          "joint2_id"_a,
          "Constructor from given joint index for the two joints implied in the constraint.")
        .def(nb::init<const Model &>(), "model"_a, "Constructor from the model only.")
        .def(
          "getA1",
          [](const Self & self, const ConstraintData & cdata, ReferenceFrame rf) -> MatrixSize6 {
            switch (rf)
            {
            case WORLD:
              return self.getA1(cdata, WorldFrameTag());
            case LOCAL:
              return self.getA1(cdata, LocalFrameTag());
            case LOCAL_WORLD_ALIGNED:
              return self.getA1(cdata, LocalWorldAlignedFrameTag());
            default:
              PINOCCHIO_UNREACHABLE();
            }
          },
          "constraint_data"_a, "reference_frame"_a,
          "Returns the constraint projector associated with joint 1. "
          "This matrix transforms a spatial velocity expressed in a reference frame "
          "to the first component of the constraint associated with joint 1.")
        .def(
          "getA2",
          [](const Self & self, const ConstraintData & cdata, ReferenceFrame rf) -> MatrixSize6 {
            switch (rf)
            {
            case WORLD:
              return self.getA2(cdata, WorldFrameTag());
            case LOCAL:
              return self.getA2(cdata, LocalFrameTag());
            case LOCAL_WORLD_ALIGNED:
              return self.getA2(cdata, LocalWorldAlignedFrameTag());
            default:
              PINOCCHIO_UNREACHABLE();
            }
          },
          "constraint_data"_a, "reference_frame"_a,
          "Returns the constraint projector associated with joint 2. "
          "This matrix transforms a spatial velocity expressed in a reference frame "
          "to the first component of the constraint associated with joint 2.")
        .def_rw("joint1_id", &Self::joint1_id, "Index of the first joint in the model tree.")
        .def_rw("joint2_id", &Self::joint2_id, "Index of the second joint in the model tree.")
        .def_rw(
          "joint1_placement", &Self::joint1_placement,
          "Position of attached point with respect to the frame of joint1.")
        .def_rw(
          "joint2_placement", &Self::joint2_placement,
          "Position of attached point with respect to the frame of joint2.")
        .def_rw(
          "desired_constraint_offset", &Self::desired_constraint_offset,
          "Desired constraint shift at position level.")
        .def_rw(
          "desired_constraint_velocity", &Self::desired_constraint_velocity,
          "Desired constraint velocity at velocity level.")
        .def_rw(
          "desired_constraint_acceleration", &Self::desired_constraint_acceleration,
          "Desired constraint velocity at acceleration level.")
        .def_rw("nv", &Self::nv, "Dimension of the model velocity.")
        .def_rw("depth_joint1", &Self::depth_joint1, "Depth of the kinematic tree for joint1.")
        .def_rw("depth_joint2", &Self::depth_joint2, "Depth of the kinematic tree for joint2.");
    }

    if constexpr (is_tpl_base_of_v<PointConstraintModelBase, Self>)
    {
      using Matrix6s = Eigen::Matrix<Scalar, 6, 6>;
      using Vector3s = Eigen::Matrix<Scalar, 3, 1>;

      cl.def(
        "computeConstraintSpatialInertia",
        [](const Self & self, const SE3 & placement, const Vector3s & diagonal_constraint_inertia)
          -> Matrix6s {
          return self.computeConstraintSpatialInertia(
            placement, diagonal_constraint_inertia.asDiagonal());
        },
        "placement"_a, "diagonal_constraint_inertia"_a,
        "This function computes the spatial inertia associated with the constraint.");
    }
  }
};

/// Visitor exposing members and methods introduced at each level of the constraint data CRTP
/// hierarchy.
template<typename DataDerived>
struct ConstraintDataInheritanceVisitor
: nanobind::def_visitor<ConstraintDataInheritanceVisitor<DataDerived>>
{
  using Self = DataDerived;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;

    if constexpr (
      std::is_base_of_v<PointConstraintDataBase<Self>, Self>
      || std::is_base_of_v<FrameConstraintDataBase<Self>, Self>)
    {
      using ConstraintModel = typename Self::ConstraintModel;

      cl.def(nb::init<>(), "Default constructor.")
        .def(nb::init<const ConstraintModel &>(), "constraint_model"_a, "From model constructor.")
        .def_rw("constraint_force", &Self::constraint_force, "Resulting force.")
        .def_rw("oMc1", &Self::oMc1, "Placement of the constraint frame 1 wrt WORLD.")
        .def_rw("oMc2", &Self::oMc2, "Placement of the constraint frame 2 wrt WORLD.")
        .def_rw("c1Mc2", &Self::c1Mc2, "Placement of the constraint frame 2 wrt frame 1.")
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
          "A_local", &Self::A_local,
          "Relative Transform between joint1 and joint2 in local frame.");
    }
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
