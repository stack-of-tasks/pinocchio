// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include "pinocchio/constraints.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Visitor that adds type-specific constructors, properties, and methods to each concrete
/// constraint model class. execute() dispatches via Tag to an overloaded static expose(); the
/// primary template is a no-op so types with no extra bindings require no boilerplate.
struct ConstraintModelDerivedVisitor : nb::def_visitor<ConstraintModelDerivedVisitor>
{
  template<typename T>
  struct Tag
  {
  };

  using PointContactConstraintModel = PointContactConstraintModelTpl<Scalar, Options>;
  using JointFrictionConstraintModel = JointFrictionConstraintModelTpl<Scalar, Options>;
  using JointLimitConstraintModel = JointLimitConstraintModelTpl<Scalar, Options>;

  // ── default: nothing extra
  template<typename T, typename PyClass>
  static void expose(Tag<T>, PyClass &)
  {
  }

  // ── PointContactConstraintModel
  template<typename PyClass>
  static void expose(Tag<PointContactConstraintModel>, PyClass & cl)
  {
    cl.def(
        "getFriction", &PointContactConstraintModel::getFriction,
        "Get coulomb friction coefficient.")
      .def(
        "setFriction", &PointContactConstraintModel::setFriction,
        "Set coulomb friction coefficient.")
      .def_rw(
        "geom1_id", &PointContactConstraintModel::geom1_id, "Index of the first geometry object.")
      .def_rw(
        "geom2_id", &PointContactConstraintModel::geom2_id, "Index of the second geometry object.");
  }

  // ── JointFrictionConstraintModel
  template<typename PyClass>
  static void expose(Tag<JointFrictionConstraintModel>, PyClass & cl)
  {
    using namespace nb::literals;
    using Self = JointFrictionConstraintModel;
    using JointIndexVector = typename Self::JointIndexVector;

    cl.def(
        nb::init<const Model &, const JointIndexVector &>(), "model"_a, "m_active_joints"_a,
        "Constructor from given joint index vector implied in the constraint.")
      .def(nb::init<const Model &>(), "model"_a, "Constructor from the model only.")
      .def(
        "getActiveJoints", &Self::getActiveJoints, nb::rv_policy::reference_internal,
        "Get the active joints.")
      .def(
        "getActiveDofs", &Self::getActiveDofs, nb::rv_policy::reference_internal,
        "Get the active DOFs.")
      .def(
        "getFrictionLowerLimit", &Self::getFrictionLowerLimit, nb::rv_policy::reference_internal,
        "Get friction lower limit.")
      .def(
        "setFrictionLowerLimit",
        [](Self & self, ConstVectorRef lb) { self.setFrictionLowerLimit(lb); }, "lb"_a,
        "Set friction lower limit.")
      .def(
        "getFrictionUpperLimit", &Self::getFrictionUpperLimit, nb::rv_policy::reference_internal,
        "Get friction upper limit.")
      .def(
        "setFrictionUpperLimit",
        [](Self & self, ConstVectorRef ub) { self.setFrictionUpperLimit(ub); }, "ub"_a,
        "Set friction upper limit.");
  }

  // ── JointLimitConstraintModel
  template<typename PyClass>
  static void expose(Tag<JointLimitConstraintModel>, PyClass & cl)
  {
    using namespace nb::literals;
    using Self = JointLimitConstraintModel;
    using JointIndexVector = typename Self::JointIndexVector;

    cl.def(
        nb::init<const Model &, const JointIndexVector &>(), "model"_a, "activable_joints"_a,
        "Constructor from given joint index vector implied in the constraint.")
      .def(
        nb::init<const Model &, const JointIndexVector &, ConstVectorRef, ConstVectorRef>(),
        "model"_a, "activable_joints"_a, "lb"_a, "ub"_a,
        "Constructor from given joint index vector implied in the constraint.")
      .def(
        nb::init<
          const Model &, const JointIndexVector &, ConstVectorRef, ConstVectorRef,
          ConstVectorRef>(),
        "model"_a, "activable_joints"_a, "lb"_a, "ub"_a, "margin"_a,
        "Constructor from given joint index vector implied in the constraint.")
      .def(nb::init<const Model &>(), "model"_a, "Constructor from the model only.")
      .def(
        "getSelectedJoints", &Self::getSelectedJoints, nb::rv_policy::reference_internal,
        "Joints for which there is at least one position limit.")
      .def("getNqReduce", &Self::getNqReduce, "Sum of nq of activable joints.")
      .def("getMaxOfNvs", &Self::getMaxOfNvs, "Max nv of atomic joints in activable joints.")
      .def(
        "getActivablePositionLimit", &Self::getActivablePositionLimit,
        nb::rv_policy::reference_internal, "Position limit of the dof of the constraints.")
      .def(
        "lowerResidualSize",
        [](const Self & self, ConstraintSelectionType sel) -> int {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.lowerResidualSize(CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.lowerResidualSize(MaximalSelection());
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT)
      .def(
        "upperResidualSize",
        [](const Self & self, ConstraintSelectionType sel) -> int {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.upperResidualSize(CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.upperResidualSize(MaximalSelection());
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT)
      .def(
        "setPositionLimitAndMargin",
        [](Self & self, ConstVectorRef lb, ConstVectorRef ub, ConstVectorRef margin) {
          self.setPositionLimitAndMargin(lb, ub, margin);
        },
        "lb"_a, "ub"_a, "margin"_a,
        "Set position limit and margin for activable constraints from lower_bound, upper_bound "
        "and margin of size model.nq.")
      .def(
        "makeSelectionMaximal", &Self::makeSelectionMaximal,
        "Make the selection maximal (all activable constraints become active).")
      .def(
        "makeSelectionFilteredByLimitProximity",
        [](Self & self, ConstVectorRef q) { self.makeSelectionFilteredByLimitProximity(q); }, "q"_a,
        "Set the selection to constraints that are near their limits given configuration q.")
      .def(
        "active_idx_in_activable",
        [](const Self & self) -> const typename Self::VectorOfSize & {
          return self.active_idx_in_activable();
        },
        nb::rv_policy::reference_internal,
        "Vector of active indices in the activable constraints.");
  }

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    expose(Tag<typename PyClass::Type>{}, cl);
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
