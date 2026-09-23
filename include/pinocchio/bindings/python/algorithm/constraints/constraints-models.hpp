//
// Copyright (c) 2025 INRIA
//

#pragma once

#include "pinocchio/constraints.hpp"
#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/algorithm/constraints/constraint-model-inheritance.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    // ---------------------
    // Generic expose_constraint_model : do nothing special
    template<class T>
    bp::class_<T> & expose_constraint_model(bp::class_<T> & cl)
    {
      return cl;
    }

    // specialization for terminal ConstraintModels
    template<>
    bp::class_<context::PointContactConstraintModel> &
    expose_constraint_model(bp::class_<context::PointContactConstraintModel> & cl)
    {
      typedef context::PointContactConstraintModel Self;
      return cl.def("getFriction", &Self::getFriction, "Get coulomb friction coefficient.")
        .def("setFriction", &Self::setFriction, "Set coulomb friction coefficient.")
        .def_readwrite("geom1_id", &Self::geom1_id, "Index of the first geometry object.")
        .def_readwrite("geom2_id", &Self::geom2_id, "Index of the second geometry object.");
    }

    template<>
    bp::class_<context::ConstantLengthConstraintModel> &
    expose_constraint_model(bp::class_<context::ConstantLengthConstraintModel> & cl)
    {
      typedef context::ConstantLengthConstraintModel Self;
      typedef context::SE3 SE3;
      return cl
        .def(
          bp::init<
            const context::Model &, JointIndex, const SE3 &, JointIndex, const SE3 &,
            context::Scalar>(
            (bp::arg("self"), bp::arg("model"), bp::arg("joint1_id"), bp::arg("joint1_placement"),
             bp::arg("joint2_id"), bp::arg("joint2_placement"), bp::arg("length")),
            "Constructor from the joint indexes and placements of the two points "
            "implied in the constraint, and from the constant length to enforce between them."))
        .def(
          "getLength", &Self::getLength, bp::return_value_policy<bp::copy_const_reference>(),
          "Get the constant length enforced by the constraint.")
        .def(
          "setLength", &Self::setLength, bp::args("self", "length"),
          "Set the constant length enforced by the constraint.");
    }

    template<>
    bp::class_<context::JointFrictionConstraintModel> &
    expose_constraint_model(bp::class_<context::JointFrictionConstraintModel> & cl)
    {
      typedef typename context::JointFrictionConstraintModel::JointIndexVector JointIndexVector;
      typedef context::JointFrictionConstraintModel Self;
      typedef context::VectorXs VectorXs;
      cl.def(
          bp::init<const context::Model &, const JointIndexVector &>(
            (bp::arg("self"), bp::arg("model"), bp::arg("m_active_joints")),
            "Constructor from given joint index vector "
            "implied in the constraint."))
        .def(
          bp::init<
            const context::Model &, const JointIndexVector &, const context::VectorXs &,
            const context::VectorXs &>(
            (bp::arg("self"), bp::arg("model"), bp::arg("m_active_joints"), bp::arg("lb"),
             bp::arg("ub")),
            "Full constructor from model, active joints, lower and upper friction bounds."))
        .def(
          bp::init<const context::Model &>(
            (bp::arg("self"), bp::arg("model")), "Constructor from the model only."))
        .def(
          "getActiveJoints", &Self::getActiveJoints,
          bp::return_value_policy<bp::copy_const_reference>())
        .def(
          "getActiveDofs", &Self::getActiveDofs,
          bp::return_value_policy<bp::copy_const_reference>())
        .def(
          "getFrictionLowerLimit", &Self::getFrictionLowerLimit,
          bp::return_value_policy<bp::copy_const_reference>(), "Get friction lower limit.")
        .def(
          "setFrictionLowerLimit", bp::make_function(+[](Self & self, const VectorXs & lb) {
            self.setFrictionLowerLimit(lb);
          }),
          "Set friction lower limit.")
        .def(
          "getFrictionUpperLimit", &Self::getFrictionUpperLimit,
          bp::return_value_policy<bp::copy_const_reference>(), "Get friction upper limit.")
        .def(
          "setFrictionUpperLimit", bp::make_function(+[](Self & self, const VectorXs & ub) {
            self.setFrictionUpperLimit(ub);
          }),
          "Set friction upper limit.")
        .def(
          "getTimeStep", &Self::getTimeStep,
          "Get dt used to convert friction force limits to impulse limits.")
        .def(
          "setTimeStep", +[](Self & self, const context::Scalar dt) { self.setTimeStep(dt); },
          bp::args("self", "dt"),
          "Set dt used to convert friction force limits to impulse limits.");
      return cl;
    }

    template<>
    bp::class_<context::JointLimitConstraintModel> &
    expose_constraint_model(bp::class_<context::JointLimitConstraintModel> & cl)
    {
      typedef typename context::JointLimitConstraintModel::JointIndexVector JointIndexVector;
      typedef typename context::JointLimitConstraintModel Self;
      cl.def(
          bp::init<const context::Model &, const JointIndexVector &>(
            (bp::arg("self"), bp::arg("model"), bp::arg("activable_joints")),
            "Constructor from given joint index vector "
            "implied in the constraint."))
        .def(
          bp::init<
            const context::Model &, const JointIndexVector &, const context::VectorXs &,
            const context::VectorXs &>(
            (bp::arg("self"), bp::arg("model"), bp::arg("activable_joints"), bp::arg("lb"),
             bp::arg("ub")),
            "Constructor from given joint index vector "
            "implied in the constraint."))
        .def(
          bp::init<
            const context::Model &, const JointIndexVector &, const context::VectorXs &,
            const context::VectorXs &, const context::VectorXs &>(
            (bp::arg("self"), bp::arg("model"), bp::arg("activable_joints"), bp::arg("lb"),
             bp::arg("ub"), bp::arg("margin")),
            "Constructor from given joint index vector "
            "implied in the constraint."))
        .def(
          bp::init<const context::Model &>(
            (bp::arg("self"), bp::arg("model")), "Constructor from the model only."))
        .def(
          "getSelectedJoints", &Self::getSelectedJoints,
          bp::return_value_policy<bp::copy_const_reference>(),
          "Joints for which there is at least one position limit.")
        .def("getNqReduce", &Self::getNqReduce, "Sum of nq of activable joints.")
        .def("getMaxOfNvs", &Self::getMaxOfNvs, "Max nv of atomic joints in activable joints.")
        .def(
          "getActivablePositionLimit", &Self::getActivablePositionLimit,
          bp::return_value_policy<bp::copy_const_reference>(),
          "Position limit of the dof of the constraints.")
        .def(
          "lowerResidualSize",
          +[](const Self & self, ConstraintSelectionType sel) -> int {
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
          (bp::arg("self"), bp::arg("sel") = ConstraintSelectionType::CURRENT))
        .def(
          "upperResidualSize",
          +[](const Self & self, ConstraintSelectionType sel) -> int {
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
          (bp::arg("self"), bp::arg("sel") = ConstraintSelectionType::CURRENT))
        .def(
          "setPositionLimitAndMargin",
          +[](
             Self & self, const context::VectorXs & lb, const context::VectorXs & ub,
             const context::VectorXs & margin) -> void {
            self.setPositionLimitAndMargin(lb, ub, margin);
          },
          (bp::arg("self"), bp::arg("lb"), bp::arg("ub"), bp::arg("margin")),
          "Set position limit and margin for activable constraints from lower_bound, upper_bound "
          "and margin of size model.nq.")
        .def(
          "makeSelectionMaximal", &Self::makeSelectionMaximal, bp::arg("self"),
          "Make the selection maximal (all activable constraints become active).")
        .def(
          "makeSelectionFilteredByLimitProximity",
          +[](Self & self, const context::VectorXs & q) {
            self.makeSelectionFilteredByLimitProximity(q);
          },
          bp::args("self", "q"),
          "Set the selection to constraints that are near their limits given configuration q.")
        .def(
          "active_idx_in_activable",
          +[](const Self & self) -> const typename Self::VectorOfSize & {
            return self.active_idx_in_activable();
          },
          bp::return_internal_reference<>(),
          "Vector of active indices in the activable constraints.");
      return cl;
    }
  } // namespace python
} // namespace pinocchio
