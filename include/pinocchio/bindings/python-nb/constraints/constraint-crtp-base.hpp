// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"

#include "pinocchio/constraints.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<typename ModelDerived>
struct ConstraintModelBaseVisitor : nanobind::def_visitor<ConstraintModelBaseVisitor<ModelDerived>>
{
  using Self = ModelDerived;
  using DataDerived = typename ModelDerived::ConstraintData;

  using ConstraintSet = typename ::pinocchio::traits<Self>::ConstraintSet;

  using ResidualVectorType = typename ::pinocchio::traits<Self>::ResidualVectorType;
  using JacobianMatrixType = typename ::pinocchio::traits<Self>::JacobianMatrixType;
  using ConeVectorType = typename ::pinocchio::traits<Self>::ConeVectorType;
  using ConeScalingVectorType = typename ::pinocchio::traits<Self>::ConeScalingVectorType;

  static constexpr bool has_baumgarte_corrector =
    ::pinocchio::traits<Self>::has_baumgarte_corrector;
  static constexpr bool has_set = ::pinocchio::traits<Self>::has_set;
  using BaumgarteCorrectorParameters = BaumgarteCorrectorParametersTpl<context::Scalar>;

  using ForceVector = std::vector<Force>;
  using MotionVector = std::vector<Motion>;

  using BooleanVector = typename Self::BooleanVector;
  using EigenIndexVector = typename Self::EigenIndexVector;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def_rw("name", &Self::name, "Name of the constraint.")
      .def_static("classname", Self::classname)
      .def("shortname", &Self::shortname, "Short name of the class.")
      .def("createData", &Self::createData, "Create a Data object for the given constraint model.")
      .def(
        "residualSize",
        [](const Self & self, ConstraintSelectionType sel) {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.residualSize(CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.residualSize(MaximalSelection());
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT, "Constraint size for the selection.")
      .def(
        "symmetricConeResidualSize",
        [](const Self & self, ConstraintSelectionType sel) {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.symmetricConeResidualSize(CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.symmetricConeResidualSize(MaximalSelection());
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT,
        "Symmetric cone residual size for the selection.")
      .def(
        "symmetricConeResidualScalingSize",
        [](const Self & self, ConstraintSelectionType sel) {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.symmetricConeResidualScalingSize(CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.symmetricConeResidualScalingSize(MaximalSelection());
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT)
      .def(
        "setCompliance",
        [](Self & self, const ResidualVectorType & vector, ConstraintSelectionType sel) {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.setCompliance(vector, CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.setCompliance(vector, MaximalSelection());
          }
        },
        "vector"_a, "sel"_a = ConstraintSelectionType::CURRENT,
        "Set the compliance value for the selected constraint.")
      .def(
        "retrieveCompliance",
        [](const Self & self, ConstraintSelectionType sel) -> ResidualVectorType {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT: {
            ResidualVectorType res(self.residualSize(CurrentSelection()));
            self.retrieveCompliance(res, CurrentSelection());
            return res;
          }
          case ConstraintSelectionType::MAXIMAL: {
            ResidualVectorType res(self.residualSize(MaximalSelection()));
            self.retrieveCompliance(res, MaximalSelection());
            return res;
          }
          default:
            PINOCCHIO_UNREACHABLE();
          }
        },
        "sel"_a = ConstraintSelectionType::CURRENT,
        "Retrieve the compliance value for the selected constraint.")
      .def(
        "setBaumgarteCorrectorParameters",
        [](Self & self, const BaumgarteCorrectorParameters & params, ConstraintSelectionType sel) {
          switch (sel)
          {
          case ConstraintSelectionType::CURRENT:
            return self.setBaumgarteCorrectorParameters(params, CurrentSelection());
          case ConstraintSelectionType::MAXIMAL:
            return self.setBaumgarteCorrectorParameters(params, MaximalSelection());
          }
        },
        "params"_a, "sel"_a = ConstraintSelectionType::CURRENT, "Set the Baumgarte parameters.")
      .def(
        "calc",
        [](const Self & self, const Model & model, const Data & data, DataDerived & cdata) {
          self.calc(model, data, cdata);
        },
        "model"_a, "data"_a, "constraint_data"_a,
        "Evaluate the constraint values at the current state given by data and store the results.")
      .def(
        "getRowSparsityPattern",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          Eigen::Index row_id) -> BooleanVector {
          BooleanVector res;
          self.getRowSparsityPattern(model, data, cdata, row_id, res);
          return res;
        },
        "model"_a, "data"_a, "constraint_data"_a, "row_id"_a,
        "Active colwise sparsity associated with a given row.")
      .def(
        "getRowIndexes",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          Eigen::Index row_id) -> EigenIndexVector {
          EigenIndexVector res;
          self.getRowIndexes(model, data, cdata, row_id, res);
          return res;
        },
        "model"_a, "data"_a, "constraint_data"_a, "row_id"_a,
        "Vector of the active indexes associated with a given row.")
      .def(
        "jacobian",
        (JacobianMatrixType (Self::*)(const Model &, const Data &, DataDerived &) const)
          & Self::jacobian,
        "model"_a, "data"_a, "constraint_data"_a, "Compute the constraint jacobian.")
      .def(
        "jacobianMatrixProduct",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          const context::MatrixXs & matrix) -> context::MatrixXs {
          context::MatrixXs res = context::MatrixXs::Zero(self.residualSize(), matrix.cols());
          self.jacobianMatrixProduct(model, data, cdata, matrix, res);
          return res;
        },
        "model"_a, "data"_a, "constraint_data"_a, "matrix"_a,
        "Forward chain rule: return product between the jacobian and a matrix.")
      .def(
        "jacobianTransposeMatrixProduct",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          const context::MatrixXs & matrix) -> context::MatrixXs {
          context::MatrixXs res = context::MatrixXs::Zero(model.nv, matrix.cols());
          self.jacobianTransposeMatrixProduct(model, data, cdata, matrix, res);
          return res;
        },
        "model"_a, "data"_a, "constraint_data"_a, "matrix"_a,
        "Backward chain rule: return product between the jacobian transpose and a matrix.")
      .def(
        "mapConstraintForceToJointSpace",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          const VectorXs & constraint_forces, ForceVector & joint_forces,
          ReferenceFrame rf) -> VectorXs {
          VectorXs joint_torques = VectorXs::Zero(model.nv);
          switch (rf)
          {
          case WORLD:
            self.mapConstraintForceToJointSpace(
              model, data, cdata, constraint_forces, joint_forces, joint_torques, WorldFrameTag());
            break;
          case LOCAL:
            self.mapConstraintForceToJointSpace(
              model, data, cdata, constraint_forces, joint_forces, joint_torques, LocalFrameTag());
            break;
          case LOCAL_WORLD_ALIGNED:
            self.mapConstraintForceToJointSpace(
              model, data, cdata, constraint_forces, joint_forces, joint_torques,
              LocalWorldAlignedFrameTag());
            break;
          }
          return joint_torques;
        },
        "model"_a, "data"_a, "constraint_data"_a, "constraint_forces"_a, "joint_forces"_a,
        "reference_frame"_a = LOCAL, "Map constraint forces to joint space.")
      .def(
        "mapJointSpaceToConstraintMotion",
        [](
          const Self & self, const Model & model, const Data & data, const DataDerived & cdata,
          const MotionVector & joint_motions, const VectorXs & joint_generalized_velocity,
          ReferenceFrame rf) -> VectorXs {
          VectorXs constraint_motions = VectorXs::Zero(self.residualSize());
          switch (rf)
          {
          case WORLD:
            self.mapJointSpaceToConstraintMotion(
              model, data, cdata, joint_motions, joint_generalized_velocity, constraint_motions,
              WorldFrameTag());
            break;
          case LOCAL:
            self.mapJointSpaceToConstraintMotion(
              model, data, cdata, joint_motions, joint_generalized_velocity, constraint_motions,
              LocalFrameTag());
            break;
          case LOCAL_WORLD_ALIGNED:
            self.mapJointSpaceToConstraintMotion(
              model, data, cdata, joint_motions, joint_generalized_velocity, constraint_motions,
              LocalWorldAlignedFrameTag());
            break;
          }
          return constraint_motions;
        },
        "model"_a, "data"_a, "constraint_data"_a, "joint_motions"_a, "joint_generalized_velocity"_a,
        "reference_frame"_a = LOCAL, "Map joint space quantities to constraint motion.")
      .def(
        "appendCouplingConstraintInertias",
        [](
          const Self & self, const Model & model, Data & data, const DataDerived & cdata,
          const VectorXs & diagonal_constraint_inertia, ReferenceFrame rf) {
          switch (rf)
          {
          case WORLD:
            self.appendCouplingConstraintInertias(
              model, data, cdata, diagonal_constraint_inertia, WorldFrameTag());
            break;
          case LOCAL:
            self.appendCouplingConstraintInertias(
              model, data, cdata, diagonal_constraint_inertia, LocalFrameTag());
            break;
          case LOCAL_WORLD_ALIGNED:
            self.appendCouplingConstraintInertias(
              model, data, cdata, diagonal_constraint_inertia, LocalWorldAlignedFrameTag());
            break;
          }
        },
        "model"_a, "data"_a, "constraint_data"_a, "diagonal_constraint_inertia"_a,
        "reference_frame"_a = LOCAL, "Append to data the apparent inertia due to the constraint.")
      .def(ComparableVisitor<ModelDerived>());

    if constexpr (has_baumgarte_corrector)
    {
      cl.def_prop_rw(
        "baumgarte_corrector_parameters",
        [](Self & self) -> BaumgarteCorrectorParameters & {
          return self.baumgarte_corrector_parameters();
        },
        [](Self & self, const BaumgarteCorrectorParameters & copy) {
          self.baumgarte_corrector_parameters() = copy;
        },
        "Baumgarte parameters associated with the constraint.");
    }
    if constexpr (has_set)
    {
      cl.def(
        "set",
        [](const Self & self, const DataDerived & cdata) -> ConstraintSet {
          return self.set(cdata);
        },
        "constraint_data"_a, "Constraint set.");
    }
  }
};

template<typename DataDerived>
struct ConstraintDataBaseVisitor : nanobind::def_visitor<ConstraintDataBaseVisitor<DataDerived>>
{
  static_assert(is_tpl_base_of_v<ConstraintDataBase, DataDerived>);

  using Self = DataDerived;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def_static("classname", Self::classname)
      .def("shortname", &Self::shortname, "Short name of the class.")
      .def(ComparableVisitor<DataDerived>());
  }
};

using BaumgarteCorrectorParameters = BaumgarteCorrectorParametersTpl<context::Scalar>;

PINOCCHIO_PYTHON_NAMESPACE_END

namespace nanobind::detail
{
  template<>
  struct type_caster<pinocchio::BlankConstraintModel> : none_caster<pinocchio::BlankConstraintModel>
  {
  };
  template<>
  struct type_caster<pinocchio::BlankConstraintData> : none_caster<pinocchio::BlankConstraintData>
  {
  };
} // namespace nanobind::detail
