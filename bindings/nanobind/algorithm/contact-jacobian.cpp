// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/contact-jacobian.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeContactJacobian(nb::module_ m)
{
  m.def(
    "getConstraintJacobian",
    [](
      const Model & model, const Data & data, const RigidConstraintModel & constraint_model,
      const RigidConstraintData & constraint_data) {
      MatrixXs J(constraint_model.residualSize(), model.nv);
      J.setZero();
      getConstraintJacobian(model, data, constraint_model, constraint_data, J);
      return J;
    },
    "model"_a, "data"_a, "constraint_model"_a, "constraint_data"_a,
    "Computes the kinematic Jacobian associated with a given constraint model.");

  m.def(
    "getConstraintsJacobian",
    [](
      const Model & model, const Data & data, const RigidConstraintModelVector & constraint_models,
      const RigidConstraintDataVector & constraint_datas) {
      MatrixXs J(getTotalConstraintResidualSize(constraint_models), model.nv);
      J.setZero();
      getConstraintsJacobian(model, data, constraint_models, constraint_datas, J);
      return J;
    },
    "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a,
    "Computes the kinematic Jacobian associated with a given set of constraint models.");

  m.def(
    "getConstraintJacobian",
    [](
      const Model & model, const Data & data, const ConstraintModel & constraint_model,
      const ConstraintData & constraint_data) {
      MatrixXs J(constraint_model.residualSize(), model.nv);
      J.setZero();
      getConstraintJacobian(model, data, constraint_model, constraint_data, J);
      return J;
    },
    "model"_a, "data"_a, "constraint_model"_a, "constraint_data"_a,
    "Computes the kinematic Jacobian associated with a given constraint model.");

  m.def(
    "getConstraintsJacobian",
    [](
      const Model & model, const Data & data, const ConstraintModelVector & constraint_models,
      const ConstraintDataVector & constraint_datas) {
      MatrixXs J(getTotalConstraintResidualSize(constraint_models), model.nv);
      J.setZero();
      getConstraintsJacobian(model, data, constraint_models, constraint_datas, J);
      return J;
    },
    "model"_a, "data"_a, "constraint_models"_a, "constraint_datas"_a,
    "Computes the kinematic Jacobian associated with a given set of constraint models.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
