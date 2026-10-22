// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/contact-dynamics.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using ConstMatrixRef = Eigen::Ref<const MatrixXs>;

void exposeContactDynamics(nb::module_ m)
{
  m.def(
    "forwardDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      ConstMatrixRef J, ConstVectorRef gamma, Scalar inv_damping) -> const VectorXs & {
      return forwardDynamics(model, data, q, v, tau, J, gamma, inv_damping);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "constraint_jacobian"_a, "constraint_drift"_a,
    "damping"_a = Scalar(0),
    "Solves the constrained dynamics problem with contacts, puts the result in data.ddq and "
    "returns it. The contact forces are stored in data.lambda_c.\n"
    "Note: internally, pinocchio.computeAllTerms is called.",
    nb::rv_policy::reference);

  m.def(
    "forwardDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef tau, ConstMatrixRef J, ConstVectorRef gamma,
      Scalar inv_damping) -> const VectorXs & {
      return forwardDynamics(model, data, tau, J, gamma, inv_damping);
    },
    "model"_a, "data"_a, "tau"_a, "constraint_jacobian"_a, "constraint_drift"_a,
    "damping"_a = Scalar(0),
    "Solves the forward dynamics problem with contacts, puts the result in data.ddq and "
    "returns it. The contact forces are stored in data.lambda_c.\n"
    "Note: this function assumes that pinocchio.computeAllTerms has been called first.",
    nb::rv_policy::reference);

  m.def(
    "impulseDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v_before, ConstMatrixRef J,
      Scalar r_coeff, Scalar inv_damping) -> const VectorXs & {
      return impulseDynamics(model, data, q, v_before, J, r_coeff, inv_damping);
    },
    "model"_a, "data"_a, "q"_a, "v_before"_a, "constraint_jacobian"_a,
    "restitution_coefficient"_a = Scalar(0), "damping"_a = Scalar(0),
    "Solves the impact dynamics problem with contacts, stores the result in data.dq_after and "
    "returns it. The contact impulses are stored in data.impulse_c.\n"
    "Note: internally, pinocchio.crba is called.",
    nb::rv_policy::reference);

  m.def(
    "impulseDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef v_before, ConstMatrixRef J, Scalar r_coeff,
      Scalar inv_damping) -> const VectorXs & {
      return impulseDynamics(model, data, v_before, J, r_coeff, inv_damping);
    },
    "model"_a, "data"_a, "v_before"_a, "constraint_jacobian"_a,
    "restitution_coefficient"_a = Scalar(0), "damping"_a = Scalar(0),
    "Solves the impact dynamics problem with contacts, stores the result in data.dq_after and "
    "returns it. The contact impulses are stored in data.impulse_c.\n"
    "Note: this function assumes that pinocchio.crba has been called first.",
    nb::rv_policy::reference);

  m.def(
    "computeKKTContactDynamicMatrixInverse",
    [](const Model & model, Data & data, ConstVectorRef q, ConstMatrixRef J, Scalar mu) {
      MatrixXs KKTMatrix_inv(model.nv + J.rows(), model.nv + J.rows());
      computeKKTContactDynamicMatrixInverse(model, data, q, J, KKTMatrix_inv, mu);
      return KKTMatrix_inv;
    },
    "model"_a, "data"_a, "q"_a, "constraint_jacobian"_a, "damping"_a = Scalar(0),
    "Computes the inverse of the constraint matrix [[M J^T], [J 0]].");

  m.def(
    "getKKTContactDynamicMatrixInverse",
    [](const Model & model, Data & data, ConstMatrixRef J) {
      MatrixXs MJtJ_inv(model.nv + J.rows(), model.nv + J.rows());
      getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
      return MJtJ_inv;
    },
    "model"_a, "data"_a, "constraint_jacobian"_a,
    "Computes the inverse of the constraint matrix [[M J^T], [J 0]].\n"
    "forwardDynamics or impulseDynamics must have been called first.\n"
    "Note: the constraint Jacobian should be the same that was provided to forwardDynamics or "
    "impulseDynamics.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
