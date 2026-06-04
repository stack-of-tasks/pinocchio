// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeConstraintDynamicsDerivatives(nb::module_ m)
{
  m.def(
    "computeConstraintDynamicsDerivatives",
    [](
      const Model & model, Data & data, const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas, const ProximalSettings & settings) {
      computeConstraintDynamicsDerivatives(model, data, contact_models, contact_datas, settings);

      return nb::make_tuple(
        make_ref(data.ddq_dq),     //
        make_ref(data.ddq_dv),     //
        make_ref(data.ddq_dtau),   //
        make_ref(data.dlambda_dq), //
        make_ref(data.dlambda_dv), //
        make_ref(data.dlambda_dtau));
    },
    "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a, "settings"_a = ProximalSettings(),
    "Computes the derivatives of the forward dynamics with kinematic constraints (given in the "
    "list of constraint models).\n"
    "Assumes that constraintDynamics has been called first. See constraintDynamics for more "
    "details.\n"
    "This function returns the derivatives of joint acceleration (ddq) and contact forces "
    "(lambda_c) of the system with respect to q, v and tau.\n"
    "The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
