// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeImpulseDynamicsDerivatives(nb::module_ m)
{
  m.def(
    "computeImpulseDynamicsDerivatives",
    [](
      const Model & model, Data & data, const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas, Scalar r_coeff,
      const ProximalSettings & prox_settings) {
      computeImpulseDynamicsDerivatives(
        model, data, contact_models, contact_datas, r_coeff, prox_settings);
    },
    "model"_a, "data"_a, "contact_models"_a, "contact_datas"_a, "r_coeff"_a = Scalar(0),
    "prox_settings"_a = ProximalSettings(),
    "Computes the impulse dynamics derivatives with contact constraints according to a "
    "given list of contact information.\n"
    "impulseDynamics should have been called before.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
