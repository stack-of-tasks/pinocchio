// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeImpulseDynamics(nb::module_ m)
{
  m.def(
    "impulseDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v,
      const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas,
      Scalar r_coeff, const ProximalSettings & prox_settings) {
      return make_const_ref(
        impulseDynamics(model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings));
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "contact_models"_a, "contact_datas"_a,
    "r_coeff"_a = Scalar(0), "prox_settings"_a = ProximalSettings(),
    "Computes the impulse dynamics with contact constraints according to a given list of "
    "contact information.\n"
    "When using impulseDynamics for the first time, you should call first "
    "initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
    "This function returns the after-impulse velocity of the system. The impulses acting "
    "on the contacts are stored in data.contact_forces.",
    nb::rv_policy::reference);
}

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
