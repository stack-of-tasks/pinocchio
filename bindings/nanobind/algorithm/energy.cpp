// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/energy.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeEnergy(nb::module_ m)
{
  m.def(
    "computeKineticEnergy",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) -> Scalar {
      return computeKineticEnergy(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Computes the forward kinematics and the kinetic energy of the system for the given joint "
    "configuration and velocity. The result is accessible through data.kinetic_energy.");

  m.def(
    "computeKineticEnergy",
    [](const Model & model, Data & data) -> Scalar { return computeKineticEnergy(model, data); },
    "model"_a, "data"_a,
    "Computes the kinetic energy of the system for the given joint placement and velocity "
    "stored in data. The result is accessible through data.kinetic_energy.");

  m.def(
    "computePotentialEnergy",
    [](const Model & model, Data & data, ConstVectorRef q) -> Scalar {
      return computePotentialEnergy(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Computes the potential energy of the system for the given joint configuration. "
    "The result is accessible through data.potential_energy.");

  m.def(
    "computePotentialEnergy",
    [](const Model & model, Data & data) -> Scalar { return computePotentialEnergy(model, data); },
    "model"_a, "data"_a,
    "Computes the potential energy of the system for the given joint placement stored in data. "
    "The result is accessible through data.potential_energy.");

  m.def(
    "computeMechanicalEnergy",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) -> Scalar {
      return computeMechanicalEnergy(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Computes the forward kinematics and the mechanical energy of the system for the given "
    "joint configuration and velocity. The result is accessible through "
    "data.mechanical_energy.\n"
    "A byproduct of this function is the computation of both data.kinetic_energy and "
    "data.potential_energy too.");

  m.def(
    "computeMechanicalEnergy",
    [](const Model & model, Data & data) -> Scalar { return computeMechanicalEnergy(model, data); },
    "model"_a, "data"_a,
    "Computes the mechanical energy of the system for the given joint placement and velocity "
    "stored in data. The result is accessible through data.mechanical_energy.\n"
    "A byproduct of this function is the computation of both data.kinetic_energy and "
    "data.potential_energy too.");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
