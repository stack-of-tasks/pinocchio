// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/centroidal.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeCentroidal(nb::module_ m)
{
  m.def(
    "computeCentroidalMomentum",
    [](const Model & model, Data & data) -> const Force & {
      return pinocchio::computeCentroidalMomentum(model, data);
    },
    "model"_a, "data"_a, nb::rv_policy::reference_internal,
    "Computes the Centroidal momentum, a.k.a. the total momentum of the system expressed "
    "around the center of mass.");

  m.def(
    "computeCentroidalMomentum",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) -> const Force & {
      return pinocchio::computeCentroidalMomentum(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, nb::rv_policy::reference_internal,
    "Computes the Centroidal momentum, a.k.a. the total momentum of the system expressed "
    "around the center of mass.");

  m.def(
    "computeCentroidalMomentumTimeVariation",
    [](const Model & model, Data & data) -> const Force & {
      return pinocchio::computeCentroidalMomentumTimeVariation(model, data);
    },
    "model"_a, "data"_a, nb::rv_policy::reference_internal,
    "Computes the Centroidal momentum and its time derivatives, a.k.a. the total momentum of "
    "the system and its time derivative expressed around the center of mass.");

  m.def(
    "computeCentroidalMomentumTimeVariation",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a)
      -> const Force & {
      return pinocchio::computeCentroidalMomentumTimeVariation(model, data, q, v, a);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, nb::rv_policy::reference_internal,
    "Computes the Centroidal momentum and its time derivatives, a.k.a. the total momentum of "
    "the system and its time derivative expressed around the center of mass.");

  m.def(
    "ccrba",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const Data::Matrix6x & { return pinocchio::ccrba(model, data, q, v); },
    "model"_a, "data"_a, "q"_a, "v"_a, nb::rv_policy::reference_internal,
    "Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite "
    "Rigid Body Inertia, puts the result in data and returns the centroidal mapping.\n"
    "For the same price, it also computes the total joint jacobians (data.J).");

  m.def(
    "computeCentroidalMap",
    [](const Model & model, Data & data, ConstVectorRef q) -> const Data::Matrix6x & {
      return pinocchio::computeCentroidalMap(model, data, q);
    },
    "model"_a, "data"_a, "q"_a, nb::rv_policy::reference_internal,
    "Computes the centroidal mapping, puts the result in data.Ag and returns the centroidal "
    "mapping.\n"
    "For the same price, it also computes the total joint jacobians (data.J).");

  m.def(
    "dccrba",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const Data::Matrix6x & { return pinocchio::dccrba(model, data, q, v); },
    "model"_a, "data"_a, "q"_a, "v"_a, nb::rv_policy::reference_internal,
    "Computes the time derivative of the centroidal momentum matrix Ag in terms of q and v.\n"
    "For the same price, it also computes the centroidal momentum matrix (data.Ag), the total "
    "joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ).");

  m.def(
    "computeCentroidalMapTimeVariation",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const Data::Matrix6x & {
      return pinocchio::computeCentroidalMapTimeVariation(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, nb::rv_policy::reference_internal,
    "Computes the time derivative of the centroidal momentum matrix Ag, puts the result in "
    "data.Ag and returns the centroidal mapping.\n"
    "For the same price, it also computes the centroidal momentum matrix (data.Ag), the total "
    "joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ).");
}
PINOCCHIO_PYTHON_NAMESPACE_END
