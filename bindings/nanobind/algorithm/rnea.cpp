// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/rnea.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeRNEA(nb::module_ m)
{
  m.def(
    "rnea",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a)
      -> const Data::TangentVectorType & { return rnea(model, data, q, v, a); },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a,
    "Compute the RNEA, store the result in data.tau and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t a: the joint acceleration vector (size model.nv)\n");

  m.def(
    "rnea",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a,
      const std::vector<Force> & fext) -> const Data::TangentVectorType & {
      return rnea(model, data, q, v, a, fext);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, "fext"_a,
    "Compute the RNEA with external forces, store the result in data.tau and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t a: the joint acceleration vector (size model.nv)\n"
    "\t fext: list of external forces expressed in the local frame of the joints (size "
    "model.njoints)\n");

  m.def(
    "nonLinearEffects",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const Data::TangentVectorType & { return nonLinearEffects(model, data, q, v); },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), store "
    "the result in data.nle and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n");

  m.def(
    "computeGeneralizedGravity",
    [](const Model & model, Data & data, ConstVectorRef q) -> const Data::TangentVectorType & {
      return computeGeneralizedGravity(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store the "
    "result in data.g and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n");

  m.def(
    "computeStaticTorque",
    [](const Model & model, Data & data, ConstVectorRef q, const std::vector<Force> & fext)
      -> const Data::TangentVectorType & { return computeStaticTorque(model, data, q, fext); },
    "model"_a, "data"_a, "q"_a, "fext"_a,
    "Compute the generalized static torque contribution g(q) - J.T f_ext of the Lagrangian "
    "dynamics, store the result in data.tau and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t fext: list of external forces expressed in the local frame of the joints (size "
    "model.njoints)\n");

  m.def(
    "computeCoriolisMatrix",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const Data::MatrixXs & { return computeCoriolisMatrix(model, data, q, v); },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Compute the Coriolis matrix C(q,v) of the Lagrangian dynamics, store the result in "
    "data.C and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n");

  m.def(
    "getCoriolisMatrix",
    [](const Model & model, Data & data) -> const Data::MatrixXs & {
      return getCoriolisMatrix(model, data);
    },
    "model"_a, "data"_a,
    "Retrieves the Coriolis matrix C(q,v) of the Lagrangian dynamics after calling one of the "
    "derivative algorithms, store the result in data.C and return it.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
