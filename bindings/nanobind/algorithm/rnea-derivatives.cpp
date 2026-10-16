// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/model-checker.hpp"

#include "pinocchio/algorithm/rnea-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeRNEADerivatives(nb::module_ m)
{
  m.def(
    "computeGeneralizedGravityDerivatives",
    [](const Model & model, Data & data, ConstVectorRef q) -> Data::MatrixXs {
      Data::MatrixXs res = Data::MatrixXs::Zero(model.nv, model.nv);
      pinocchio::computeGeneralizedGravityDerivatives(model, data, q, res);
      return res;
    },
    "model"_a, "data"_a, "q"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the partial derivative of the generalized gravity contribution\n"
    "with respect to the joint configuration.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n\n"
    "Returns: dtau_statique_dq");

  m.def(
    "computeStaticTorqueDerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, const std::vector<Force> & fext)
      -> Data::MatrixXs {
      Data::MatrixXs res = Data::MatrixXs::Zero(model.nv, model.nv);
      pinocchio::computeStaticTorqueDerivatives(model, data, q, fext, res);
      return res;
    },
    "model"_a, "data"_a, "q"_a, "fext"_a,
    "Computes the partial derivative of the generalized gravity and external forces "
    "contributions (a.k.a static torque vector)\n"
    "with respect to the joint configuration.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tfext: list of external forces expressed in the local frame of the joints (size "
    "model.njoints)\n\n"
    "Returns: dtau_statique_dq");

  m.def(
    "computeRNEADerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a)
      -> nb::tuple {
      pinocchio::computeRNEADerivatives(model, data, q, v, a);
      make_symmetric(data.M);
      return nb::make_tuple(make_ref(data.dtau_dq), make_ref(data.dtau_dv), make_ref(data.M));
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the RNEA partial derivatives, store the result in data.dtau_dq, data.dtau_dv and "
    "data.M (aka dtau_da)\n"
    "which correspond to the partial derivatives of the torque output with respect to the "
    "joint configuration,\n"
    "velocity and acceleration vectors.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)\n\n"
    "Returns: (dtau_dq, dtau_dv, dtau_da)");

  m.def(
    "computeRNEADerivatives",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a,
      const std::vector<Force> & fext) -> nb::tuple {
      pinocchio::computeRNEADerivatives(model, data, q, v, a, fext);
      make_symmetric(data.M);
      return nb::make_tuple(make_ref(data.dtau_dq), make_ref(data.dtau_dv), make_ref(data.M));
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, "fext"_a,
    nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the RNEA partial derivatives with external contact forces,\n"
    "store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)\n"
    "which correspond to the partial derivatives of the torque output with respect to the "
    "joint configuration,\n"
    "velocity and acceleration vectors.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)\n"
    "\tfext: list of external forces expressed in the local frame of the joints (size "
    "model.njoints)\n\n"
    "Returns: (dtau_dq, dtau_dv, dtau_da)");
}
PINOCCHIO_PYTHON_NAMESPACE_END
