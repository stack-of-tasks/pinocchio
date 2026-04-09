// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/model-checker.hpp"

#include "pinocchio/algorithm/aba-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeABADerivatives(nb::module_ m)
{
  m.def(
    "computeABADerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau)
      -> nb::tuple {
      computeABADerivatives(model, data, q, v, tau);
      make_symmetric(data.Minv);
      return nb::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv "
    "(aka ddq_dtau)\n"
    "which correspond to the partial derivatives of the joint acceleration vector output with "
    "respect to the joint configuration,\n"
    "velocity and torque vectors.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t tau: the joint torque vector (size model.nv)\n\n"
    "Returns: (ddq_dq, ddq_dv, ddq_dtau)");

  m.def(
    "computeABADerivatives",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      const std::vector<Force> & fext) -> nb::tuple {
      computeABADerivatives(model, data, q, v, tau, fext);
      make_symmetric(data.Minv);
      return nb::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "fext"_a,
    nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the ABA derivatives with external contact forces,\n"
    "store the result in data.ddq_dq, data.ddq_dv and data.Minv (aka ddq_dtau)\n"
    "which correspond to the partial derivatives of the acceleration output with respect to "
    "the joint configuration,\n"
    "velocity and torque vectors.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t data: data related to the model\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t tau: the joint torque vector (size model.nv)\n"
    "\t fext: list of external forces expressed in the local frame of the joints (size "
    "model.njoints)\n\n"
    "Returns: (ddq_dq, ddq_dv, ddq_dtau)");

  m.def(
    "computeABADerivatives",
    [](const Model & model, Data & data) -> nb::tuple {
      computeABADerivatives(model, data);
      make_symmetric(data.Minv);
      return nb::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    },
    "model"_a, "data"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
    "which correspond to the partial derivatives of the joint acceleration vector output with "
    "respect to the joint configuration,\n"
    "velocity and torque vectors.\n"
    "By calling this function, the user assumes that pinocchio.aba has been called first, "
    "allowing to significantly reduce the computation timings by not recalculating "
    "intermediate results.\n\n"
    "Returns: (ddq_dq, ddq_dv, ddq_dtau)");

  m.def(
    "computeABADerivatives",
    [](const Model & model, Data & data, const std::vector<Force> & fext) -> nb::tuple {
      computeABADerivatives(model, data, fext);
      make_symmetric(data.Minv);
      return nb::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    },
    "model"_a, "data"_a, "fext"_a, nb::call_policy<mimic_not_supported_policy<0>>(),
    "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
    "which correspond to the partial derivatives of the joint acceleration vector output with "
    "respect to the joint configuration,\n"
    "velocity and torque vectors.\n"
    "By calling this function, the user assumes that pinocchio.aba has been called first, "
    "allowing to significantly reduce the computation timings by not recalculating "
    "intermediate results.\n\n"
    "Returns: (ddq_dq, ddq_dv, ddq_dtau)");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
