// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/model-checker.hpp"

#include "pinocchio/algorithm/loop-constrained-aba.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

template<class ConstraintModel>
void exposeLcabaFor(nb::module_ m)
{
  using ConstraintData = typename ConstraintModel::ConstraintData;
  using ConstraintModelAllocator = std::allocator<ConstraintModel>;

  m.def(
    "computeJointMinimalOrdering",
    &computeJointMinimalOrdering<
      Scalar, Options, JointCollectionDefaultTpl, ConstraintModel, ConstraintModelAllocator>,
    "model"_a, "data"_a, "constraint_models"_a,
    "Computes the joint minimal ordering for closed-loop constrained dynamics.\n"
    "This function must be called before lcaba.\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree\n"
    "\t constraint_models: vector of constraint models",
    nb::call_policy<mimic_not_supported_policy<0>>());

  m.def(
    "lcaba",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef tau,
      const std::vector<ConstraintModel> & constraint_models,
      std::vector<ConstraintData> & constraint_datas, ProximalSettings & prox_settings) {
      return lcaba(model, data, q, v, tau, constraint_models, constraint_datas, prox_settings);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "tau"_a, "constraint_models"_a, "constraint_datas"_a,
    "prox_settings"_a,
    "Computes the forward dynamics with closed-loop contact constraints using the\n"
    "Loop-Constrained Articulated Body Algorithm (LCABA).\n"
    "Before calling lcaba for the first time, you should call computeJointMinimalOrdering\n"
    "to set up the joint elimination order.\n\n"
    "Parameters:\n"
    "\t model: Model of the kinematic tree\n"
    "\t data: Data related to the kinematic tree\n"
    "\t q: joint configuration (size model.nq)\n"
    "\t v: joint velocity (size model.nv)\n"
    "\t tau: joint torque (size model.nv)\n"
    "\t constraint_models: vector of constraint models\n"
    "\t constraint_datas: vector of constraint data\n"
    "\t prox_settings: Proximal settings (mu, accuracy and maximal number of iterations)\n\n"
    "Note: A typical value for mu in proximal settings is 1e-6, and it must be positive.\n"
    "This function returns joint acceleration stored in data.ddq.\n"
    "The constraint forces are stored in data.lambdaA[0].",
    nb::call_policy<mimic_not_supported_policy<0>>());
}

void exposeLcaba(nb::module_ m)
{
  exposeLcabaFor<RigidConstraintModel>(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
