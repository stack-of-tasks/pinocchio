//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel)
      RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData)
      RigidConstraintDataVector;

    bp::tuple computeConstraintDynamicsDerivatives_proxy(
      const context::Model & model,
      context::Data & data,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas,
      const context::ProximalSettings & settings = context::ProximalSettings())
    {
      pinocchio::computeConstraintDynamicsDerivatives(
        model, data, contact_models, contact_datas,
        const_cast<context::ProximalSettings &>(settings));

      return bp::make_tuple(
        make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.ddq_dtau),
        make_ref(data.dlambda_dq), make_ref(data.dlambda_dv), make_ref(data.dlambda_dtau));
    }

    void exposeConstraintDynamicsDerivatives()
    {
      using namespace Eigen;

      bp::def(
        "computeConstraintDynamicsDerivatives", computeConstraintDynamicsDerivatives_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("contact_models"), bp::arg("contact_datas"),
         bp::arg("settings") = context::ProximalSettings()),
        "Computes the derivatives of the forward dynamics with kinematic constraints (given in the "
        "list of constraint models).\n"
        "Assumes that constraintDynamics has been called first. See constraintDynamics for more "
        "details.\n"
        "This function returns the derivatives of joint acceleration (ddq) and contact forces "
        "(lambda_c) of the system with respect to q, v and tau.\n"
        "The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da.",
        mimic_not_supported_function<>(0));
    }
  } // namespace python
} // namespace pinocchio
