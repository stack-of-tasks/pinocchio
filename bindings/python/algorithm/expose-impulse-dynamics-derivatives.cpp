//
// Copyright (c) 2020-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/proximal.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {

    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel)
      RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData)
      RigidConstraintDataVector;

    static void impulseDynamicsDerivatives_proxy(
      const context::Model & model,
      context::Data & data,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas,
      const context::Scalar & r_coeff,
      const context::ProximalSettings & prox_settings)
    {
      computeImpulseDynamicsDerivatives(
        model, data, contact_models, contact_datas, r_coeff, prox_settings);
      return;
    }

    void exposeImpulseDynamicsDerivatives()
    {
      bp::def(
        "computeImpulseDynamicsDerivatives", impulseDynamicsDerivatives_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("contact_models"), bp::arg("contact_datas"),
         bp::arg("r_coeff") = 0, bp::arg("prox_settings") = context::ProximalSettings()),
        "Computes the impulse dynamics derivatives with contact constraints according to a "
        "given list of Contact information.\n"
        "impulseDynamics should have been called before.");
    }
  } // namespace python
} // namespace pinocchio
