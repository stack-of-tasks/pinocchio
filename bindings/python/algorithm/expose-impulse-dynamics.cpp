//
// Copyright (c) 2020-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include "pinocchio/algorithm/impulse-dynamics.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {

#ifndef PINOCCHIO_PYTHON_SKIP_ALGORITHM_IMPULSE_DYNAMICS
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel)
      RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData)
      RigidConstraintDataVector;

    static const context::VectorXs impulseDynamics_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas,
      const context::Scalar r_coeff,
      const context::ProximalSettings & prox_settings)
    {
      return impulseDynamics(
        model, data, q, v, contact_models, contact_datas, r_coeff, prox_settings);
    }
#endif // PINOCCHIO_PYTHON_SKIP_ALGORITHM_IMPULSE_DYNAMICS

    void exposeImpulseDynamics()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_ALGORITHM_IMPULSE_DYNAMICS
      bp::def(
        "impulseDynamics", impulseDynamics_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"), bp::arg("contact_models"),
         bp::arg("contact_datas"), bp::arg("r_coeff") = 0,
         bp::arg("prox_settings") = context::ProximalSettings()),
        "Computes the impulse dynamics with contact constraints according to a given list of "
        "Contact information.\n"
        "When using impulseDynamics for the first time, you should call first "
        "initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
        "This function returns the after-impulse velocity of the system. The impulses acting "
        "on the contacts are stored in the list data.contact_forces.",
        mimic_not_supported_function<>(0));
#endif // PINOCCHIO_PYTHON_SKIP_ALGORITHM_IMPULSE_DYNAMICS
    }
  } // namespace python
} // namespace pinocchio
