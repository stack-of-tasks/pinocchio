//
// Copyright (c) 2020-2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace pinocchio
{
  namespace python
  {

    static const context::VectorXs forwardDynamics_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & tau,
      const context::MatrixXs & J,
      const context::VectorXs & gamma,
      const context::Scalar inv_damping = context::Scalar(0.0))
    {

      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      return forwardDynamics(model, data, q, v, tau, J, gamma, inv_damping);
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP
    }

    static const context::VectorXs forwardDynamics_proxy_no_q(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & tau,
      const context::MatrixXs & J,
      const context::VectorXs & gamma,
      const context::Scalar inv_damping = context::Scalar(0.0))
    {

      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      return forwardDynamics(model, data, tau, J, gamma, inv_damping);
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP
    }

    static const context::VectorXs impulseDynamics_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v_before,
      const context::MatrixXs & J,
      const context::Scalar r_coeff = context::Scalar(0.),
      const context::Scalar inv_damping = context::Scalar(0.))
    {

      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      return impulseDynamics(model, data, q, v_before, J, r_coeff, inv_damping);
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP
    }

    static const context::VectorXs impulseDynamics_proxy_no_q(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & v_before,
      const context::MatrixXs & J,
      const context::Scalar r_coeff = context::Scalar(0.),
      const context::Scalar inv_damping = context::Scalar(0.))
    {

      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      return impulseDynamics(model, data, v_before, J, r_coeff, inv_damping);
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP
    }

    static context::MatrixXs computeKKTContactDynamicMatrixInverse_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::MatrixXs & J,
      const context::Scalar mu = context::Scalar(0))
    {
      context::MatrixXs KKTMatrix_inv(model.nv + J.rows(), model.nv + J.rows());
      computeKKTContactDynamicMatrixInverse(model, data, q, J, KKTMatrix_inv, mu);
      return KKTMatrix_inv;
    }

    static const context::MatrixXs getKKTContactDynamicMatrixInverse_proxy(
      const context::Model & model, context::Data & data, const context::MatrixXs & J)
    {
      context::MatrixXs MJtJ_inv(model.nv + J.rows(), model.nv + J.rows());

      PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
      PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
      getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
      PINOCCHIO_COMPILER_DIAGNOSTIC_POP

      return MJtJ_inv;
    }

    void exposeContactDynamics()
    {
      using namespace Eigen;

      bp::def(
        "forwardDynamics", &forwardDynamics_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"), bp::arg("tau"),
         bp::arg("constraint_jacobian"), bp::arg("constraint_drift"), bp::arg("damping") = 0),
        "Solves the constrained dynamics problem with contacts, puts the result in "
        "context::Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
        "Note: internally, pinocchio.computeAllTerms is called.",
        mimic_not_supported_function<>(0));

      bp::def(
        "forwardDynamics", &forwardDynamics_proxy_no_q,
        (bp::arg("model"), bp::arg("data"), bp::arg("tau"), bp::arg("constraint_jacobian"),
         bp::arg("constraint_drift"), bp::arg("damping") = 0),
        "Solves the forward dynamics problem with contacts, puts the result in "
        "context::Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
        "Note: this function assumes that pinocchio.computeAllTerms has been called first.",
        mimic_not_supported_function<>(0));

      bp::def(
        "impulseDynamics", &impulseDynamics_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v_before"),
         bp::arg("constraint_jacobian"), bp::arg("restitution_coefficient") = 0,
         bp::arg("damping") = 0),
        "Solves the impact dynamics problem with contacts, store the result in "
        "context::Data::dq_after and return it. The contact impulses are stored in "
        "data.impulse_c.\n"
        "Note: internally, pinocchio.crba is called.",
        mimic_not_supported_function<>(0));

      bp::def(
        "impulseDynamics", &impulseDynamics_proxy_no_q,
        (bp::arg("model"), bp::arg("data"), bp::arg("v_before"), bp::arg("constraint_jacobian"),
         bp::arg("restitution_coefficient") = 0, bp::arg("damping") = 0),
        "Solves the impact dynamics problem with contacts, store the result in "
        "context::Data::dq_after and return it. The contact impulses are stored in "
        "data.impulse_c.\n"
        "Note: this function assumes that pinocchio.crba has been called first.",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeKKTContactDynamicMatrixInverse", computeKKTContactDynamicMatrixInverse_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("constraint_jacobian"),
         bp::arg("damping") = 0),
        "Computes the inverse of the constraint matrix [[M J^T], [J 0]].",
        mimic_not_supported_function<>(0));

      bp::def(
        "getKKTContactDynamicMatrixInverse", getKKTContactDynamicMatrixInverse_proxy,
        bp::args("model", "data", "constraint_jacobian"),
        "Computes the inverse of the constraint matrix [[M Jt], [J 0]].\n forwardDynamics or "
        "impulseDynamics must have been called first.\n"
        "Note: the constraint Jacobian should be the same that was provided to "
        "forwardDynamics or impulseDynamics.",
        mimic_not_supported_function<>(0));
    }

  } // namespace python
} // namespace pinocchio
