//
// Copyright (c) 2020-2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

namespace pinocchio
{
  namespace python
  {
   
    static const context::VectorXs forwardDynamics_proxy(const context::Model & model,
                                                         context::Data & data,
                                                         const context::VectorXs & q,
                                                         const context::VectorXs & v,
                                                         const context::VectorXs & tau,
                                                         const context::MatrixXs & J,
                                                         const context::VectorXs & gamma,
                                                         const context::Scalar inv_damping = 0.0)
    {
      return forwardDynamics(model, data, q, v, tau, J, gamma, inv_damping);
    }
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(forwardDynamics_overloads, forwardDynamics_proxy, 7, 8)

    static const context::VectorXs forwardDynamics_proxy_no_q(const context::Model & model,
                                                              context::Data & data,
                                                              const context::VectorXs & tau,
                                                              const context::MatrixXs & J,
                                                              const context::VectorXs & gamma,
                                                              const context::Scalar inv_damping = 0.0)
    {
      return forwardDynamics(model, data, tau, J, gamma, inv_damping);
    }
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(forwardDynamics_overloads_no_q, forwardDynamics_proxy_no_q, 5, 6)

    static const context::VectorXs impulseDynamics_proxy(const context::Model & model,
                                                         context::Data & data,
                                                         const context::VectorXs & q,
                                                         const context::VectorXs & v_before,
                                                         const context::MatrixXs & J,
                                                         const context::Scalar r_coeff = 0.,
                                                         const context::Scalar inv_damping = 0.)
    {
      return impulseDynamics(model, data, q, v_before, J, r_coeff, inv_damping);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(impulseDynamics_overloads, impulseDynamics_proxy, 5, 7)
    
    static const context::VectorXs impulseDynamics_proxy_no_q(const context::Model & model,
                                                              context::Data & data,
                                                              const context::VectorXs & v_before,
                                                              const context::MatrixXs & J,
                                                              const context::Scalar r_coeff = 0.,
                                                              const context::Scalar inv_damping = 0.)
    {
      return impulseDynamics(model, data, v_before, J, r_coeff, inv_damping);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(impulseDynamics_overloads_no_q, impulseDynamics_proxy_no_q, 4, 6)

    static context::MatrixXs computeKKTContactDynamicMatrixInverse_proxy(const context::Model & model,
                                                                         context::Data & data,
                                                                         const context::VectorXs & q,
                                                                         const context::MatrixXs & J,
                                                                         const context::Scalar mu = 0)
    {
      context::MatrixXs KKTMatrix_inv(model.nv+J.rows(), model.nv+J.rows());
      computeKKTContactDynamicMatrixInverse(model, data, q, J, KKTMatrix_inv, mu);
      return KKTMatrix_inv;
    }
  
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeKKTContactDynamicMatrixInverse_overload,
                                    computeKKTContactDynamicMatrixInverse_proxy, 4, 5)

    static const context::MatrixXs getKKTContactDynamicMatrixInverse_proxy(const context::Model & model,
                                                                           context::Data & data,
                                                                           const context::MatrixXs & J)
    {
      context::MatrixXs MJtJ_inv(model.nv+J.rows(), model.nv+J.rows());
      getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
      return MJtJ_inv;
    }

    void exposeContactDynamics()
    {
      using namespace Eigen;
      
      bp::def("forwardDynamics",
              &forwardDynamics_proxy,
              forwardDynamics_overloads(
              bp::args("model","data","q","v","tau","constraint_jacobian","constraint_drift","damping"),
              "Solves the constrained dynamics problem with contacts, puts the result in context::Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
              "Note: internally, pinocchio.computeAllTerms is called."
              ));

      bp::def("forwardDynamics",
              &forwardDynamics_proxy_no_q,
              forwardDynamics_overloads_no_q(
              bp::args("model","data","tau","constraint_jacobian","constraint_drift","damping"),
              "Solves the forward dynamics problem with contacts, puts the result in context::Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
              "Note: this function assumes that pinocchio.computeAllTerms has been called first."
              ));

      bp::def("impulseDynamics",
              &impulseDynamics_proxy,
              impulseDynamics_overloads(
              bp::args("model","data","q","v_before","constraint_jacobian","restitution_coefficient","damping"),
              "Solves the impact dynamics problem with contacts, store the result in context::Data::dq_after and return it. The contact impulses are stored in data.impulse_c.\n"
              "Note: internally, pinocchio.crba is called."
              ));
      
      bp::def("impulseDynamics",
              &impulseDynamics_proxy_no_q,
              impulseDynamics_overloads_no_q(
              bp::args("model","data","v_before","constraint_jacobian","restitution_coefficient","damping"),
              "Solves the impact dynamics problem with contacts, store the result in context::Data::dq_after and return it. The contact impulses are stored in data.impulse_c.\n"
              "Note: this function assumes that pinocchio.crba has been called first."
              ));

      bp::def("computeKKTContactDynamicMatrixInverse",
              computeKKTContactDynamicMatrixInverse_proxy,
              computeKKTContactDynamicMatrixInverse_overload(bp::args("model","data","q","constraint_jacobian","damping"),
              "Computes the inverse of the constraint matrix [[M J^T], [J 0]]."));
      
      bp::def("getKKTContactDynamicMatrixInverse",
              getKKTContactDynamicMatrixInverse_proxy,
              bp::args("model","data","constraint_jacobian"),
              "Computes the inverse of the constraint matrix [[M Jt], [J 0]].\n forwardDynamics or impulseDynamics must have been called first.\n"
              "Note: the constraint Jacobian should be the same that was provided to forwardDynamics or impulseDynamics.");
    }
    
  } // namespace python
} // namespace pinocchio
