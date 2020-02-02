//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

namespace pinocchio
{
  namespace python
  {
   
    static const Eigen::VectorXd forwardDynamics_proxy(const Model & model,
                                                       Data & data,
                                                       const Eigen::VectorXd & q,
                                                       const Eigen::VectorXd & v,
                                                       const Eigen::VectorXd & tau,
                                                       const Eigen::MatrixXd & J,
                                                       const Eigen::VectorXd & gamma,
                                                       const double inv_damping = 0.0)
    {
      return forwardDynamics(model, data, q, v, tau, J, gamma, inv_damping);
    }
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(forwardDynamics_overloads, forwardDynamics_proxy, 7, 8)

    static const Eigen::VectorXd forwardDynamics_proxy_no_q(const Model & model,
                                                            Data & data,
                                                            const Eigen::VectorXd & tau,
                                                            const Eigen::MatrixXd & J,
                                                            const Eigen::VectorXd & gamma,
                                                            const double inv_damping = 0.0)
    {
      return forwardDynamics(model, data, tau, J, gamma, inv_damping);
    }
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(forwardDynamics_overloads_no_q, forwardDynamics_proxy_no_q, 5, 6)

    static const Eigen::VectorXd impulseDynamics_proxy(const Model & model,
                                                       Data & data,
                                                       const Eigen::VectorXd & q,
                                                       const Eigen::VectorXd & v_before,
                                                       const Eigen::MatrixXd & J,
                                                       const double r_coeff = 0.,
                                                       const double inv_damping = 0.)
    {
      return impulseDynamics(model, data, q, v_before, J, r_coeff, inv_damping);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(impulseDynamics_overloads, impulseDynamics_proxy, 5, 7)
    
    static const Eigen::VectorXd impulseDynamics_proxy_no_q(const Model & model,
                                                            Data & data,
                                                            const Eigen::VectorXd & v_before,
                                                            const Eigen::MatrixXd & J,
                                                            const double r_coeff = 0.,
                                                            const double inv_damping = 0.)
    {
      return impulseDynamics(model, data, v_before, J, r_coeff, inv_damping);
    }

    BOOST_PYTHON_FUNCTION_OVERLOADS(impulseDynamics_overloads_no_q, impulseDynamics_proxy_no_q, 4, 6)

    static const Eigen::MatrixXd getKKTContactDynamicMatrixInverse_proxy(const Model & model,
                                                                         Data & data,
                                                                         const Eigen::MatrixXd & J)
    {
      Eigen::MatrixXd MJtJ_inv(model.nv+J.rows(), model.nv+J.rows());
      getKKTContactDynamicMatrixInverse(model, data, J, MJtJ_inv);
      return MJtJ_inv;
    }
    

    void exposeConstrainedDynamics()
    {
      using namespace Eigen;
      
      bp::def("forwardDynamics",
              &forwardDynamics_proxy,
              forwardDynamics_overloads(
              bp::args("model","data","q","v","tau","constraint_jacobian","constraint_drift","damping"),
              "Solves the constrained dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
              "Note: internally, pinocchio.computeAllTerms is called."
              ));

      bp::def("forwardDynamics",
              &forwardDynamics_proxy_no_q,
              forwardDynamics_overloads_no_q(
              bp::args("model","data","tau","constraint_jacobian","constraint_drift","damping"),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c.\n"
              "Note: this function assumes that pinocchio.computeAllTerms has been called first."
              ));

      bp::def("impulseDynamics",
              &impulseDynamics_proxy,
              impulseDynamics_overloads(
              bp::args("model","data","q","v_before","constraint_jacobian","restitution_coefficient","damping"),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c.\n"
              "Note: internally, pinocchio.crba is called."
              ));
      
      bp::def("impulseDynamics",
              &impulseDynamics_proxy_no_q,
              impulseDynamics_overloads_no_q(
              bp::args("model","data","v_before","constraint_jacobian","restitution_coefficient","damping"),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c."
              "Note: this function assumes that pinocchio.crba has been called first."
              ));
      
      bp::def("getKKTContactDynamicMatrixInverse",
              getKKTContactDynamicMatrixInverse_proxy,
              bp::args("model","data","constraint_jacobian"),
              "Computes the inverse of the constraint matrix [[M Jt], [J 0]]. forwardDynamics or impulseDynamics must have been called first.\n"
              "Note: the constraint Jacobian should be the same that was provided to forwardDynamics or impulseDynamics.");
    }
    
  } // namespace python
} // namespace pinocchio
