//
// Copyright (c) 2015-2019 CNRS, INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

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
    

    void exposeDynamics()
    {
      using namespace Eigen;
      
      bp::def("forwardDynamics",
              &forwardDynamics_proxy,
              forwardDynamics_overloads(
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Contact drift gamma (size nb_constraint)",
                       "(double) Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank."),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c."
              " Internally, pinocchio.computeAllTerms is called."
              ));

      bp::def("forwardDynamics",
              &forwardDynamics_proxy_no_q,
              forwardDynamics_overloads_no_q(
              bp::args("Model","Data",
                       "Joint torque tau (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Contact drift gamma (size nb_constraint)",
                       "(double) Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank."),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c."
              " Assumes pinocchio.computeAllTerms has been called."
              ));

      bp::def("impulseDynamics",
              &impulseDynamics_proxy,
              impulseDynamics_overloads(
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity before impact v_before (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Coefficient of restitution r_coeff (0 = rigid impact; 1 = fully elastic impact)",
                       "Damping factor when J is rank deficient."
                       ),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c."
              " Internally, pinocchio.crba is called."
              ));
      
      bp::def("impulseDynamics",
              &impulseDynamics_proxy_no_q,
              impulseDynamics_overloads_no_q(
              bp::args("Model","Data",
                       "Joint velocity before impact v_before (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Coefficient of restitution r_coeff (0 = rigid impact; 1 = fully elastic impact)",
                       "Damping factor when J is rank deficient."),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c."
              " Assumes pinocchio.crba has been called."
              ));
      
      bp::def("getKKTContactDynamicMatrixInverse",getKKTContactDynamicMatrixInverse_proxy,
              bp::args("Model","Data",
                       "Contact Jacobian J(size nb_constraint * Model::nv)"),
              "Computes the inverse of the constraint matrix [[M JT], [J 0]]. forward/impulseDynamics must be called first. The jacobian should be the same that was provided to forward/impulseDynamics.");
    }
    
  } // namespace python
} // namespace pinocchio
