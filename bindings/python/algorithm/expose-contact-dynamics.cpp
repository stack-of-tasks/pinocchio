//
// Copyright (c) 2015-2020 CNRS, INRIA
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

    static Eigen::MatrixXd computeKKTContactDynamicMatrixInverse_proxy(const Model & model,
                                                                       Data & data,
                                                                       const Eigen::VectorXd & q,
                                                                       const Eigen::MatrixXd & J,
                                                                       const double mu = 0)
    {
      Eigen::MatrixXd KKTMatrix_inv(model.nv+J.rows(), model.nv+J.rows());
      computeKKTContactDynamicMatrixInverse(model, data, q, J, KKTMatrix_inv, mu);
      return KKTMatrix_inv;
    }
  
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeKKTContactDynamicMatrixInverse_overload,
                                    computeKKTContactDynamicMatrixInverse_proxy, 4, 5)

    static const Eigen::MatrixXd getKKTContactDynamicMatrixInverse_proxy(const Model & model,
                                                                         Data & data,
                                                                         const Eigen::MatrixXd & J)
    {
      Eigen::MatrixXd KKTMatrix_inv(model.nv+J.rows(), model.nv+J.rows());
      getKKTContactDynamicMatrixInverse(model, data, J, KKTMatrix_inv);
      return KKTMatrix_inv;
    }

    void exposeDynamics()
    {
      using namespace Eigen;
      
      bp::def("forwardDynamics",
              &forwardDynamics_proxy,
              forwardDynamics_overloads(
              bp::args("Model","Data","q","v","tau","J","gamma","damping"),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c."
              " Internally, pinocchio.computeAllTerms is called."
              ));

      bp::def("forwardDynamics",
              &forwardDynamics_proxy_no_q,
              forwardDynamics_overloads_no_q(
              bp::args("Model","Data","tau","J","gamma","damping"),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c."
              " Assumes pinocchio.computeAllTerms has been called."
              ));

      bp::def("impulseDynamics",
              &impulseDynamics_proxy,
              impulseDynamics_overloads(
              bp::args("Model","Data","q","v_before","J","r_coeff","damping"),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c."
              " Internally, pinocchio.crba is called."
              ));
      
      bp::def("impulseDynamics",
              &impulseDynamics_proxy_no_q,
              impulseDynamics_overloads_no_q(
              bp::args("Model","Data","v_before","J","r_coeff","damping"),
              "Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c."
              " Assumes pinocchio.crba has been called."
              ));
      
      bp::def("computeKKTContactDynamicMatrixInverse",
              computeKKTContactDynamicMatrixInverse_proxy,
              computeKKTContactDynamicMatrixInverse_overload(bp::args("model","data","q","J","damping"),
              "Computes the inverse of the constraint matrix [[M J^T], [J 0]]."));
      
      bp::def("getKKTContactDynamicMatrixInverse",
              getKKTContactDynamicMatrixInverse_proxy,
              bp::args("Model","Data","J"),
              "Computes the inverse of the constraint matrix [[M JT], [J 0]]. forward/impulseDynamics must be called first. The jacobian should be the same that was provided to forward/impulseDynamics.");
    }
    
  } // namespace python
} // namespace pinocchio
