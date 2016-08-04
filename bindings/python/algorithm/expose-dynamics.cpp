//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/dynamics.hpp"

namespace se3
{
  namespace python
  {
    static Eigen::MatrixXd fd_llt_proxy(const ModelHandler & model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v,
                                        const VectorXd_fx & tau,
                                        const eigenpy::MatrixXd_fx & J,
                                        const VectorXd_fx & gamma,
                                        const bool update_kinematics = true)
    {
      forwardDynamics(*model,*data,q,v,tau,J,gamma,update_kinematics);
      return data->ddq;
    }
    
    static Eigen::MatrixXd id_llt_proxy(const ModelHandler & model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v_before,
                                        const eigenpy::MatrixXd_fx & J,
                                        const double r_coeff,
                                        const bool update_kinematics = true)
    {
      impulseDynamics(*model,*data,q,v_before,J,r_coeff,update_kinematics);
      return data->dq_after;
    }
    
    void exposeDynamics()
    {
      bp::def("forwardDynamics",fd_llt_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Contact drift gamma (size nb_constraint)",
                       "Update kinematics (if true, it updates the dynamic variable according to the current state)"),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c");
      
      bp::def("impactDynamics",id_llt_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity before impact v_before (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Coefficient of restitution r_coeff (0 = rigid impact; 1 = fully elastic impact.",
                       "Update kinematics (if true, it updates only the joint space inertia matrix)"),
              "Solve the impact dynamics problem with contacts, put the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c");
    }
    
  } // namespace python
} // namespace se3
