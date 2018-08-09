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
   
    void exposeDynamics()
    {
      using namespace Eigen;
      
      bp::def("forwardDynamics",
              (const VectorXd & (*)(const Model &, Data &,
                                    const VectorXd &, const VectorXd &, const VectorXd &,
                                    const MatrixXd &, const VectorXd &, const double, const bool))
              &forwardDynamics,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Joint torque tau (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Contact drift gamma (size nb_constraint)",
                       "(double) Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.",                       
                       "Update kinematics (if true, it updates the dynamic variable according to the current state )"),
              "Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("impactDynamics",
              (const VectorXd & (*)(const Model &, Data &,
                                    const VectorXd &, const VectorXd &,
                                    const MatrixXd &, const double, const bool))
              &impulseDynamics,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity before impact v_before (size Model::nv)",
                       "Contact Jacobian J (size nb_constraint * Model::nv)",
                       "Coefficient of restitution r_coeff (0 = rigid impact; 1 = fully elastic impact.",
                       "Update kinematics (if true, it updates only the joint space inertia matrix)"),
              "Solve the impact dynamics problem with contacts, put the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c",
              bp::return_value_policy<bp::return_by_value>());
    }
    
  } // namespace python
} // namespace se3
