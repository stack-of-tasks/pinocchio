//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_dynamics_hpp__
#define __se3_dynamics_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/simulation/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"

#include <Eigen/Cholesky>
namespace se3
{
  
  ///
  /// \brief Compute the forward dynamics with contact constraints.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\ddot{q}}{\min} & & \| \ddot{q} - \ddot{q}_{\text{free}} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \ddot{q} + \gamma (q, \dot{q}) = 0 \end{eqnarray} \f$ </CENTER> <BR>
  ///       where \f$ \ddot{q}_{\text{free}} \f$ is the free acceleration (i.e. without constraints),
  ///       \f$ M \f$ is the mass matrix, \f$ J \f$ the constraint Jacobian and \f$ \gamma \f$ is the constraint drift.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] gamma The drift of the constraints (dim nb_constraints).
  /// \param[in] updateKinematics If true, the algorithm calls first se3::computeAllTerms. Otherwise, it uses the current dynamic values stored in data.
  ///
  /// \return A reference to the joint acceleration stored in data.ddq. The Lagrange Multipliers linked to the contact forces are available throw data.lambda_c vector.
  ///
  inline const Eigen::VectorXd & forwardDynamics(const Model & model,
                                                 Data & data,
                                                 const Eigen::VectorXd & q,
                                                 const Eigen::VectorXd & v,
                                                 const Eigen::VectorXd & tau,
                                                 const Eigen::MatrixXd & J,
                                                 const Eigen::VectorXd & gamma,
                                                 const bool updateKinematics = true
                                                 )
  {
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(J.cols() == model.nv);
    assert(J.rows() == gamma.size());
    
    Eigen::VectorXd & a = data.ddq;
    Eigen::VectorXd & lambda_c = data.lambda_c;
    
    if (updateKinematics)
      computeAllTerms(model, data, q, v);
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    // Compute the dynamic drift (control - nle)
    data.torque_residual = tau - data.nle;
    cholesky::solve(model, data, data.torque_residual);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(int k=0;k<model.nv;++k) data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    // Compute the Lagrange Multipliers
    lambda_c = -gamma -J*data.torque_residual;
    data.llt_JMinvJt.solveInPlace (lambda_c);
    
    // Compute the joint acceleration
    a = J.transpose() * lambda_c;
    cholesky::solve (model, data, a);
    a += data.torque_residual;
    
    return a;
  }
} // namespace se3


#endif // ifndef __se3_dynamics_hpp__
