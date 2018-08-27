//
// Copyright (c) 2016-2018 CNRS
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
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"

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
  ///  By default, the constraint Jacobian is assumed to be full rank, and undamped Cholesky inverse is performed.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] gamma The drift of the constraints (dim nb_constraints).
  /// \param[in] inv_damping Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank.    
  /// \param[in] updateKinematics If true, the algorithm calls first se3::computeAllTerms. Otherwise, it uses the current dynamic values stored in data. \\
  ///            \note A hint: 1e-12 as the damping factor gave good result in the particular case of redundancy in contact constraints on the two feet.
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
                                                 const double inv_damping = 0.,
                                                 const bool updateKinematics = true
                                                 )
  {
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(J.cols() == model.nv);
    assert(J.rows() == gamma.size());
    assert(model.check(data) && "data is not consistent with model.");
    
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

    data.JMinvJt.diagonal().array() += inv_damping;
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
  
  ///
  /// \brief Compute the impulse dynamics with contact constraints.
  /// \note It computes the following problem: <BR>
  ///       <CENTER> \f$ \begin{eqnarray} \underset{\dot{q}^{+}}{\min} & & \| \dot{q}^{+} - \dot{q}^{-} \|_{M(q)} \\
  ///           \text{s.t.} & & J (q) \dot{q}^{+} = - \epsilon J (q) \dot{q}^{-}  \end{eqnarray} \f$ </CENTER> <BR>
  ///       where \f$ \dot{q}^{-} \f$ is the generalized velocity before impact,
  ///       \f$ M \f$ is the joint space mass matrix, \f$ J \f$ the constraint Jacobian and \f$ \epsilon \f$ is the coefficient of restitution (1 for a fully elastic impact or 0 for a rigid impact).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v_before The joint velocity before impact (vector dim model.nv).
  /// \param[in] J The Jacobian of the constraints (dim nb_constraints*model.nv).
  /// \param[in] r_coeff The coefficient of restitution. Must be in [0;1].
  /// \param[in] updateKinematics If true, the algorithm calls first se3::crba. Otherwise, it uses the current mass matrix value stored in data.
  ///
  /// \return A reference to the generalized velocity after impact stored in data.dq_after. The Lagrange Multipliers linked to the contact impulsed are available throw data.impulse_c vector.
  ///
  inline const Eigen::VectorXd & impulseDynamics(const Model & model,
                                                 Data & data,
                                                 const Eigen::VectorXd & q,
                                                 const Eigen::VectorXd & v_before,
                                                 const Eigen::MatrixXd & J,
                                                 const double r_coeff = 0.,
                                                 const bool updateKinematics = true
                                                 )
  {
    assert(q.size() == model.nq);
    assert(v_before.size() == model.nv);
    assert(J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    Eigen::VectorXd & impulse_c = data.impulse_c;
    Eigen::VectorXd & dq_after = data.dq_after;
    
    // Compute the mass matrix
    if (updateKinematics)
      crba(model, data, q);
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(int k=0;k<model.nv;++k) data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    // Compute the Lagrange Multipliers related to the contact impulses
    impulse_c = (-r_coeff - 1.) * (J * v_before);
    data.llt_JMinvJt.solveInPlace (impulse_c);
    
    // Compute the joint velocity after impacts
    dq_after = J.transpose() * impulse_c;
    cholesky::solve (model, data, dq_after);
    dq_after += v_before;
    
    return dq_after;
  }
} // namespace se3


#endif // ifndef __se3_dynamics_hpp__
