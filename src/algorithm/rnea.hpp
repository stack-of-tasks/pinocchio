//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_rnea_hpp__
#define __se3_rnea_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
  
namespace se3
{
  ///
  /// \brief The Recursive Newton-Euler algorithm. It computes the inverse dynamics, aka the joint torques according to the current state of the system and the desired joint accelerations.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  ///
  /// \return The desired joint torques stored in data.tau.
  ///
  inline const Eigen::VectorXd &
  rnea(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a);
  
  ///
  /// \brief The Recursive Newton-Euler algorithm. It computes the inverse dynamics, aka the joint torques according to the current state of the system, the desired joint accelerations and the external forces.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  /// \param[in] fext Vector of external forces expressed in the local frame of the joints (dim model.njoints)
  ///
  /// \return The desired joint torques stored in data.tau.
  ///
  inline const Eigen::VectorXd &
  rnea(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a,
       const container::aligned_vector<Force> & fext);
  
  ///
  /// \brief Computes the non-linear effects (Corriolis, centrifual and gravitationnal effects), also called the bias terms \f$ b(q,\dot{q}) \f$ of the Lagrangian dynamics:
  /// <CENTER> \f$ \begin{eqnarray} M \ddot{q} + b(q, \dot{q}) = \tau  \end{eqnarray} \f$ </CENTER> <BR>
  /// \note This function is equivalent to se3::rnea(model, data, q, v, 0).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The bias terms stored in data.nle.
  ///
  inline const Eigen::VectorXd &
  nonLinearEffects(const Model & model, Data & data,
                   const Eigen::VectorXd & q,
                   const Eigen::VectorXd & v);
  
  ///
  /// \brief Computes the generalized gravity contribution \f$ g(q) \f$ of the Lagrangian dynamics:
  /// <CENTER> \f$ \begin{eqnarray} M \ddot{q} + c(q, \dot{q}) + g(q) = \tau  \end{eqnarray} \f$ </CENTER> <BR>
  /// \note This function is equivalent to se3::rnea(model, data, q, 0, 0).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The bias terms stored in data.g.
  ///
  inline const Eigen::VectorXd &
  computeGeneralizedGravity(const Model & model, Data & data,
                            const Eigen::VectorXd & q);
  
  ///
  /// \brief Computes the Coriolis Matrix \f$ C(q,\dot{q}) \f$ of the Lagrangian dynamics:
  /// <CENTER> \f$ \begin{eqnarray} M \ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau  \end{eqnarray} \f$ </CENTER> <BR>
  /// \note In the previous equation, \f$ c(q, \dot{q}) = C(q, \dot{q})\dot{q} \f$.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The Coriolis matrix stored in data.C.
  ///
  inline const Eigen::MatrixXd &
  computeCoriolisMatrix(const Model & model, Data & data,
                        const Eigen::VectorXd & q,
                        const Eigen::VectorXd & v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/rnea.hxx"

#endif // ifndef __se3_rnea_hpp__
