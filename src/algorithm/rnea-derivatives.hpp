//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_rnea_derivatives_hpp__
#define __se3_rnea_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{

  ///
  /// \brief Computes the derivative of the generalized gravity contribution
  ///        with respect to the joint configuration.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[out] gravity_partial_dq Partial derivative of the generalized gravity vector with respect to the joint configuration.
  ///
  /// \remark gravity_partial_dq must be first initialized with zeros (gravity_partial_dq.setZero).
  ///
  /// \sa se3::computeGeneralizedGravity
  ///
  inline void
  computeGeneralizedGravityDerivatives(const Model & model, Data & data,
                                       const Eigen::VectorXd & q,
                                       Eigen::MatrixXd & gravity_partial_dq);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration and the joint velocity.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  /// \param[out] rnea_partial_dq Partial derivative of the generalized torque vector with respect to the joint configuration.
  /// \param[out] rnea_partial_dv Partial derivative of the generalized torque vector with respect to the joint velocity.
  /// \param[out] rnea_partial_da Partial derivative of the generalized torque vector with respect to the joint acceleration.
  ///
  /// \remark rnea_partial_dq, rnea_partial_dv and rnea_partial_da must be first initialized with zeros (rnea_partial_dq.setZero(),etc).
  ///         As for se3::crba, only the upper triangular part of rnea_partial_da is filled.
  ///
  /// \sa se3::rnea
  ///
  inline void
  computeRNEADerivatives(const Model & model, Data & data,
                         const Eigen::VectorXd & q,
                         const Eigen::VectorXd & v,
                         const Eigen::VectorXd & a,
                         Eigen::MatrixXd & rnea_partial_dq,
                         Eigen::MatrixXd & rnea_partial_dv,
                         Eigen::MatrixXd & rnea_partial_da);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration and the joint velocity.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  ///
  /// \returns The results are stored in data.dtau_dq, data.dtau_dv and data.M which respectively correspond
  ///          to the partial derivatives of the joint torque vector with respect to the joint configuration, velocity and acceleration.
  ///          And as for se3::crba, only the upper triangular part of data.M is filled.
  ///
  /// \sa se3::rnea, se3::crba, se3::cholesky::decompose
  ///
  inline void
  computeRNEADerivatives(const Model & model, Data & data,
                         const Eigen::VectorXd & q,
                         const Eigen::VectorXd & v,
                         const Eigen::VectorXd & a)
  {
    computeRNEADerivatives(model,data,q,v,a,
                           data.dtau_dq, data.dtau_dv, data.M);
  }


} // namespace se3 

#include "pinocchio/algorithm/rnea-derivatives.hxx"

#endif // ifndef __se3_rnea_derivatives_hpp__
