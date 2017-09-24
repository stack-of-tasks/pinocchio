//
// Copyright (c) 2015-2017 CNRS
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

#ifndef __se3_jacobian_hpp__
#define __se3_jacobian_hpp__

#include "pinocchio/multibody/model.hpp"

namespace se3
{
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  inline const Data::Matrix6x &
  computeJacobians(const Model & model,
                   Data & data,
                   const Eigen::VectorXd & q);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed either in the world frame or in the local frame of the joint.
  /// \note This jacobian is extracted from data.J. You have to run se3::computeJacobians before calling it.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<bool localFrame>
  void getJacobian(const Model & model,
                   const Data & data,
                   const Model::JointIndex jointId,
                   Data::Matrix6x & J);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed in the local frame of the joint. The result is stored in data.J.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] jointId The id of the joint.
  ///
  /// \return The Jacobian of the specific joint frame expressed in the local frame of the joint (matrix 6 x model.nv).
  ///
  inline const Data::Matrix6x &
  jacobian(const Model & model,
           Data & data,
           const Eigen::VectorXd & q,
           const Model::JointIndex jointId);

  
  ///
  /// \brief Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v
  ///        The result is accessible through data.dJ.
  ///
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  inline const Data::Matrix6x &
  computeJacobiansTimeVariation(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v);
  
  ///
  /// \brief Computes the Jacobian time variation of a specific joint frame expressed either in the world frame or in the local frame of the joint.
  /// \note This jacobian is extracted from data.dJ. You have to run se3::computeJacobiansTimeVariation before calling it.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<bool localFrame>
  void getJacobian(const Model & model,
                   const Data & data,
                   const Model::JointIndex jointId,
                   Data::Matrix6x & dJ);
  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */

#include "pinocchio/algorithm/jacobian.hxx"

#endif // ifndef __se3_jacobian_hpp__
