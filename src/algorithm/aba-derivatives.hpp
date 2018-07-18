//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_aba_derivatives_hpp__
#define __se3_aba_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{
  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[out] aba_partial_dq Partial derivative of the generalized torque vector with respect to the joint configuration.
  /// \param[out] aba_partial_dv Partial derivative of the generalized torque vector with respect to the joint velocity.
  /// \param[out] aba_partial_dtau Partial derivative of the generalized torque vector with respect to the joint torque.
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia matrix.
  ///
  /// \sa se3::aba
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
           typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void computeABADerivatives(const ModelTpl<JointCollection> & model,
                                    DataTpl<JointCollection> & data,
                                    const Eigen::MatrixBase<ConfigVectorType> & q,
                                    const Eigen::MatrixBase<TangentVectorType1> & v,
                                    const Eigen::MatrixBase<TangentVectorType2> & tau,
                                    const Eigen::MatrixBase<MatrixType1> & aba_partial_dq,
                                    const Eigen::MatrixBase<MatrixType2> & aba_partial_dv,
                                    const Eigen::MatrixBase<MatrixType3> & aba_partial_dtau);
  
  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  ///
  /// \returns The results are stored in data.ddq_dq, data.ddq_dv and data.Minv which respectively correspond
  ///          to the partial derivatives of the joint acceleration vector with respect to the joint configuration, velocity and torque.
  ///          And as for se3::computeMinverse, only the upper triangular part of data.Minv is filled.
  ///
  /// \sa se3::aba and \sa se3::computeABADerivatives.
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void computeABADerivatives(const ModelTpl<JointCollection> & model,
                                    DataTpl<JointCollection> & data,
                                    const Eigen::MatrixBase<ConfigVectorType> & q,
                                    const Eigen::MatrixBase<TangentVectorType1> & v,
                                    const Eigen::MatrixBase<TangentVectorType2> & tau)
  {
    computeABADerivatives(model,data,q,v,tau,
                          data.ddq_dq,data.ddq_dv,data.Minv);
  }

} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/aba-derivatives.hxx"

#endif // ifndef __se3_aba_derivatives_hpp__
