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
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam ReturnMatrixType Type of the matrix containing the partial derivative of the gravity vector with respect to the joint configuration vector.
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
  template<typename JointCollection, typename ConfigVectorType, typename ReturnMatrixType>
  inline void
  computeGeneralizedGravityDerivatives(const ModelTpl<JointCollection> & model,
                                       DataTpl<JointCollection> & data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q,
                                       const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration and the joint velocity.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  /// \tparam MatrixType1 Type of the matrix containing the partial derivative with respect to the joint configuration vector.
  /// \tparam MatrixType2 Type of the matrix containing the partial derivative with respect to the joint velocity vector.
  /// \tparam MatrixType3 Type of the matrix containing the partial derivative with respect to the joint acceleration vector.
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
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void
  computeRNEADerivatives(const ModelTpl<JointCollection> & model,
                         DataTpl<JointCollection> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                         const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                         const Eigen::MatrixBase<MatrixType3> & rnea_partial_da);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration and the joint velocity.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
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
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void
  computeRNEADerivatives(const ModelTpl<JointCollection> & model,
                         DataTpl<JointCollection> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    computeRNEADerivatives(model,data,q.derived(),v.derived(),a.derived(),
                           data.dtau_dq, data.dtau_dv, data.M);
  }


} // namespace se3 

#include "pinocchio/algorithm/rnea-derivatives.hxx"

#endif // ifndef __se3_rnea_derivatives_hpp__
