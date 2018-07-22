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

#ifndef __se3_kinematics_derivatives_hpp__
#define __se3_kinematics_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{
  
  ///
  /// \brief Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration
  ///        for any joint of the model.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  ///
  /// \remarks This function is similar to do a forwardKinematics(model,data,q,v) followed by a computeJointJacobians(model,data,q).
  ///          In addition, it computes the spatial velocity of the joint expressed in the world frame (see data.ov).
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void computeForwardKinematicsDerivatives(const ModelTpl<JointCollection> & model,
                                                  DataTpl<JointCollection> & data,
                                                  const Eigen::MatrixBase<ConfigVectorType> & q,
                                                  const Eigen::MatrixBase<TangentVectorType1> & v,
                                                  const Eigen::MatrixBase<TangentVectorType2> & a);
  
  ///
  /// \brief Computes the partial derivaties of the spatial velocity of a given with respect to
  ///        the joint configuration and velocity.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xOut1 Matrix6x containing the partial derivatives with respect to the joint configuration vector.
  /// \tparam Matrix6xOut2 Matrix6x containing the partial derivatives with respect to the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] partial_dq Partial derivative of the joint velociy w.r.t. \f$ q \f$.
  /// \param[out] partial_dq Partial derivative of the joint velociy w.r.t. \f$ \dot{q} \f$.
  ///
  template<typename JointCollection, typename Matrix6xOut1, typename Matrix6xOut2>
  inline void getJointVelocityDerivatives(const ModelTpl<JointCollection> & model,
                                          DataTpl<JointCollection> & data,
                                          const Model::JointIndex jointId,
                                          const ReferenceFrame rf,
                                          const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                          const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv);
  
  ///
  /// \brief Computes the partial derivaties of the spatial acceleration of a given with respect to
  ///        the joint configuration, velocity and acceleration.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///        It is important to notice that a direct outcome (for free) of this algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the spatial velocity with respect to the joint configuration vector.
  /// \tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint configuration vector.
  /// \tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint velocity vector.
  /// \tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId Index of the joint in model.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] v_partial_dq Partial derivative of the joint spatial velocity w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \dot{q} \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \ddot{q} \f$.
  ///
  template<typename JointCollection, typename Matrix6xOut1, typename Matrix6xOut2, typename Matrix6xOut3, typename Matrix6xOut4>
  inline void getJointAccelerationDerivatives(const ModelTpl<JointCollection> & model,
                                              DataTpl<JointCollection> & data,
                                              const Model::JointIndex jointId,
                                              const ReferenceFrame rf,
                                              const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
                                              const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da);

 

} // namespace se3 

#include "pinocchio/algorithm/kinematics-derivatives.hxx"

#endif // ifndef __se3_kinematics_derivatives_hpp__
