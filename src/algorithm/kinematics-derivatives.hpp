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
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  ///
  /// \remarks This function is similar to do a forwardKinematics(model,data,q,v) followed by a computeJointJacobians(model,data,q).
  ///          In addition, it computes the spatial velocity of the joint expressed in the world frame (see data.ov).
  ///
  inline void computeForwardKinematicsDerivatives(const Model & model,
                                                  Data & data,
                                                  const Eigen::VectorXd & q,
                                                  const Eigen::VectorXd & v,
                                                  const Eigen::VectorXd & a);
  
  ///
  /// \brief Computes the partial derivaties of the spatial velocity of a given with respect to
  ///        the joint configuration and velocity.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId Index of the joint in model.
  /// \param[out] partial_dq Partial derivative of the joint velociy w.r.t. \f$ q \f$.
  /// \param[out] partial_dq Partial derivative of the joint velociy w.r.t. \f$ \dot{q} \f$.
  ///
  template<ReferenceFrame rf>
  inline void getJointVelocityDerivatives(const Model & model,
                                          Data & data,
                                          const Model::JointIndex jointId,
                                          Data::Matrix6x & v_partial_dq,
                                          Data::Matrix6x & v_partial_dv);
  
  ///
  /// \brief Computes the partial derivaties of the spatial acceleration of a given with respect to
  ///        the joint configuration, velocity and acceleration.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///        It is important to notice that a direct outcome (for free) of this algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId Index of the joint in model.
  /// \param[out] v_partial_dq Partial derivative of the joint spatial velocity w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \dot{q} \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \ddot{q} \f$.
  ///
  template<ReferenceFrame rf>
  inline void getJointAccelerationDerivatives(const Model & model,
                                              Data & data,
                                              const Model::JointIndex jointId,
                                              Data::Matrix6x & v_partial_dq,
                                              Data::Matrix6x & a_partial_dq,
                                              Data::Matrix6x & a_partial_dv,
                                              Data::Matrix6x & a_partial_da);

 

} // namespace se3 

#include "pinocchio/algorithm/kinematics-derivatives.hxx"

#endif // ifndef __se3_kinematics_derivatives_hpp__
