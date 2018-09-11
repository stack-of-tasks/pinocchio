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

#ifndef __se3_jacobian_hpp__
#define __se3_jacobian_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function computes also the forwardKinematics of the model.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa se3::getJointJacobian for doing this specific extraction.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  inline const Data::Matrix6x &
  computeJointJacobians(const Model & model,
                        Data & data,
                        const Eigen::VectorXd & q);
  
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function computes also the forwardKinematics of the model.
  ///
  /// \deprecated This function is now deprecated. Please refer now to se3::computeJointJacobians for similar function with updated name.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa se3::getJointJacobian for doing this specific extraction.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  PINOCCHIO_DEPRECATED
  inline const Data::Matrix6x &
  computeJacobians(const Model & model,
                   Data & data,
                   const Eigen::VectorXd & q)
  { return computeJointJacobians(model,data,q); }
  
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function assumes that se3::forwardKinematics has been called before.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa se3::getJointJacobian for doing this specific extraction.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  inline const Data::Matrix6x &
  computeJointJacobians(const Model & model,
                        Data & data);
  
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function assumes that se3::forwardKinematics has been called before.
  ///
  /// \deprecated This function is now deprecated. Please refer now to se3::computeJointJacobians for similar function with updated name.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa se3::getJointJacobian for doing this specific extraction.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  PINOCCHIO_DEPRECATED
  inline const Data::Matrix6x &
  computeJacobians(const Model & model,
                   Data & data)
  { return computeJointJacobians(model,data); }
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed either in the world (rf = WORLD) frame or in the local frame (rf = LOCAL) of the joint.
  /// \note This jacobian is extracted from data.J. You have to run se3::computeJointJacobians before calling it.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<ReferenceFrame rf>
  void getJointJacobian(const Model & model,
                        const Data & data,
                        const Model::JointIndex jointId,
                        Data::Matrix6x & J);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed either in the world (rf = WORLD) frame or in the local frame (rf = LOCAL) of the joint.
  /// \note This jacobian is extracted from data.J. You have to run se3::computeJacobians before calling it.
  ///
  /// \deprecated This function is now deprecated. Please refer now to se3::getJointJacobian for similar function with updated name.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<ReferenceFrame rf>
  PINOCCHIO_DEPRECATED
  void getJacobian(const Model & model,
                   const Data & data,
                   const Model::JointIndex jointId,
                   Data::Matrix6x & J)
  { getJointJacobian<rf>(model,data,jointId,J); }
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed in the local frame of the joint and store the result in the input argument J.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] jointId The id of the joint refering to model.joints[jointId].
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific joint frame expressed in the local frame of the joint (matrix 6 x model.nv).
  ///
  /// \remark This function is equivalent to call first computeJacobians(model,data,q) and then call getJointJacobian<LOCAL>(model,data,jointId,J).
  ///         It is worth to call jacobian if you only need a single Jacobian for a specific joint. Otherwise, for several Jacobians, it is better
  ///         to call computeJacobians(model,data,q) followed by getJointJacobian<LOCAL>(model,data,jointId,J) for each Jacobian.
  ///
  inline void jointJacobian(const Model & model,
                            Data & data,
                            const Eigen::VectorXd & q,
                            const Model::JointIndex jointId,
                            Data::Matrix6x & J);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed in the local frame of the joint and store the result in the input argument J.
  ///
  /// \deprecated This function is now deprecated. Please refer now to se3::jointJacobian for similar function with updated name.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] jointId The id of the joint refering to model.joints[jointId].
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific joint frame expressed in the local frame of the joint (matrix 6 x model.nv).
  ///
  /// \remark This function is equivalent to call first computeJacobians(model,data,q) and then call getJointJacobian<LOCAL>(model,data,jointId,J).
  ///         It is worth to call jacobian if you only need a single Jacobian for a specific joint. Otherwise, for several Jacobians, it is better
  ///         to call computeJacobians(model,data,q) followed by getJointJacobian<LOCAL>(model,data,jointId,J) for each Jacobian.
  ///
  PINOCCHIO_DEPRECATED
  inline void jacobian(const Model & model,
                       Data & data,
                       const Eigen::VectorXd & q,
                       const Model::JointIndex jointId,
                       Data::Matrix6x & J)
  { jointJacobian(model,data,q,jointId,J); }
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed in the local frame of the joint. The result is stored in data.J.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] jointId The id of the joint refering to model.joints[jointId].
  ///
  /// \return The Jacobian of the specific joint frame expressed in the local frame of the joint (matrix 6 x model.nv).
  ///
  PINOCCHIO_DEPRECATED
  inline const Data::Matrix6x &
  jointJacobian(const Model & model,
                Data & data,
                const Eigen::VectorXd & q,
                const Model::JointIndex jointId)
  {
    data.J.setZero();
    jointJacobian(model,data,q,jointId,data.J);
    
    return data.J;
  }
  
  ///
  /// \brief Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v.
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
  computeJointJacobiansTimeVariation(const Model & model,
                                     Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v);
  
  ///
  /// \brief Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v.
  ///        The result is accessible through data.dJ.
  ///
  /// \deprecated This function is now deprecated. Please refer now to se3::computeJointJacobiansTimeVariation for similar function with updated name.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  PINOCCHIO_DEPRECATED
  inline const Data::Matrix6x &
  computeJacobiansTimeVariation(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v)
  { return computeJointJacobiansTimeVariation(model,data,q,v); }
  
  ///
  /// \brief Computes the Jacobian time variation of a specific joint frame expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL) of the joint.
  /// \note This jacobian is extracted from data.dJ. You have to run se3::computeJacobiansTimeVariation before calling it.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<ReferenceFrame rf>
  void getJointJacobianTimeVariation(const Model & model,
                                     const Data & data,
                                     const Model::JointIndex jointId,
                                     Data::Matrix6x & dJ);
  
  ///
  /// \brief Computes the Jacobian time variation of a specific joint frame expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL) of the joint.
  /// \note This jacobian is extracted from data.dJ. You have to run se3::computeJointJacobiansTimeVariation before calling it.
  /// \deprecated This function is now deprecated. Please refer now to se3::getJointJacobianTimeVariation for similar function with updated name.  
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<ReferenceFrame rf>
  PINOCCHIO_DEPRECATED
  void getJacobianTimeVariation(const Model & model,
                                const Data & data,
                                const Model::JointIndex jointId,
                                Data::Matrix6x & dJ)
  { getJointJacobianTimeVariation<rf>(model,data,jointId,dJ); }
  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */

#include "pinocchio/algorithm/jacobian.hxx"

#endif // ifndef __se3_jacobian_hpp__
