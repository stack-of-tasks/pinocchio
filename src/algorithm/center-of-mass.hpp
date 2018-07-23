//
// Copyright (c) 2015-2016,2018 CNRS
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

#ifndef __se3_center_of_mass_hpp__
#define __se3_center_of_mass_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{
  
  ///
  /// \brief Computes the center of mass position of a given model according to a particular joint configuration.
  ///        The result is accessible through data.com[0] for the full body com and data.com[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::Vector3 &
  centerOfMass(const ModelTpl<JointCollection> & model,
               DataTpl<JointCollection> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const bool computeSubtreeComs = true);
  
  ///
  /// \brief Computes the center of mass position and velocity of a given model according to a particular joint configuration and velocity.
  ///        The result is accessible through data.com[0], data.vcom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<JointCollection>::Vector3 &
  centerOfMass(const ModelTpl<JointCollection> & model,
               DataTpl<JointCollection> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType> & v,
               const bool computeSubtreeComs = true);
  
  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to a particular joint configuration, velocity and acceleration.
  ///        The result is accessible through data.com[0], data.vcom[0], data.acom[0] for the full body com position, velocity and acceleation.
  ///        And data.com[i], data.vcom[i] and data.acom[i] for the subtree supported by joint i (expressed in the joint i frame).
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
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<JointCollection>::Vector3 &
  centerOfMass(const ModelTpl<JointCollection> & model,
               DataTpl<JointCollection> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType1> & v,
               const Eigen::MatrixBase<TangentVectorType2> & a,
               const bool computeSubtreeComs = true);
  
  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data and the template value parameters.
  ///        The result is accessible through data.com[0], data.vcom[0] and data.acom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] LEVEL if =0, compytes CoM position, if =1, also computes CoM velocity and if =2, also computes CoM acceleration.
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  template<typename JointCollection>
  inline void centerOfMass(const ModelTpl<JointCollection> & model,
                           DataTpl<JointCollection> & data,
                           const int LEVEL,
                           const bool computeSubtreeComs = true);
  
  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data.
  ///        The result is accessible through data.com[0], data.vcom[0] and data.acom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  template<typename JointCollection>
  inline void centerOfMass(const ModelTpl<JointCollection> & model,
                           DataTpl<JointCollection> & data,
                           const bool computeSubtreeComs)
  { centerOfMass(model,data,2,computeSubtreeComs); }
  
  ///
  /// \brief Computes both the jacobian and the the center of mass position of a given model according to a particular joint configuration.
  ///        The results are accessible through data.Jcom and data.com[0] and are both expressed in the world frame. In addition, the algorithm also computes the Jacobian of all the joints (\sa se3::computeJointJacobians).
  ///        And data.com[i] gives the center of mass of the subtree supported by joint i (expressed in the world frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  /// \param[in] updateKinematics If true, the algorithm updates first the geometry of the tree. Otherwise, it uses the current kinematics stored in data.
  ///
  /// \return The jacobian of center of mass position of the rigid body system expressed in the world frame (matrix 3 x model.nv).
  ///
  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::Matrix3x &
  jacobianCenterOfMass(const ModelTpl<JointCollection> & model,
                       DataTpl<JointCollection> & data,
                       const Eigen::MatrixBase<ConfigVectorType> & q,
                       const bool computeSubtreeComs = true,
                       const bool updateKinematics = true);

  /* If the CRBA has been run, then both COM and Jcom are easily available from
   * the mass matrix. Use the following methods to access them. In that case,
   * the COM subtrees (also easily available from CRBA data) are not
   * explicitely set. Use data.Ycrb[i].lever() to get them. */
  ///
  /// \brief Extracts the center of mass position from the joint space inertia matrix (also called the mass matrix).
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The center of mass position of the rigid body system expressed in the world frame (vector 3).
  ///
  template<typename JointCollection>
  inline const typename DataTpl<JointCollection>::Vector3 &
  getComFromCrba(const ModelTpl<JointCollection> & model,
                 DataTpl<JointCollection> & data);
  
  ///
  /// \brief Extracts both the jacobian of the center of mass (CoM), the total mass of the system and the CoM position from the joint space inertia matrix (also called the mass matrix).
  ///        The results are accessible through data.Jcom, data.mass[0] and data.com[0] and are both expressed in the world frame.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The jacobian of the CoM expressed in the world frame (matrix 3 x model.nv).
  ///
  /// \remark This extraction of inertial quantities is only valid for free-floating base systems.
  ///
  template<typename JointCollection>
  inline const typename DataTpl<JointCollection>::Matrix3x &
  getJacobianComFromCrba(const ModelTpl<JointCollection> & model,
                         DataTpl<JointCollection> & data);
  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/center-of-mass.hxx"

#endif // ifndef __se3_center_of_mass_hpp__
