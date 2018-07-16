//
// Copyright (c) 2016,2018 CNRS
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

#ifndef __se3_joint_basic_visitors_hpp__
#define __se3_joint_basic_visitors_hpp__

#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace se3
{
  
  /**
   * @brief      Visit a JointModelVariant through CreateData visitor to create a JointDataVariant
   *
   * @param[in]  jmodel  The JointModelVariant we want to create a data for
   *
   * @return     The created JointDataVariant
   */
  template<typename JointCollection>
  inline typename JointCollection::JointDataVariant
  createData(const typename JointCollection::JointModelVariant & jmodel);

  
  /**
   * @brief      Visit a JointDataVariant and the corresponding JointModelVariant through JointCalcZeroOrderVisitor
   *             to compute the joint data kinematics at order zero
   *
   * @param[in]  jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param      jdata   The JointDataVariant we want to update
   * @param[in]  q       The full model's (in which the joint belongs to) configuration vector
   */
  template<typename JointCollection>
  inline void calc_zero_order(const JointModelTpl<JointCollection> & jmodel,
                              JointDataTpl<JointCollection> & jdata,
                              const Eigen::VectorXd & q);

  /**
   * @brief      Visit a JointDataVariant and the corresponding JointModelVariant through JointCalcFirstOrderVisitor
   *             to compute the joint data kinematics at order one
   *
   * @param[in]  jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param      jdata   The JointDataVariant we want to update
   * @param[in]  q       The full model's (in which the joint belongs to) configuration vector
   */
  template<typename JointCollection>
  inline void calc_first_order(const JointModelTpl<JointCollection> & jmodel,
                               JointDataTpl<JointCollection> & jdata,
                               const Eigen::VectorXd & q,
                               const Eigen::VectorXd & v);
  
  
  /**
   * @brief      Visit a JointDataVariant and the corresponding JointModelVariant through JointCalcAbaVisitor to
   * 
   *
   * @param[in]  jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param      jdata   The JointDataVariant we want to update
   * @param      I       Inertia matrix of the subtree following the jmodel in the kinematic chain as dense matrix
   * @param[in]  update_I  If I should be updated or not
   */
  template<typename JointCollection>
  inline void calc_aba(const JointModelTpl<JointCollection> & jmodel,
                       JointDataTpl<JointCollection> & jdata,
                       Inertia::Matrix6 & I,
                       const bool update_I);

  ///
  /// \brief Returns the finite difference increment of the joint model.
  ///
  /// \param[in] jmodel The model of the joint.
  ///
  /// \returns The finite diffrence increment.
  ///
  template<typename JointCollection>
  inline typename JointCollection::Scalar
  finiteDifferenceIncrement(const JointModelTpl<JointCollection> & jmodel);

  /**
   * @brief      Visit a JointModelVariant through JointNvVisitor to get the dimension of 
   *             the joint tangent space
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The dimension of joint tangent space
   */
  template<typename JointCollection>
  inline int nv(const JointModelTpl<JointCollection> & jmodel);


  
  /**
   * @brief      Visit a JointModelVariant through JointNqVisitor to get the dimension of 
   *             the joint configuration space
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The dimension of joint configuration space
   */
  template<typename JointCollection>
  inline int nq(const JointModelTpl<JointCollection> & jmodel);

  
  /**
   * @brief      Visit a JointModelVariant through JointIdxQVisitor to get the index in the full model configuration
   *             space corresponding to the first degree of freedom of the Joint
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index in the full model configuration space corresponding to the first
   *             degree of freedom of jmodel
   */
  template<typename JointCollection>
  inline int idx_q(const JointModelTpl<JointCollection> & jmodel);

  
  /**
   * @brief      Visit a JointModelVariant through JointIdxVVisitor to get the index in the full model tangent
   *             space corresponding to the first joint tangent space degree
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index in the full model tangent space corresponding to the first
   *             joint tangent space degree
   */
  template<typename JointCollection>
  inline int idx_v(const JointModelTpl<JointCollection> & jmodel);

  
  /**
   * @brief      Visit a JointModelVariant through JointIdVisitor to get the index of the joint in the kinematic chain
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index of the joint in the kinematic chain
   */
  template<typename JointCollection>
  inline JointIndex id(const JointModelTpl<JointCollection> & jmodel);

  /**
   * @brief      Visit a JointModelVariant through JointSetIndexesVisitor to set
   *             the indexes of the joint in the kinematic chain
   *
   * @param[in]  jmodel  The JointModelVariant
   * @param[in]  id      The index of joint in the kinematic chain
   * @param[in]  q       The index in the full model configuration space corresponding to the first degree of freedom
   * @param[in]  v       The index in the full model tangent space corresponding to the first joint tangent space degree
   *
   * @return     The index of the joint in the kinematic chain
   */
  template<typename JointCollection>
  inline void setIndexes(JointModelTpl<JointCollection> & jmodel, JointIndex id, int q,int v);


  /**
   * @brief      Visit a JointModelVariant through JointShortnameVisitor to get the shortname of the derived joint model
   *
   * @param      jmodel  The JointModelVariant we want the shortname of the type held in
   */
  template<typename JointCollection>
  inline std::string shortname(const JointModelTpl<JointCollection> & jmodel);
 
  
  //
  // Visitors on JointDatas
  //
  
  
  /**
   * @brief      Visit a JointDataVariant through JointConstraintVisitor to get the joint constraint 
   *             as a dense constraint
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The constraint dense corresponding to the joint derived constraint
   */
  template<typename JointCollection>
  inline ConstraintTpl<Eigen::Dynamic,typename JointCollection::Scalar, JointCollection::Options>
  constraint_xd(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointTransformVisitor to get the joint internal transform  (transform
   *             between the entry frame and the exit frame of the joint)
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The joint transform corresponding to the joint derived transform (sXp)
   */
  template<typename JointCollection>
  inline SE3Tpl<typename JointCollection::Scalar,JointCollection::Options>
  joint_transform(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointMotionVisitor to get the joint internal motion 
   *             as a dense motion
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The motion dense corresponding to the joint derived motion
   */
  template<typename JointCollection>
  inline MotionTpl<typename JointCollection::Scalar,JointCollection::Options>
  motion(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointBiasVisitor to get the joint bias
   *             as a dense motion
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The motion dense corresponding to the joint derived bias
   */
  template<typename JointCollection>
  inline MotionTpl<typename JointCollection::Scalar,JointCollection::Options>
  bias(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointUInertiaVisitor to get the U matrix of the inertia matrix 
   *             decomposition
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The U matrix of the inertia matrix decomposition
   */
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic,JointCollection::Options>
  u_inertia(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointDInvInertiaVisitor to get the D^{-1} matrix of the inertia matrix 
   *             decomposition
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The D^{-1} matrix of the inertia matrix decomposition
   */
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,Eigen::Dynamic,JointCollection::Options>
  dinv_inertia(const JointDataTpl<JointCollection> & jdata);

  /**
   * @brief      Visit a JointDataVariant through JointUDInvInertiaVisitor to get U*D^{-1} matrix of the inertia matrix 
   *             decomposition
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The U*D^{-1} matrix of the inertia matrix decomposition
   */
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic,JointCollection::Options>
  udinv_inertia(const JointDataTpl<JointCollection> & jdata);
  
} // namespace se3


/* --- Details -------------------------------------------------------------------- */
// Included later
// #include "pinocchio/multibody/joint/joint-basic-visitors.hxx"


#endif // ifndef __se3_joint_basic_visitors_hpp__
