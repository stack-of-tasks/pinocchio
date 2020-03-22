//
// Copyright (c) 2016,2018 CNRS
//

#ifndef __pinocchio_joint_basic_visitors_hpp__
#define __pinocchio_joint_basic_visitors_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{
  
  /**
   * @brief      Visit a JointModelTpl through CreateData visitor to create a JointDataTpl.
   *
   * @tparam JointCollection    Collection of Joint types.
   *
   * @param[in]  jmodel  The JointModelTpl we want to create a data for.
   *
   * @return     The created JointDataTpl
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointDataTpl<Scalar,Options,JointCollectionTpl>
  createData(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  
  /**
   * @brief      Visit a JointModelTpl and the corresponding JointDataTpl through JointCalcZeroOrderVisitor
   *             to compute the joint data kinematics at order zero.
   *
   * @tparam JointCollection    Collection of Joint types.
   * @tparam ConfigVectorType   Type of the joint configuration vector.
   *
   * @param[in]  jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param      jdata   The JointDataVariant we want to update
   * @param[in]  q       The full model's (in which the joint belongs to) configuration vector
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType>
  inline void calc_zero_order(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                              JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                              const Eigen::MatrixBase<ConfigVectorType> & q);

  /**
   * @brief      Visit a JointModelTpl and the corresponding JointDataTpl through JointCalcFirstOrderVisitor
   *             to compute the joint data kinematics at order one.
   *
   * @tparam JointCollection    Collection of Joint types.
   * @tparam ConfigVectorType   Type of the joint configuration vector.
   * @tparam TangentVectorType  Type of the joint velocity vector.
   *
   * @param[in]  jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param      jdata   The JointDataVariant we want to update
   * @param[in]  q       The full model's (in which the joint belongs to) configuration vector
   * @param[in]  v       The full model's (in which the joint belongs to) velocity vector
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline void calc_first_order(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                               JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                               const Eigen::MatrixBase<ConfigVectorType> & q,
                               const Eigen::MatrixBase<TangentVectorType> & v);
  
  
  /**
   * @brief      Visit a JointModelTpl and the corresponding JointDataTpl through JointCalcAbaVisitor to.
   *
   * @tparam JointCollection    Collection of Joint types.
   * @tparam Matrix6Type        A matrix 6x6 like Eigen container.
   *
   * @param[in]         jmodel  The corresponding JointModelVariant to the JointDataVariant we want to update
   * @param[inout]      jdata   The JointDataVariant we want to update
   * @param[inout]      I       Inertia matrix of the subtree following the jmodel in the kinematic chain as dense matrix
   * @param[in]         update_I  If I should be updated or not
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename Matrix6Type>
  inline void calc_aba(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                       JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                       const Eigen::MatrixBase<Matrix6Type> & I,
                       const bool update_I);

  /**
   * @brief      Visit a JointModelTpl through JointNvVisitor to get the dimension of
   *             the joint tangent space
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The dimension of joint tangent space
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nv(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);


  
  /**
   * @brief      Visit a JointModelTpl through JointNqVisitor to get the dimension of
   *             the joint configuration space
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The dimension of joint configuration space
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nq(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  
  /**
   * @brief      Visit a JointModelTpl through JointIdxQVisitor to get the index in the full model configuration
   *             space corresponding to the first degree of freedom of the Joint
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index in the full model configuration space corresponding to the first
   *             degree of freedom of jmodel
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_q(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  
  /**
   * @brief      Visit a JointModelTpl through JointIdxVVisitor to get the index in the full model tangent
   *             space corresponding to the first joint tangent space degree
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index in the full model tangent space corresponding to the first
   *             joint tangent space degree
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_v(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  
  /**
   * @brief      Visit a JointModelTpl through JointIdVisitor to get the index of the joint in the kinematic chain
   *
   * @param[in]  jmodel  The JointModelVariant
   *
   * @return     The index of the joint in the kinematic chain
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointIndex id(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  /**
   * @brief      Visit a JointModelTpl through JointSetIndexesVisitor to set
   *             the indexes of the joint in the kinematic chain
   *
   * @param[in]  jmodel  The JointModelVariant
   * @param[in]  id      The index of joint in the kinematic chain
   * @param[in]  q       The index in the full model configuration space corresponding to the first degree of freedom
   * @param[in]  v       The index in the full model tangent space corresponding to the first joint tangent space degree
   *
   * @return     The index of the joint in the kinematic chain
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline void setIndexes(JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel, JointIndex id, int q,int v);


  /**
   * @brief      Visit a JointModelTpl through JointShortnameVisitor to get the shortname of the derived joint model
   *
   * @param      jmodel  The JointModelVariant we want the shortname of the type held in
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline std::string shortname(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);
  
  /**
   * @brief      Visit a JointModelTpl<Scalar,...> to cast it into JointModelTpl<NewScalar,...>
   *
   * @tparam     NewScalar new scalar type of of the JointModelTpl
   *
   * @param[in]  jmodel  The joint model to cast.
   *
   * @return     A new JointModelTpl<NewScalar,...> casted from JointModelTpl<Scalar,...>.
   */
  template<typename NewScalar, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  typename CastType< NewScalar,JointModelTpl<Scalar,Options,JointCollectionTpl> >::type
  cast_joint(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel);

  
  //
  // Visitors on JointDatas
  //
  
  
  /**
   * @brief      Visit a JointDataVariant through JointConstraintVisitor to get the joint constraint 
   *             as a dense constraint
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The constraint dense corresponding to the joint derived constraint
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline ConstraintTpl<Eigen::Dynamic,Scalar,Options>
  constraint_xd(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointTransformVisitor to get the joint internal transform  (transform
   *             between the entry frame and the exit frame of the joint).
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The joint transform corresponding to the joint derived transform (sXp)
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline SE3Tpl<Scalar,Options>
  joint_transform(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointMotionVisitor to get the joint internal motion
   *             as a dense motion.
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The motion dense corresponding to the joint derived motion
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar,Options>
  motion(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointBiasVisitor to get the joint bias
   *             as a dense motion.
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The motion dense corresponding to the joint derived bias
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar,Options>
  bias(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointUInertiaVisitor to get the U matrix of the inertia matrix
   *             decomposition.
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The U matrix of the inertia matrix decomposition
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options>
  u_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointDInvInertiaVisitor to get the D^{-1} matrix of the inertia matrix
   *             decomposition.
   *
   * @param[in]  jdata  The jdata
   *
   * @return     The D^{-1} matrix of the inertia matrix decomposition
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options>
  dinv_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);

  /**
   * @brief      Visit a JointDataTpl through JointUDInvInertiaVisitor to get U*D^{-1} matrix of the inertia matrix
   *             decomposition.
   *
   * @param[in]  jdata  The joint data to visit.
   *
   * @return     The U*D^{-1} matrix of the inertia matrix decomposition
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options>
  udinv_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata);
  
} // namespace pinocchio


/* --- Details -------------------------------------------------------------------- */
// Included later
// #include "pinocchio/multibody/joint/joint-basic-visitors.hxx"


#endif // ifndef __pinocchio_joint_basic_visitors_hpp__
