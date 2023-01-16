//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_jacobian_hpp__
#define __pinocchio_jacobian_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function computes also the forwardKinematics of the model.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa pinocchio::getJointJacobian for doing this specific extraction.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  computeJointJacobians(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<ConfigVectorType> & q);
  
  ///
  /// \brief Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
  ///        The result is accessible through data.J. This function assumes that pinocchio::forwardKinematics has been called before.
  ///
  /// \note This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame. \sa pinocchio::getJointJacobian for doing this specific extraction.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  computeJointJacobians(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        DataTpl<Scalar,Options,JointCollectionTpl> & data);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed either in the world (rf = WORLD) frame or in the local frame (rf = LOCAL) of the joint.
  ///
  /// For the world frame W, the Jacobian \f${}^0 J_{0j}$ from the joint frame \f$j$ to the world frame $0$ is such that \f${}^0 v_{0j} = {}^0 J_{0j} \dot{q}$,
  /// where \f${}^0 v_{0j}$ is the spatial velocity of the joint frame. (When serialized to a 6D vector, the three linear coordinates are followed by the three
  /// angular coordinates).
  ///
  /// \note This jacobian is extracted from data.J. You have to run pinocchio::computeJointJacobians before calling it.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] J A reference to the Jacobian matrix where the results will be stored (dim 6 x model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6Like>
  inline void getJointJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex jointId,
                               const ReferenceFrame rf,
                               const Eigen::MatrixBase<Matrix6Like> & J);
  
  ///
  /// \brief Computes the Jacobian of a specific joint frame expressed in the local frame of the joint and store the result in the input argument J.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] jointId The id of the joint refering to model.joints[jointId].
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific joint frame expressed in the local frame of the joint (matrix 6 x model.nv).
  ///
  /// \remarks The result of this function is equivalent to call first computeJointJacobians(model,data,q) and then call getJointJacobian(model,data,jointId,LOCAL,J),
  ///         but forwardKinematics is not fully computed.
  ///         It is worth to call jacobian if you only need a single Jacobian for a specific joint. Otherwise, for several Jacobians, it is better
  ///         to call computeJointJacobians(model,data,q) followed by getJointJacobian(model,data,jointId,LOCAL,J) for each Jacobian.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6Like>
  inline void computeJointJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const Eigen::MatrixBase<ConfigVectorType> & q,
                                   const JointIndex jointId,
                                   const Eigen::MatrixBase<Matrix6Like> & J);
  
  ///
  /// \brief This function is now deprecated and has been renamed into computeJointJacobian. It will be removed in future releases of Pinocchio.
  ///
  /// \copydoc pinocchio::computeJointJacobian
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6Like>
  PINOCCHIO_DEPRECATED
  inline void jointJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const JointIndex jointId,
                            const Eigen::MatrixBase<Matrix6Like> & J)
  {
    computeJointJacobian(model,data,q,jointId,PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,J));
  }

  ///
  /// \brief Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v.
  ///        The result is accessible through data.dJ.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The full model Jacobian (matrix 6 x model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  computeJointJacobiansTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                     const Eigen::MatrixBase<ConfigVectorType> & q,
                                     const Eigen::MatrixBase<TangentVectorType> & v);
  
  ///
  /// \brief Computes the Jacobian time variation of a specific joint frame expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL) of the joint.
  /// \note This jacobian is extracted from data.dJ. You have to run pinocchio::computeJointJacobiansTimeVariation before calling it.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6Like>
  inline void getJointJacobianTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                            const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                            const JointIndex jointId,
                                            const ReferenceFrame rf,
                                            const Eigen::MatrixBase<Matrix6Like> & dJ);
  
} // namespace pinocchio 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */

#include "pinocchio/algorithm/jacobian.hxx"

#endif // ifndef __pinocchio_jacobian_hpp__
