//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_rnea_derivatives_hpp__
#define __pinocchio_rnea_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/container/aligned-vector.hpp"

namespace pinocchio
{

  ///
  /// \brief Computes the partial derivative of the generalized gravity contribution
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
  /// \remarks gravity_partial_dq must be first initialized with zeros (gravity_partial_dq.setZero).
  ///
  /// \sa pinocchio::computeGeneralizedGravity
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename ReturnMatrixType>
  inline void
  computeGeneralizedGravityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q,
                                       const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq);

  ///
  /// \brief Computes the partial derivative of the generalized gravity and external forces contributions (a.k.a static torque vector)
  ///        with respect to the joint configuration.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam ReturnMatrixType Type of the matrix containing the partial derivative of the gravity vector with respect to the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] fext External forces expressed in the local frame of the joints (dim model.njoints).
  /// \param[out] static_torque_partial_dq Partial derivative of the static torque vector with respect to the joint configuration.
  ///
  /// \remarks gravity_partial_dq must be first initialized with zeros (gravity_partial_dq.setZero).
  ///
  /// \sa pinocchio::computeGeneralizedTorque
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename ReturnMatrixType>
  inline void
  computeStaticTorqueDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const Eigen::MatrixBase<ConfigVectorType> & q,
                                 const container::aligned_vector< ForceTpl<Scalar,Options> > & fext,
                                 const Eigen::MatrixBase<ReturnMatrixType> & static_torque_partial_dq);
  
  ///
  /// \brief Computes the partial derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration, the joint velocity and the joint acceleration.
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
  /// \remarks rnea_partial_dq, rnea_partial_dv and rnea_partial_da must be first initialized with zeros (rnea_partial_dq.setZero(),etc).
  ///         As for pinocchio::crba, only the upper triangular part of rnea_partial_da is filled.
  ///
  /// \sa pinocchio::rnea
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                         const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                         const Eigen::MatrixBase<MatrixType3> & rnea_partial_da);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration, the joint velocity and the joint acceleration.
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
  /// \param[in] fext External forces expressed in the local frame of the joints (dim model.njoints).
  /// \param[out] rnea_partial_dq Partial derivative of the generalized torque vector with respect to the joint configuration.
  /// \param[out] rnea_partial_dv Partial derivative of the generalized torque vector with respect to the joint velocity.
  /// \param[out] rnea_partial_da Partial derivative of the generalized torque vector with respect to the joint acceleration.
  ///
  /// \remarks rnea_partial_dq, rnea_partial_dv and rnea_partial_da must be first initialized with zeros (rnea_partial_dq.setZero(),etc).
  ///         As for pinocchio::crba, only the upper triangular part of rnea_partial_da is filled.
  ///
  /// \sa pinocchio::rnea
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const container::aligned_vector< ForceTpl<Scalar,Options> > & fext,
                         const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                         const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                         const Eigen::MatrixBase<MatrixType3> & rnea_partial_da);
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration, the joint velocity and the joint acceleration.
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
  ///          As for pinocchio::crba, only the upper triangular part of data.M is filled.
  ///
  /// \sa pinocchio::rnea, pinocchio::crba, pinocchio::cholesky::decompose
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    computeRNEADerivatives(model,data,q.derived(),v.derived(),a.derived(),
                           data.dtau_dq, data.dtau_dv, data.M);
  }
  
  ///
  /// \brief Computes the derivatives of the Recursive Newton Euler Algorithms
  ///        with respect to the joint configuration, the joint velocity and the joint acceleration.
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
  /// \param[in] fext External forces expressed in the local frame of the joints (dim model.njoints).
  ///
  /// \returns The results are stored in data.dtau_dq, data.dtau_dv and data.M which respectively correspond
  ///          to the partial derivatives of the joint torque vector with respect to the joint configuration, velocity and acceleration.
  ///          As for pinocchio::crba, only the upper triangular part of data.M is filled.
  ///
  /// \sa pinocchio::rnea, pinocchio::crba, pinocchio::cholesky::decompose
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const container::aligned_vector< ForceTpl<Scalar,Options> > & fext)
  {
    computeRNEADerivatives(model,data,q.derived(),v.derived(),a.derived(),fext,
                           data.dtau_dq, data.dtau_dv, data.M);
  }


} // namespace pinocchio 

#include "pinocchio/algorithm/rnea-derivatives.hxx"

#endif // ifndef __pinocchio_rnea_derivatives_hpp__
