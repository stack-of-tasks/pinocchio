//
// Copyright (c) 2018-2021 CNRS INRIA
//

#ifndef __pinocchio_algorithm_aba_derivatives_hpp__
#define __pinocchio_algorithm_aba_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include <type_traits>

namespace pinocchio
{
  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam MatrixType1 Type of the matrix containing the partial derivative with respect to the
  /// joint configuration vector. \tparam MatrixType2 Type of the matrix containing the partial
  /// derivative with respect to the joint velocity vector. \tparam MatrixType3 Type of the matrix
  /// containing the partial derivative with respect to the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[out] aba_partial_dq Partial derivative of the generalized torque vector with respect to
  /// the joint configuration. \param[out] aba_partial_dv Partial derivative of the generalized
  /// torque vector with respect to the joint velocity. \param[out] aba_partial_dtau Partial
  /// derivative of the generalized torque vector with respect to the joint torque.
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia
  /// matrix.
  ///
  /// \sa pinocchio::aba
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const Eigen::MatrixBase<MatrixType1> & aba_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & aba_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & aba_partial_dtau);
  ///
  /// \brief The derivatives of the Articulated-Body algorithm with external forces.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam MatrixType1 Type of the matrix containing the partial derivative with respect to the
  /// joint configuration vector. \tparam MatrixType2 Type of the matrix containing the partial
  /// derivative with respect to the joint velocity vector. \tparam MatrixType3 Type of the matrix
  /// containing the partial derivative with respect to the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] fext External forces expressed in the local frame of the joints (dim
  /// model.njoints). \param[out] aba_partial_dq Partial derivative of the generalized torque vector
  /// with respect to the joint configuration. \param[out] aba_partial_dv Partial derivative of the
  /// generalized torque vector with respect to the joint velocity. \param[out] aba_partial_dtau
  /// Partial derivative of the generalized torque vector with respect to the joint torque.
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia
  /// matrix.
  ///
  /// \sa pinocchio::aba
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
    const Eigen::MatrixBase<MatrixType1> & aba_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & aba_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & aba_partial_dtau);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  ///
  /// \returns The results are stored in data.ddq_dq, data.ddq_dv and data.Minv which respectively
  /// correspond
  ///          to the partial derivatives of the joint acceleration vector with respect to the joint
  ///          configuration, velocity and torque. And as for pinocchio::computeMinverse, only the
  ///          upper triangular part of data.Minv is filled.
  ///
  /// \sa pinocchio::aba and \sa pinocchio::computeABADerivatives.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  typename std::enable_if<
    ConfigVectorType::IsVectorAtCompileTime || TangentVectorType1::IsVectorAtCompileTime
      || TangentVectorType2::IsVectorAtCompileTime,
    void>::type
  computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm with external forces.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[in] fext External forces expressed in the local frame of the joints (dim
  /// model.njoints).
  ///
  /// \returns The results are stored in data.ddq_dq, data.ddq_dv and data.Minv which respectively
  /// correspond
  ///          to the partial derivatives of the joint acceleration vector with respect to the joint
  ///          configuration, velocity and torque. And as for pinocchio::computeMinverse, only the
  ///          upper triangular part of data.Minv is filled.
  ///
  /// \sa pinocchio::aba and \sa pinocchio::computeABADerivatives.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///        This function exploits the internal computations made in pinocchio::aba to
  ///        significantly reduced the computation burden.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam MatrixType1 Type of the matrix containing the partial derivative with respect to the
  /// joint configuration vector. \tparam MatrixType2 Type of the matrix containing the partial
  /// derivative with respect to the joint velocity vector. \tparam MatrixType3 Type of the matrix
  /// containing the partial derivative with respect to the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[out] aba_partial_dq Partial derivative of the generalized torque vector with respect to
  /// the joint configuration. \param[out] aba_partial_dv Partial derivative of the generalized
  /// torque vector with respect to the joint velocity. \param[out] aba_partial_dtau Partial
  /// derivative of the generalized torque vector with respect to the joint torque.
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia
  /// matrix.
  ///
  /// \sa pinocchio::aba
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  typename std::enable_if<
    !(MatrixType1::IsVectorAtCompileTime || MatrixType2::IsVectorAtCompileTime
      || MatrixType3::IsVectorAtCompileTime),
    void>::type
  computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<MatrixType1> & aba_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & aba_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & aba_partial_dtau);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm.
  ///        This function exploits the internal computations made in pinocchio::aba to
  ///        significantly reduced the computation burden.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \sa pinocchio::aba
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm with external forces.
  ///        This function exploits the internal computations made in pinocchio::aba to
  ///        significantly reduced the computation burden.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam MatrixType1 Type of the matrix containing the partial derivative with respect to the
  /// joint configuration vector. \tparam MatrixType2 Type of the matrix containing the partial
  /// derivative with respect to the joint velocity vector. \tparam MatrixType3 Type of the matrix
  /// containing the partial derivative with respect to the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] fext External forces expressed in the local frame of the joints (dim
  /// model.njoints). \param[out] aba_partial_dq Partial derivative of the generalized torque vector
  /// with respect to the joint configuration. \param[out] aba_partial_dv Partial derivative of the
  /// generalized torque vector with respect to the joint velocity. \param[out] aba_partial_dtau
  /// Partial derivative of the generalized torque vector with respect to the joint torque.
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia
  /// matrix.
  ///
  /// \sa pinocchio::aba
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext,
    const Eigen::MatrixBase<MatrixType1> & aba_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & aba_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & aba_partial_dtau);

  ///
  /// \brief The derivatives of the Articulated-Body algorithm with external forces.
  ///        This function exploits the internal computations made in pinocchio::aba to
  ///        significantly reduced the computation burden.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] fext External forces expressed in the local frame of the joints (dim
  /// model.njoints).
  ///
  /// \note aba_partial_dtau is in fact nothing more than the inverse of the joint space inertia
  /// matrix.
  ///
  /// \sa pinocchio::aba
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void computeABADerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const container::aligned_vector<ForceTpl<Scalar, Options>> & fext);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/aba-derivatives.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/aba-derivatives.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_aba_derivatives_hpp__
