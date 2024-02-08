//
// Copyright (c) 2017-2019 CNRS INRIA

#ifndef __pinocchio_algorithm_rnea_second_order_derivatives_hpp__
#define __pinocchio_algorithm_rnea_second_order_derivatives_hpp__

#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace pinocchio {
///
/// \brief Computes the Second-Order partial derivatives of the Recursive Newton
/// Euler Algorithm w.r.t the joint configuration, the joint velocity and the
/// joint acceleration.
///
/// \tparam JointCollection Collection of Joint types.
/// \tparam ConfigVectorType Type of the joint configuration vector.
/// \tparam TangentVectorType1 Type of the joint velocity vector.
/// \tparam TangentVectorType2 Type of the joint acceleration vector.
/// \tparam Tensor1 Type of the 3D-Tensor containing the SO partial
/// derivative with respect to the joint configuration vector. The elements of
/// Torque vector are along the 1st dim, and joint config along 2nd,3rd
/// dimensions.
/// \tparam Tensor2 Type of the 3D-Tensor containing the
/// Second-Order partial derivative with respect to the joint velocity vector.
/// The elements of Torque vector are along the 1st dim, and the velocity
/// along 2nd,3rd dimensions.
/// \tparam Tensor3 Type of the 3D-Tensor
/// containing the cross Second-Order partial derivative with respect to the
/// joint configuration and velocty vector. The elements of Torque vector are
/// along the 1st dim, and the config. vector along 2nd dimension, and velocity
/// along the third dimension.
///\tparam Tensor4 Type of the 3D-Tensor containing the cross Second-Order
/// partial derivative with respect to the joint configuration and acceleration
/// vector. This is also the First-order partial derivative of Mass-Matrix (M)
/// with respect to configuration vector. The elements of Torque vector are
/// along the 1st dim, and the acceleration vector along 2nd dimension, while
/// configuration along the third dimension.
///
/// \param[in] model The model structure of the rigid body system.
/// \param[in] data The data structure of the rigid body system.
/// \param[in] q The joint configuration vector (dim model.nq).
/// \param[in] v The joint velocity vector (dim model.nv).
/// \param[in] a The joint acceleration vector (dim model.nv).
/// \param[out] d2tau_dqdq Second-Order Partial derivative of the generalized
/// torque vector with respect to the joint configuration.
/// \param[out] d2tau_dvdv Second-Order Partial derivative of the generalized
/// torque vector with respect to the joint velocity
/// \param[out] dtau_dqdv Cross Second-Order Partial derivative of the
/// generalized torque vector with respect to the joint configuration and
/// velocity.
/// \param[out] dtau_dadq Cross Second-Order Partial derivative of
/// the generalized torque vector with respect to the joint configuration and
/// accleration.
/// \remarks d2tau_dqdq,
/// d2tau_dvdv, dtau_dqdv and dtau_dadq must be first initialized with zeros
/// (d2tau_dqdq.setZero(), etc). The storage order of the 3D-tensor derivatives is
/// important. For d2tau_dqdq, the elements of generalized torque varies along
/// the rows, while elements of q vary along the columns and pages of the
/// tensor. For dtau_dqdv, the elements of generalized torque varies along the
/// rows, while elements of v vary along the columns and elements of q along the
/// pages of the tensor. Hence, dtau_dqdv is essentially d (d tau/dq)/dv, with
/// outer-most derivative representing the third dimension (pages) of the
/// tensor.  The tensor dtau_dadq reduces down to dM/dq, and hence the elements
/// of q vary along the pages of the tensor. In other words, this tensor
/// derivative is d(d tau/da)/dq. All other remaining combinations of
/// second-order derivatives of generalized torque are zero.  \sa
///
template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2, typename Tensor1,
          typename Tensor2, typename Tensor3, typename Tensor4>
inline void ComputeRNEASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a,
    const Tensor1 &d2tau_dqdq, const Tensor2 &d2tau_dvdv,
    const Tensor3 &dtau_dqdv, const Tensor4 &dtau_dadq);

///
/// \brief Computes the Second-Order partial derivatives of the Recursive Newton
/// Euler Algorithms
///        with respect to the joint configuration, the joint velocity and the
///        joint acceleration.
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
/// \returns The results are stored in data.d2tau_dqdq, data.d2tau_dvdv,
/// data.d2tau_dqdv, and data.d2tau_dadq which respectively correspond to the
/// Second-Order partial derivatives of the joint torque vector with respect to
/// the joint configuration, velocity and cross Second-Order partial derivatives
/// with respect to configuration/velocity and configuration/acceleration
/// respectively.
///
/// \remarks d2tau_dqdq,
/// d2tau_dvdv2, d2tau_dqdv and d2tau_dadq must be first initialized with zeros
/// (d2tau_dqdq.setZero(),etc). The storage order of the 3D-tensor derivatives is
/// important. For d2tau_dqdq, the elements of generalized torque varies along
/// the rows, while elements of q vary along the columns and pages of the
/// tensor. For d2tau_dqdv, the elements of generalized torque varies along the
/// rows, while elements of v vary along the columns and elements of q along the
/// pages of the tensor. Hence, d2tau_dqdv is essentially d (d tau/dq)/dv, with
/// outer-most derivative representing the third dimension (pages) of the
/// tensor.  The tensor d2tau_dadq reduces down to dM/dq, and hence the elements
/// of q vary along the pages of the tensor. In other words, this tensor
/// derivative is d(d tau/da)/dq. All other remaining combinations of
/// second-order derivatives of generalized torque are zero.  \sa

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2>
inline void ComputeRNEASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a) {
  (data.d2tau_dqdq).setZero();
  (data.d2tau_dvdv).setZero();
  (data.d2tau_dqdv).setZero();
  (data.d2tau_dadq).setZero();

  ComputeRNEASecondOrderDerivatives(model, data, q.derived(), v.derived(), a.derived(),
                           data.d2tau_dqdq, data.d2tau_dvdv, data.d2tau_dqdv,
                           data.d2tau_dadq);
}

} // namespace pinocchio

#include "pinocchio/algorithm/rnea-second-order-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_rnea_second_order_derivatives_hpp__
