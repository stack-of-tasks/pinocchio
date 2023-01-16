//
// Copyright (c) 2017-2019 CNRS INRIA

#ifndef __pinocchio_RNEADerivativesSO_hpp__
#define __pinocchio_RNEADerivativesSO_hpp__

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
/// \tparam tensortype1 Type of the 3D-Tensor containing the SO partial
/// derivative with respect to the joint configuration vector. The elements of
/// Torque vector are along the 1st dim, and joint config along 2nd,3rd
/// dimensions.
/// \tparam tensortype2 Type of the 3D-Tensor containing the
/// Second-Order partial derivative with respect to the joint velocity vector.
/// The elements of Torque vector are along the 1st dim, and the velocity
/// along 2nd,3rd dimensions.
/// \tparam tensortype3 Type of the 3D-Tensor
/// containing the cross Second-Order partial derivative with respect to the
/// joint configuration and velocty vector. The elements of Torque vector are
/// along the 1st dim, and the config. vector along 2nd dimension, and velocity
/// along the third dimension.
///\tparam tensortype4 Type of the 3D-Tensor containing the cross Second-Order
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
/// \param[out] d2tau_dq2 Second-Order Partial derivative of the generalized
/// torque vector with respect to the joint configuration.
/// \param[out] d2tau_dv2 Second-Order Partial derivative of the generalized
/// torque vector with respect to the joint velocity
/// \param[out] d2tau_dqdv Cross Second-Order Partial derivative of the
/// generalized torque vector with respect to the joint configuration and
/// velocity.
/// \param[out] d2tau_dadq Cross Second-Order Partial derivative of
/// the generalized torque vector with respect to the joint configuration and
/// accleration.
/// \remarks d2tau_dq2,
/// d2tau_dv2, d2tau_dqdv and d2tau_dadq must be first initialized with zeros
/// (dtau_dq2.setZero(),etc). The storage order of the 3D-tensor derivatives is
/// important. For d2tau_dq2, the elements of generalized torque varies along
/// the rows, while elements of q vary along the columns and pages of the
/// tensor. For d2tau_dqdv, the elements of generalized torque varies along the
/// rows, while elements of v vary along the columns and elements of q along the
/// pages of the tensor. Hence, d2tau_dqdv is essentially d (d tau/dv)/dq, with
/// outer-most derivative representing the third dimension (pages) of the
/// tensor.  The tensor d2tau_dadq reduces down to dM/dq, and hence the elements
/// of q vary along the pages of the tensor. In other words, this tensor
/// derivative is d(d tau/da)/dq. All other remaining combinations of
/// second-order derivatives of generalized torque are zero.  \sa
///
template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2, typename tensortype1,
          typename tensortype2, typename tensortype3, typename tensortype4>
inline void computeRNEADerivativesSO(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a,
    const tensortype1 &d2tau_dq2, const tensortype2 &d2tau_dv2,
    const tensortype3 &d2tau_dqdv, const tensortype4 &d2tau_dadq);

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
/// \returns The results are stored in data.d2tau_dq, data.d2tau_dv,
/// data.d2tau_dqdv, and data.d2tau_dadq which respectively correspond to the
/// Second-Order partial derivatives of the joint torque vector with respect to
/// the joint configuration, velocity and cross Second-Order partial derivatives
/// with respect to configuration/velocity and configuration/acceleration
/// respectively.
///
/// \remarks d2tau_dq2,
/// d2tau_dv2, d2tau_dqdv and d2tau_dadq must be first initialized with zeros
/// (dtau_dq2.setZero(),etc). The storage order of the 3D-tensor derivatives is
/// important. For d2tau_dq2, the elements of generalized torque varies along
/// the rows, while elements of q vary along the columns and pages of the
/// tensor. For d2tau_dqdv, the elements of generalized torque varies along the
/// rows, while elements of v vary along the columns and elements of q along the
/// pages of the tensor. Hence, d2tau_dqdv is essentially d (d tau/dv)/dq, with
/// outer-most derivative representing the third dimension (pages) of the
/// tensor.  The tensor d2tau_dadq reduces down to dM/dq, and hence the elements
/// of q vary along the pages of the tensor. In other words, this tensor
/// derivative is d(d tau/da)/dq. All other remaining combinations of
/// second-order derivatives of generalized torque are zero.  \sa

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2>
inline void computeRNEADerivativesSO(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a) {
  (data.d2tau_dq).setZero();
  (data.d2tau_dv).setZero();
  (data.d2tau_dqdv).setZero();
  (data.d2tau_dadq).setZero();

  computeRNEADerivativesSO(model, data, q.derived(), v.derived(), a.derived(),
                           data.d2tau_dq, data.d2tau_dv, data.d2tau_dqdv,
                           data.d2tau_dadq);
}

} // namespace pinocchio

#include "pinocchio/algorithm/rnea-derivatives-SO.hxx"

#endif
