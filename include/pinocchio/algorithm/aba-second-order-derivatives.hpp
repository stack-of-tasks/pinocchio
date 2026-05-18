//
// Copyright (c) 2018-2024 INRIA
//

#pragma once

// IWYU pragma: begin_keep
#include <Eigen/Core>

#include <cassert>
#include <cstddef>
#include <type_traits>
#include <vector>

#include "pinocchio/macros.hpp"
#include "pinocchio/eigen-common.hpp"
#include "pinocchio/fwd.hpp"

#include "pinocchio/math.hpp"

#include "pinocchio/spatial.hpp"

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"
// IWYU pragma: end_keep

namespace pinocchio
{
  ///
  /// \brief Computes the Second-Order partial derivatives of the Articulated Body
  /// Algorithm (forward dynamics) with respect to the joint configuration, the joint
  /// velocity and the joint torque.
  ///
  /// \tparam JointCollectionTpl Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  /// \tparam Tensor1 Type of the 3D-Tensor containing the SO partial derivative
  /// with respect to the joint configuration vector. The elements of the joint
  /// acceleration vector are along the 1st dim, and joint configuration along
  /// the 2nd and 3rd dimensions.
  /// \tparam Tensor2 Type of the 3D-Tensor containing the Second-Order partial
  /// derivative with respect to the joint velocity vector. The elements of the
  /// joint acceleration vector are along the 1st dim, and the velocity along
  /// the 2nd and 3rd dimensions.
  /// \tparam Tensor3 Type of the 3D-Tensor containing the cross Second-Order
  /// partial derivative with respect to the joint configuration and velocity
  /// vector. The elements of the joint acceleration vector are along the 1st
  /// dim, the velocity vector along the 2nd dimension, and the configuration
  /// vector along the 3rd dimension.
  /// \tparam Tensor4 Type of the 3D-Tensor containing the cross Second-Order
  /// partial derivative with respect to the joint torque and joint
  /// configuration vector. The elements of the joint acceleration vector are
  /// along the 1st dim, the torque vector along the 2nd dimension, and the
  /// configuration vector along the 3rd dimension.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  /// \param[out] d2ddq_dqdq Second-Order Partial derivative of the joint
  /// acceleration vector with respect to the joint configuration.
  /// \param[out] d2ddq_dvdv Second-Order Partial derivative of the joint
  /// acceleration vector with respect to the joint velocity.
  /// \param[out] d2ddq_dqdv Cross Second-Order Partial derivative of the joint
  /// acceleration vector with respect to the joint configuration and joint
  /// velocity.
  /// \param[out] d2ddq_dtaudq Cross Second-Order Partial derivative of the
  /// joint acceleration vector with respect to the joint torque and joint
  /// configuration.
  ///
  /// \remarks The output tensors are fully overwritten by this call, so no
  /// pre-initialization is required. The storage order of the 3D-tensor
  /// derivatives mirrors the convention used for the SO RNEA derivatives:
  /// rows index the output joint-acceleration component, columns index the
  /// inner derivative variable, and pages index the outer derivative
  /// variable. For example, d2ddq_dqdv equals d(dqdd/dv)/dq, so the velocity
  /// indices vary along the columns and the configuration indices vary along
  /// the pages.
  ///
  /// \note This algorithm reuses pinocchio::computeABADerivatives and
  /// pinocchio::ComputeRNEASecondOrderDerivatives. After the call, the
  /// first-order ABA derivatives are also available in data.ddq_dq, data.ddq_dv
  /// and data.Minv, and the second-order RNEA derivatives in data.d2tau_*.
  ///
  /// \sa pinocchio::ComputeRNEASecondOrderDerivatives,
  ///     pinocchio::computeABADerivatives.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    typename Tensor1,
    typename Tensor2,
    typename Tensor3,
    typename Tensor4>
  inline void ComputeABASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const Tensor1 & d2ddq_dqdq,
    const Tensor2 & d2ddq_dvdv,
    const Tensor3 & d2ddq_dqdv,
    const Tensor4 & d2ddq_dtaudq);

  ///
  /// \brief Computes the Second-Order partial derivatives of the Articulated
  /// Body Algorithm (forward dynamics) with respect to the joint configuration,
  /// the joint velocity and the joint torque.
  ///
  /// \tparam JointCollectionTpl Collection of Joint types.
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
  /// \note The results are stored in data.d2ddq_dqdq, data.d2ddq_dvdv,
  /// data.d2ddq_dqdv and data.d2ddq_dtaudq, which respectively correspond to
  /// the Second-Order partial derivatives of the joint acceleration vector
  /// with respect to the joint configuration, joint velocity, and the cross
  /// Second-Order partial derivatives with respect to (configuration,velocity)
  /// and (torque,configuration). The first-order ABA derivatives are also
  /// available in data.ddq_dq, data.ddq_dv and data.Minv, and the second-order
  /// RNEA derivatives in data.d2tau_*.
  ///
  /// \sa pinocchio::ComputeRNEASecondOrderDerivatives,
  ///     pinocchio::computeABADerivatives.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2>
  inline void ComputeABASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau)
  {
    (data.d2ddq_dqdq).setZero();
    (data.d2ddq_dvdv).setZero();
    (data.d2ddq_dqdv).setZero();
    (data.d2ddq_dtaudq).setZero();

    ComputeABASecondOrderDerivatives(
      model, data, q.derived(), v.derived(), tau.derived(), data.d2ddq_dqdq, data.d2ddq_dvdv,
      data.d2ddq_dqdv, data.d2ddq_dtaudq);
  }

} // namespace pinocchio

// IWYU pragma: begin_exports
#include "pinocchio/src/algorithm/aba-second-order-derivatives.hxx"
// IWYU pragma: end_exports
