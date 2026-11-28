//
// Copyright (c) 2018-2024 INRIA
//

#pragma once

// IWYU pragma: begin_keep
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#include <cassert>
#include <cstddef>
#include <vector>

#include <boost/fusion/container/vector.hpp>

#include "pinocchio/macros.hpp"
#include "pinocchio/fwd.hpp"

#include "pinocchio/math.hpp"

#include "pinocchio/spatial.hpp"

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"

#include "pinocchio/algorithm/check.hpp"
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
  /// dim, the configuration vector along the 2nd dimension, and the velocity
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
  /// derivatives is the one of the SO RNEA derivatives and of Singh, Russell
  /// and Wensing, IEEE T-RO 2024, Eq. (18): rows index the joint acceleration,
  /// columns the first variable of the derivative name (inner derivative) and
  /// pages the second one (outer derivative). For example, d2ddq_dqdv equals
  /// d(d ddq/dq)/dv, i.e. d2ddq_dqdv(i,j,k) = d^2 ddq_i / (dq_j dv_k): the
  /// configuration indices vary along the columns and the velocity indices
  /// along the pages. Likewise, d2ddq_dtaudq(i,j,k) = d Minv(i,j) / dq_k. The
  /// order of the configuration derivatives matters in d2ddq_dqdq for joints
  /// whose configuration space is not a vector space (e.g. free-flyer or
  /// spherical joints).
  ///
  /// \note data must have been created with DataAllocationOption::ALL. This
  /// algorithm calls pinocchio::computeABADerivatives and
  /// pinocchio::ComputeRNEASecondOrderDerivatives, hence it modifies the
  /// following members of data:
  ///  - data.ddq, data.ddq_dq and data.ddq_dv contain the joint acceleration
  ///    and its first-order partial derivatives,
  ///  - data.Minv contains the full (symmetric) inverse of the joint space
  ///    inertia matrix,
  ///  - data.d2tau_dqdq, data.d2tau_dvdv, data.d2tau_dqdv and data.d2tau_dadq
  ///    contain the second-order RNEA derivatives evaluated at (q, v, data.ddq),
  ///  - data.oYcrb contains the composite rigid body inertias expressed in the
  ///    world frame,
  ///  - data.oa, data.of and data.ddJ are used as workspace: in particular,
  ///    data.oa does not contain the gravity contribution,
  ///  - data.Ftmp1, data.Ftmp2, data.MXtmp1 and data.MXtmp2 are used as
  ///    workspace.
  ///
  /// \warning Like the first-order derivatives it builds on, this algorithm
  /// assumes that the joint motion subspaces do not depend on the
  /// configuration. Joints such as SphericalZYX, Universal, Ellipsoid or
  /// composite joints are not supported.
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
  /// and (torque,configuration). See the overload above for their storage
  /// order.
  ///
  /// \note data must have been created with DataAllocationOption::ALL. This
  /// algorithm calls pinocchio::computeABADerivatives and
  /// pinocchio::ComputeRNEASecondOrderDerivatives, hence it modifies the
  /// following members of data:
  ///  - data.ddq, data.ddq_dq and data.ddq_dv contain the joint acceleration
  ///    and its first-order partial derivatives,
  ///  - data.Minv contains the full (symmetric) inverse of the joint space
  ///    inertia matrix,
  ///  - data.d2tau_dqdq, data.d2tau_dvdv, data.d2tau_dqdv and data.d2tau_dadq
  ///    contain the second-order RNEA derivatives evaluated at (q, v, data.ddq),
  ///  - data.oYcrb contains the composite rigid body inertias expressed in the
  ///    world frame,
  ///  - data.oa, data.of and data.ddJ are used as workspace: in particular,
  ///    data.oa does not contain the gravity contribution,
  ///  - data.Ftmp1, data.Ftmp2, data.MXtmp1 and data.MXtmp2 are used as
  ///    workspace.
  ///
  /// \warning Like the first-order derivatives it builds on, this algorithm
  /// assumes that the joint motion subspaces do not depend on the
  /// configuration. Joints such as SphericalZYX, Universal, Ellipsoid or
  /// composite joints are not supported.
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
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      data.allocation == DataAllocationOption::ALL,
      "data was created with DataAllocationOption::NO_TENSORS: tensors are not "
      "allocated");

    ComputeABASecondOrderDerivatives(
      model, data, q.derived(), v.derived(), tau.derived(), data.d2ddq_dqdq, data.d2ddq_dvdv,
      data.d2ddq_dqdv, data.d2ddq_dtaudq);
  }

} // namespace pinocchio

// IWYU pragma: begin_exports
#include "pinocchio/src/algorithm/aba-second-order-derivatives.hxx"
// IWYU pragma: end_exports
