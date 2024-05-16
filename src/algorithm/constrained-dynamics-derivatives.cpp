//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS_DERIVATIVES

  #include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeConstraintDynamicsDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs>(
    const context::Model &,
    context::Data &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    const ProximalSettingsTpl<context::Scalar> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeConstraintDynamicsDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs,
    context::MatrixXs>(
    const context::Model &,
    context::Data &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeConstraintDynamicsDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    const ProximalSettingsTpl<context::Scalar> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeConstraintDynamicsDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &);
} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS_DERIVATIVES
