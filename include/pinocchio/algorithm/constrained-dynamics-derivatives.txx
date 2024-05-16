//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_txx__
#define __pinocchio_algorithm_constrained_dynamics_derivatives_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS_DERIVATIVES

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

#endif // ifndef __pinocchio_algorithm_constrained_dynamics_derivatives_txx__
