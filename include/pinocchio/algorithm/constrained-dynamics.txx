//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_constrained_dynamics_txx__
#define __pinocchio_algorithm_constrained_dynamics_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void initConstraintDynamics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type>(
    const context::Model &, context::Data &, const context::RigidConstraintModelVector &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
  constraintDynamics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs,
    context::VectorXs,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    ProximalSettingsTpl<context::Scalar> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
  constraintDynamics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs,
    context::VectorXs,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
  contactABA<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs,
    context::VectorXs,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS

#endif // ifndef __pinocchio_algorithm_constrained_dynamics_txx__
