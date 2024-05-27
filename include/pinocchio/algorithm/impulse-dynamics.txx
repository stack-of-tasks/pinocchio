//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_txx__
#define __pinocchio_algorithm_impulse_dynamics_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_IMPULSE_DYNAMICS

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::VectorXs &
  impulseDynamics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs,
    typename context::RigidConstraintModelVector::allocator_type,
    typename context::RigidConstraintDataVector::allocator_type>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const context::RigidConstraintModelVector &,
    context::RigidConstraintDataVector &,
    const context::Scalar,
    const ProximalSettingsTpl<context::Scalar> &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_IMPULSE_DYNAMICS

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_txx__
