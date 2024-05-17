//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS

  #include "pinocchio/algorithm/constrained-dynamics.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void initConstraintDynamics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    typename context::RigidConstraintModelVector::allocator_type>(
    const context::Model &, context::Data &, const context::RigidConstraintModelVector &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::VectorXs &
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::VectorXs &
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::VectorXs & contactABA<
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
