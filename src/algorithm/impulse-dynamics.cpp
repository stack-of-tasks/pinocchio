//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_IMPULSE_DYNAMICS

  #include "pinocchio/algorithm/impulse-dynamics.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::VectorXs &
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
