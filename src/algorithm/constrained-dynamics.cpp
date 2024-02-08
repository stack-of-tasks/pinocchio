//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/constrained-dynamics.hpp"

namespace pinocchio {

    template void initConstraintDynamics
      <context::Scalar, context::Options, JointCollectionDefaultTpl, typename context::RigidConstraintModelVector::allocator_type>
    (const context::Model &, context::Data &, const context::RigidConstraintModelVector &);

    template const context::VectorXs & constraintDynamics
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs, typename context::RigidConstraintModelVector::allocator_type, typename context::RigidConstraintDataVector::allocator_type>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::RigidConstraintModelVector &, context::RigidConstraintDataVector &, ProximalSettingsTpl<context::Scalar> &);

    template const context::VectorXs & constraintDynamics
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs, typename context::RigidConstraintModelVector::allocator_type, typename context::RigidConstraintDataVector::allocator_type>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::RigidConstraintModelVector &, context::RigidConstraintDataVector &);

    template const context::VectorXs & contactABA
      <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs, typename context::RigidConstraintModelVector::allocator_type, typename context::RigidConstraintDataVector::allocator_type>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::RigidConstraintModelVector &, context::RigidConstraintDataVector &);
} // namespace pinocchio
