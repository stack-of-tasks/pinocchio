//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/impulse-dynamics.hpp"

namespace pinocchio {

  template const context::VectorXs & impulseDynamics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, typename context::RigidConstraintModelVector::allocator_type, typename context::RigidConstraintDataVector::allocator_type>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const context::RigidConstraintModelVector &, context::RigidConstraintDataVector &, const context::Scalar, const ProximalSettingsTpl<context::Scalar> &);
} // namespace pinocchio
