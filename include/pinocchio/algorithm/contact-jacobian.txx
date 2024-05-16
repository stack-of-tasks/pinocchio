//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_contact_jacobian_txx__
#define __pinocchio_algorithm_contact_jacobian_txx__

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN

namespace pinocchio
{
  // extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getConstraintJacobian
  //   <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  // (const context::Model &, const context::Data &, const context::RigidConstraintModel &,
  // context::RigidConstraintData &, const Eigen::MatrixBase<context::Matrix6xs> &);

  // extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getConstraintsJacobian
  //   <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs, typename
  //   context::RigidConstraintModelVector::allocator_type, typename
  //   context::RigidConstraintDataVector::allocator_type>
  // (const context::Model &, context::Data &, const context::RigidConstraintModelVector &,
  // context::RigidConstraintDataVector &, const Eigen::MatrixBase<context::Matrix6xs> &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN

#endif // ifndef __pinocchio_algorithm_contact_jacobian_txx__
