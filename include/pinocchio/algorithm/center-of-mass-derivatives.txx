//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_center_of_mass_derivatives_txx__
#define __pinocchio_algorithm_center_of_mass_derivatives_txx__

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getCenterOfMassVelocityDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix3x>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::Matrix3x> &);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_center_of_mass_derivatives_txx__
