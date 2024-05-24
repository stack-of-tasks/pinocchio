//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_crba_txx__
#define __pinocchio_algorithm_crba_txx__

namespace pinocchio
{
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const context::MatrixXs &
  crba<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    Eigen::Ref<const context::VectorXs>>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
    const Convention convention);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_crba_txx__
