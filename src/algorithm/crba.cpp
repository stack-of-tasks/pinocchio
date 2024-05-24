//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/crba.hpp"

namespace pinocchio
{
  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::MatrixXs & crba<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    Eigen::Ref<const context::VectorXs>>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
    const Convention convention);
} // namespace pinocchio
