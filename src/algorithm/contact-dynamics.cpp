//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_DYNAMICS

  #include "pinocchio/algorithm/contact-dynamics.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeKKTContactDynamicMatrixInverse<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::MatrixXs,
    context::MatrixXs>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const Eigen::MatrixBase<context::MatrixXs> &,
    const context::Scalar &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_CONTACT_DYNAMICS
