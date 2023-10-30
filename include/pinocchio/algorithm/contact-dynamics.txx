//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_txx__
#define __pinocchio_algorithm_contact_dynamics_txx__

namespace pinocchio {

  extern template PINOCCHIO_DLLAPI void computeKKTContactDynamicMatrixInverse
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::MatrixXs, context::MatrixXs>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const context::Scalar &);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_txx__
