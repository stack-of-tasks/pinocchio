//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/contact-dynamics.hpp"

namespace pinocchio {
  
  template void computeKKTContactDynamicMatrixInverse
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::MatrixXs, context::MatrixXs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const Eigen::MatrixBase<context::MatrixXs> &, const context::Scalar &);

} // namespace pinocchio 
