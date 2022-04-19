//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/centroidal-derivatives.hpp"

namespace pinocchio {
namespace impl {
  template void computeCentroidalDynamicsDerivatives
    <context::Scalar, context::Options, JointCollectionDefaultTpl,
     Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
   const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

  template void getCentroidalDynamicsDerivatives
    <context::Scalar, context::Options, JointCollectionDefaultTpl,
     Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>, Eigen::Ref<context::Matrix6xs>>
  (const context::Model &, context::Data &,
   const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

} // namespace impl
} // namespace pinocchio
