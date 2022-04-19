//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/energy.hpp"

namespace pinocchio {

  template context::Scalar computeKineticEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
namespace impl {
  template context::Scalar computeKineticEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl
  template context::Scalar computePotentialEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
namespace impl {
  template context::Scalar computePotentialEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl
  template context::Scalar computeMechanicalEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
namespace impl {
  template context::Scalar computeMechanicalEnergy
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl
} // namespace pinocchio
