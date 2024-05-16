//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/centroidal-derivatives.hpp"

namespace pinocchio
{
  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
    computeCentroidalDynamicsDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
    getCentroidalDynamicsDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

  } // namespace impl
} // namespace pinocchio
