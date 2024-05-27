//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_centroidal_derivatives_txx__
#define __pinocchio_algorithm_centroidal_derivatives_txx__

namespace pinocchio
{
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
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

#endif // ifndef __pinocchio_algorithm_centroidal_derivatives_txx__
